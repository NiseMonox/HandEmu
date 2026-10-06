#include "gestures.h"

#include <algorithm>

namespace handemu
{

namespace
{

// Parent of each bone in the SteamVR hand skeleton (see the OpenVR "Hand Skeleton" docs).
const int kParent[ kBoneCount ] = {
	-1, 0,             // root, wrist
	1, 2, 3, 4,        // thumb 0..3 (3 = tip)
	1, 6, 7, 8, 9,     // index 0..4 (4 = tip)
	1, 11, 12, 13, 14, // middle
	1, 16, 17, 18, 19, // ring
	1, 21, 22, 23, 24, // pinky
	0, 0, 0, 0, 0,     // aux bones hang off the root
};

const int kThumbTip = 5;
const int kFingerTip[ 4 ] = { 10, 15, 20, 25 };
// First bone of the proximal phalanx, i.e. the knuckle joint.
const int kFingerKnuckle[ 4 ] = { 7, 12, 17, 22 };

// Tip-to-knuckle distance over finger length: ~1.0 straight, ~0.45 in a tight fist.
const double kRatioStraight = 0.92;
const double kRatioCurled = 0.45;

} // namespace

void ComputeJointPositions( const vr::VRBoneTransform_t *bones, Vec3 out[ kBoneCount ] )
{
	Quat rot[ kBoneCount ];
	for ( uint32_t i = 0; i < kBoneCount; i++ )
	{
		const vr::VRBoneTransform_t &b = bones[ i ];
		Vec3 local{ b.position.v[ 0 ], b.position.v[ 1 ], b.position.v[ 2 ] };
		Quat localRot = NormalizedQ( FromVR( b.orientation ) );
		int parent = kParent[ i ];
		if ( parent < 0 )
		{
			out[ i ] = local;
			rot[ i ] = localRot;
		}
		else
		{
			out[ i ] = out[ parent ] + Rotate( rot[ parent ], local );
			rot[ i ] = rot[ parent ] * localRot;
		}
	}
}

void GestureRecognizer::Reset()
{
	m_hasSample = false;
	m_pinch = kFingerNone;
	m_pinchMin = 0;
	m_fist = false;
	m_fistCandidateFrames = 0;
	m_near = false;
	m_joyActive = false;
	m_out = GestureOutput{};
	m_dbg = GestureDebug{};
}

void GestureRecognizer::Update( const vr::VRBoneTransform_t *bones, const GestureConfig &cfg,
	const Vec3 &handPos, bool handPosValid, const Vec3 &viewRight, const Vec3 &viewForward )
{
	Vec3 joints[ kBoneCount ];
	ComputeJointPositions( bones, joints );

	float dist[ 4 ];
	float curl[ 4 ];
	for ( int f = 0; f < 4; f++ )
	{
		dist[ f ] = (float)Distance( joints[ kThumbTip ], joints[ kFingerTip[ f ] ] );

		int k = kFingerKnuckle[ f ];
		double length = Distance( joints[ k ], joints[ k + 1 ] ) + Distance( joints[ k + 1 ], joints[ k + 2 ] ) + Distance( joints[ k + 2 ], joints[ k + 3 ] );
		double ratio = length > 1e-6 ? Distance( joints[ k ], joints[ k + 3 ] ) / length : 1.0;
		curl[ f ] = (float)Clamp01( ( kRatioStraight - ratio ) / ( kRatioStraight - kRatioCurled ) );
	}

	float a = std::clamp( cfg.smoothing, 0.05f, 1.0f );
	for ( int f = 0; f < 4; f++ )
	{
		if ( !m_hasSample )
		{
			m_dist[ f ] = dist[ f ];
			m_curl[ f ] = curl[ f ];
		}
		else
		{
			m_dist[ f ] += a * ( dist[ f ] - m_dist[ f ] );
			m_curl[ f ] += a * ( curl[ f ] - m_curl[ f ] );
		}
	}
	m_hasSample = true;

	const float *d = m_dist;
	float fistScore = ( m_curl[ kFingerMiddle ] + m_curl[ kFingerRing ] + m_curl[ kFingerPinky ] ) / 3.0f;

	if ( m_pinch != kFingerNone && d[ m_pinch ] > cfg.pinchOff )
		m_pinch = kFingerNone;

	// Fist and pinch latch: whichever registers first holds until it is released, so a pinch
	// that brushes the curled fingers never turns into a grab and vice versa.
	if ( m_fist )
	{
		m_fist = fistScore > cfg.fistOff;
		m_fistCandidateFrames = 0;
	}
	else
	{
		bool candidate = m_pinch == kFingerNone && fistScore > cfg.fistOn && d[ kFingerIndex ] > cfg.pinchOn &&
			( m_curl[ kFingerIndex ] > cfg.fistIndexOn || d[ kFingerIndex ] < cfg.fistThumbNear );
		m_fistCandidateFrames = candidate ? m_fistCandidateFrames + 1 : 0;
		m_fist = m_fistCandidateFrames >= std::max( 1, cfg.fistFrames );
	}

	if ( m_pinch == kFingerNone && !m_fist )
	{
		int best = kFingerIndex;
		for ( int f = 1; f < 4; f++ )
		{
			if ( d[ f ] < d[ best ] )
				best = f;
		}
		// The index finger wins unless another finger is clearly closer to the thumb.
		if ( best != kFingerIndex && d[ kFingerIndex ] - d[ best ] < cfg.fingerMargin )
			best = kFingerIndex;

		if ( d[ best ] < cfg.pinchOn )
		{
			bool clear = true;
			if ( best != kFingerIndex )
			{
				for ( int f = 1; f < 4; f++ )
				{
					if ( f != best && d[ f ] - d[ best ] < cfg.fingerMargin * 0.5f )
						clear = false;
				}
			}
			if ( clear )
			{
				m_pinch = best;
				m_pinchMin = d[ best ];
			}
		}
	}
	if ( m_pinch != kFingerNone )
		m_pinchMin = std::min( m_pinchMin, d[ m_pinch ] );

	if ( m_fist )
		m_near = false;
	else if ( m_near )
		m_near = d[ kFingerIndex ] < cfg.nearOff;
	else
		m_near = d[ kFingerIndex ] < cfg.nearOn;

	GestureOutput o;

	// Partial values stay well below typical click thresholds, so only a real pinch/fist clicks.
	const double kApproachMax = 0.45;

	o.triggerClick = m_pinch == kFingerIndex;
	o.triggerTouch = o.triggerClick || m_near;
	if ( o.triggerClick )
		o.triggerValue = 1.0f;
	else if ( !m_fist )
		o.triggerValue = (float)( kApproachMax * Clamp01( ( cfg.analogFar - d[ kFingerIndex ] ) / std::max( 1e-4f, cfg.analogFar - cfg.pinchOn ) ) );

	o.gripClick = m_fist;
	o.gripValue = m_fist ? 1.0f : (float)( kApproachMax * Clamp01( ( fistScore - 0.25 ) / std::max( 1e-3, cfg.fistOn - 0.25 ) ) );
	o.gripTouch = o.gripClick || o.gripValue > 0.2f;

	o.aClick = o.aTouch = m_pinch == kFingerMiddle;
	o.bClick = o.bTouch = m_pinch == kFingerPinky || ( !cfg.joystickEnable && m_pinch == kFingerRing );

	if ( cfg.joystickEnable && m_pinch == kFingerRing && handPosValid )
	{
		if ( !m_joyActive )
		{
			m_joyActive = true;
			m_joyStart = handPos;
		}
		Vec3 delta = handPos - m_joyStart;
		double radius = std::max( 0.01, (double)cfg.joystickRadius );
		double x = Dot( delta, viewRight ) / radius;
		double y = Dot( delta, viewForward ) / radius;
		double mag = std::sqrt( x * x + y * y );
		if ( mag > 1.0 )
		{
			x /= mag;
			y /= mag;
			mag = 1.0;
		}
		double deadzone = std::clamp( (double)cfg.joystickDeadzone, 0.0, 0.9 );
		if ( mag < deadzone )
		{
			x = y = 0;
		}
		else
		{
			double scale = ( mag - deadzone ) / ( 1.0 - deadzone ) / mag;
			x *= scale;
			y *= scale;
		}
		o.joyX = (float)x;
		o.joyY = (float)y;
		o.joyTouch = true;
	}
	else
	{
		m_joyActive = false;
	}

	m_out = o;

	for ( int f = 0; f < 4; f++ )
	{
		m_dbg.dist[ f ] = m_dist[ f ];
		m_dbg.curl[ f ] = m_curl[ f ];
	}
	m_dbg.fistScore = fistScore;
	m_dbg.pinch = m_pinch;
	m_dbg.pinchMin = m_pinchMin;
	m_dbg.fist = m_fist;
}

} // namespace handemu
