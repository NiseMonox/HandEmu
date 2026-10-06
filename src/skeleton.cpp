#include "skeleton.h"

#include <algorithm>
#include <cmath>

namespace handemu
{

namespace
{

const double kPi = 3.14159265358979323846;

const int kParent[ kBoneCount ] = {
	-1, 0,             // root, wrist
	1, 2, 3, 4,        // thumb 0..3 (3 = tip)
	1, 6, 7, 8, 9,     // index 0..4 (4 = tip)
	1, 11, 12, 13, 14, // middle
	1, 16, 17, 18, 19, // ring
	1, 21, 22, 23, 24, // pinky
	0, 0, 0, 0, 0,     // aux bones hang off the root
};

bool IsTip( int b ) { return b == 5 || b == 10 || b == 15 || b == 20 || b == 25; }

// Joints whose bend the finger / thumb gain scales (bone = the segment after the joint).
bool IsFingerCurlJoint( int b ) { return b >= 7 && b <= 24 && kParent[ b ] != 1 && !IsTip( b ); }
bool IsThumbCurlJoint( int b ) { return b == 3 || b == 4; }

// Valve places each aux bone on the last phalanx of its finger.
const int kAuxSource[ 5 ] = { 4, 9, 14, 19, 24 };

// SteamVR's reference hand (resources/skeletons/vr_glove_*_skeleton.glb), parent space.
// Position x, y, z; orientation w, x, y, z.
struct RefBone
{
	float p[ 3 ];
	float q[ 4 ];
};

const RefBone kReference[ 2 ][ kBoneCount ] = {
	{
		{ { 0.000000f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { 0.000160f, -0.000032f, -0.000626f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { -0.017914f, 0.029178f, 0.025298f }, { 0.276387f, 0.541194f, 0.182029f, 0.773036f } },
		{ { 0.040406f, 0.000000f, 0.000000f }, { 0.969173f, 0.000061f, -0.001372f, 0.246377f } },
		{ { 0.032517f, 0.000000f, 0.000000f }, { 0.988173f, 0.000098f, 0.001403f, 0.153335f } },
		{ { 0.030464f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { -0.001557f, 0.021073f, 0.014787f }, { 0.550753f, 0.531056f, -0.351434f, 0.539578f } },
		{ { 0.073798f, 0.000000f, 0.000000f }, { 0.968981f, 0.001623f, -0.052887f, 0.241404f } },
		{ { 0.043287f, 0.000000f, 0.000000f }, { 0.982770f, -0.000086f, 0.005041f, 0.184762f } },
		{ { 0.028275f, 0.000000f, 0.000000f }, { 0.997072f, 0.000031f, -0.001166f, 0.076458f } },
		{ { 0.022822f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { 0.002177f, 0.007120f, 0.016319f }, { 0.533423f, 0.561750f, -0.419737f, 0.472988f } },
		{ { 0.070886f, 0.000000f, 0.000000f }, { 0.973388f, 0.000000f, -0.000185f, 0.229163f } },
		{ { 0.043109f, 0.000000f, 0.000000f }, { 0.987529f, 0.000093f, -0.003688f, 0.157396f } },
		{ { 0.033266f, 0.000000f, 0.000000f }, { 0.989962f, -0.000107f, 0.004125f, 0.141277f } },
		{ { 0.025892f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { 0.000513f, -0.006545f, 0.016348f }, { 0.516692f, 0.550143f, -0.495548f, 0.429888f } },
		{ { 0.065975f, 0.000000f, 0.000000f }, { 0.974556f, -0.000901f, -0.040957f, 0.220369f } },
		{ { 0.040331f, 0.000000f, 0.000000f }, { 0.991001f, -0.000073f, 0.002531f, 0.133830f } },
		{ { 0.028489f, 0.000000f, 0.000000f }, { 0.990788f, 0.000199f, -0.004263f, 0.135355f } },
		{ { 0.022430f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { -0.002478f, -0.018981f, 0.015214f }, { -0.485758f, -0.515327f, 0.615016f, -0.346746f } },
		{ { 0.062856f, 0.000000f, 0.000000f }, { 0.993491f, 0.003942f, 0.028164f, 0.110308f } },
		{ { 0.029874f, 0.000000f, 0.000000f }, { 0.991114f, 0.000383f, -0.011456f, 0.132523f } },
		{ { 0.017979f, 0.000000f, 0.000000f }, { 0.994006f, -0.000538f, 0.012702f, 0.108581f } },
		{ { 0.018018f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { -0.039281f, 0.060081f, 0.084492f }, { -0.048614f, 0.569107f, -0.045037f, 0.819589f } },
		{ { -0.018233f, 0.037278f, 0.148956f }, { 0.209555f, 0.312326f, -0.597228f, 0.708419f } },
		{ { -0.012561f, 0.007871f, 0.154690f }, { 0.221141f, 0.271171f, -0.647061f, 0.677403f } },
		{ { -0.017869f, -0.023238f, 0.142235f }, { -0.237415f, -0.262352f, 0.721629f, -0.595027f } },
		{ { -0.016008f, -0.045652f, 0.119280f }, { -0.349000f, -0.265484f, 0.739030f, -0.511421f } },
	},
	{
		{ { 0.000000f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { -0.000160f, -0.000032f, -0.000626f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { 0.017914f, 0.029178f, 0.025298f }, { 0.541194f, -0.276387f, 0.773036f, -0.182029f } },
		{ { -0.040406f, 0.000000f, 0.000000f }, { 0.969173f, 0.000061f, -0.001372f, 0.246377f } },
		{ { -0.032517f, 0.000000f, 0.000000f }, { 0.988173f, 0.000098f, 0.001403f, 0.153335f } },
		{ { -0.030464f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { 0.001557f, 0.021073f, 0.014787f }, { 0.531056f, -0.550753f, 0.539578f, 0.351434f } },
		{ { -0.073798f, 0.000000f, 0.000000f }, { 0.968981f, 0.001623f, -0.052887f, 0.241404f } },
		{ { -0.043287f, 0.000000f, 0.000000f }, { 0.982770f, -0.000086f, 0.005041f, 0.184762f } },
		{ { -0.028275f, 0.000000f, 0.000000f }, { 0.997072f, 0.000031f, -0.001166f, 0.076458f } },
		{ { -0.022821f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { -0.002177f, 0.007120f, 0.016319f }, { 0.561750f, -0.533423f, 0.472988f, 0.419737f } },
		{ { -0.070886f, 0.000000f, 0.000000f }, { 0.973388f, 0.000000f, -0.000185f, 0.229163f } },
		{ { -0.043108f, 0.000000f, 0.000000f }, { 0.987529f, 0.000093f, -0.003688f, 0.157396f } },
		{ { -0.033266f, 0.000000f, 0.000000f }, { 0.989962f, -0.000107f, 0.004125f, 0.141277f } },
		{ { -0.025892f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { -0.000513f, -0.006545f, 0.016348f }, { 0.550143f, -0.516692f, 0.429888f, 0.495548f } },
		{ { -0.065975f, 0.000000f, 0.000000f }, { 0.974556f, -0.000901f, -0.040957f, 0.220369f } },
		{ { -0.040331f, 0.000000f, 0.000000f }, { 0.991001f, -0.000073f, 0.002531f, 0.133830f } },
		{ { -0.028489f, 0.000000f, 0.000000f }, { 0.990788f, 0.000199f, -0.004263f, 0.135355f } },
		{ { -0.022430f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { 0.002478f, -0.018981f, 0.015214f }, { 0.515327f, -0.485758f, 0.346746f, 0.615016f } },
		{ { -0.062856f, 0.000000f, 0.000000f }, { 0.993491f, 0.003942f, 0.028164f, 0.110308f } },
		{ { -0.029874f, 0.000000f, 0.000000f }, { 0.991114f, 0.000383f, -0.011456f, 0.132523f } },
		{ { -0.017979f, 0.000000f, 0.000000f }, { 0.994006f, -0.000538f, 0.012702f, 0.108581f } },
		{ { -0.018018f, 0.000000f, 0.000000f }, { 1.000000f, 0.000000f, 0.000000f, 0.000000f } },
		{ { 0.039281f, 0.060081f, 0.084492f }, { 0.569107f, 0.048614f, 0.819589f, 0.045037f } },
		{ { 0.018233f, 0.037278f, 0.148956f }, { 0.312326f, -0.209555f, 0.708419f, 0.597228f } },
		{ { 0.012561f, 0.007871f, 0.154690f }, { 0.271171f, -0.221141f, 0.677403f, 0.647061f } },
		{ { 0.017869f, -0.023238f, 0.142235f }, { 0.262352f, -0.237415f, 0.595027f, 0.721629f } },
		{ { 0.016008f, -0.045652f, 0.119280f }, { 0.265484f, -0.349000f, 0.511421f, 0.739030f } },
	},
};

void ModelSpace( const vr::VRBoneTransform_t *bones, Vec3 pos[ kBoneCount ], Quat rot[ kBoneCount ] )
{
	for ( uint32_t i = 0; i < kBoneCount; i++ )
	{
		const vr::VRBoneTransform_t &b = bones[ i ];
		Vec3 local{ b.position.v[ 0 ], b.position.v[ 1 ], b.position.v[ 2 ] };
		Quat localRot = NormalizedQ( FromVR( b.orientation ) );
		int parent = kParent[ i ];
		if ( parent < 0 )
		{
			pos[ i ] = local;
			rot[ i ] = localRot;
		}
		else
		{
			pos[ i ] = pos[ parent ] + Rotate( rot[ parent ], local );
			rot[ i ] = NormalizedQ( rot[ parent ] * localRot );
		}
	}
}

// A wrist frame built from joint positions only: z towards the middle knuckle, x across the
// knuckles (pinky -> index), y the palm normal. Only its relation to the reference wrist matters.
bool AnatomicalWrist( const Vec3 pos[ kBoneCount ], Quat &out )
{
	Vec3 along = pos[ 12 ] - pos[ 1 ];
	Vec3 across = pos[ 7 ] - pos[ 22 ];
	if ( Length( along ) < 0.02 || Length( across ) < 0.01 )
		return false;
	Vec3 z = Normalized( along );
	Vec3 y = Cross( z, across );
	if ( Length( y ) < 1e-4 )
		return false;
	y = Normalized( y );
	Vec3 x = Cross( y, z );
	out = QuatFromBasis( x, y, z );
	return true;
}

void SetBone( vr::VRBoneTransform_t &b, const Vec3 &p, const Quat &q )
{
	b.position.v[ 0 ] = (float)p.x;
	b.position.v[ 1 ] = (float)p.y;
	b.position.v[ 2 ] = (float)p.z;
	b.position.v[ 3 ] = 1.0f;
	b.orientation = ToVRf( NormalizedQ( q ) );
}

struct Reference
{
	vr::VRBoneTransform_t bones[ kBoneCount ];
	Quat localRot[ kBoneCount ];
	Vec3 childDir[ kBoneCount ];  // direction of the bone (towards its child joint) in the parent's frame
	Quat wristOffset;             // anatomical wrist frame -> reference wrist frame
	Quat auxRot[ 5 ];             // last phalanx frame -> aux frame
	Vec3 auxPos[ 5 ];             // aux position in the last phalanx frame
	double axisSign;              // bones run along +X on the left hand, -X on the right
};

Reference BuildReference( int side )
{
	Reference r{};
	for ( uint32_t i = 0; i < kBoneCount; i++ )
	{
		const RefBone &src = kReference[ side ][ i ];
		r.bones[ i ].position = { { src.p[ 0 ], src.p[ 1 ], src.p[ 2 ], 1.0f } };
		r.bones[ i ].orientation = { src.q[ 0 ], src.q[ 1 ], src.q[ 2 ], src.q[ 3 ] };
		r.localRot[ i ] = NormalizedQ( Quat{ src.q[ 0 ], src.q[ 1 ], src.q[ 2 ], src.q[ 3 ] } );
	}
	r.axisSign = side == 0 ? 1.0 : -1.0;
	for ( int b = 2; b <= 25; b++ )
	{
		if ( IsTip( b ) )
			continue;
		const RefBone &child = kReference[ side ][ b + 1 ];
		r.childDir[ b ] = Normalized( Rotate( r.localRot[ b ], Vec3{ child.p[ 0 ], child.p[ 1 ], child.p[ 2 ] } ) );
	}

	Vec3 pos[ kBoneCount ];
	Quat rot[ kBoneCount ];
	ModelSpace( r.bones, pos, rot );
	Quat anatomical;
	AnatomicalWrist( pos, anatomical );
	r.wristOffset = NormalizedQ( Conjugate( anatomical ) * rot[ 1 ] );
	for ( int a = 0; a < 5; a++ )
	{
		int src = kAuxSource[ a ];
		int aux = 26 + a;
		r.auxRot[ a ] = NormalizedQ( Conjugate( rot[ src ] ) * rot[ aux ] );
		r.auxPos[ a ] = Rotate( Conjugate( rot[ src ] ), pos[ aux ] - pos[ src ] );
	}
	return r;
}

const Reference &GetReference( int side )
{
	static const Reference refs[ 2 ] = { BuildReference( 0 ), BuildReference( 1 ) };
	return refs[ side ];
}

// Bends direction u (in the parent's frame) further around the parent's +Z axis.
Vec3 ScaleBend( const Vec3 &u, double axisSign, double gain, double maxBendRad )
{
	if ( gain == 1.0 )
		return u;
	double bend = std::atan2( axisSign * u.y, axisSign * u.x );
	if ( bend <= 0 )
		return u;
	double target = std::min( bend * gain, std::max( bend, maxBendRad ) );
	return Rotate( AxisAngle( Vec3{ 0, 0, 1 }, target - bend ), u );
}

double AngleBetween( const Vec3 &a, const Vec3 &b )
{
	double la = Length( a ), lb = Length( b );
	if ( la < 1e-9 || lb < 1e-9 )
		return 0;
	double c = Dot( a, b ) / ( la * lb );
	return std::acos( std::clamp( c, -1.0, 1.0 ) );
}

} // namespace

const vr::VRBoneTransform_t *ReferencePose( int side ) { return GetReference( side ).bones; }

void RetargetSkeleton( int side, const vr::VRBoneTransform_t *in, vr::VRBoneTransform_t *out, const SkeletonTuning &tuning )
{
	const Reference &ref = GetReference( side );
	Vec3 pos[ kBoneCount ];
	Quat rot[ kBoneCount ];
	ModelSpace( in, pos, rot );

	Quat anatomical;
	if ( !AnatomicalWrist( pos, anatomical ) )
	{
		// Not a usable hand (e.g. an all-zero placeholder): pass it through untouched.
		for ( uint32_t i = 0; i < kBoneCount; i++ )
			out[ i ] = in[ i ];
		return;
	}

	const double maxBend = std::clamp( (double)tuning.maxBend, 30.0, 180.0 ) * kPi / 180.0;
	// real[]: frames that follow the measured hand; frame[]: the frames actually sent (gains applied).
	Quat real[ kBoneCount ];
	Quat frame[ kBoneCount ];
	real[ 0 ] = frame[ 0 ] = rot[ 0 ];
	out[ 0 ] = in[ 0 ];
	real[ 1 ] = frame[ 1 ] = NormalizedQ( anatomical * ref.wristOffset );
	SetBone( out[ 1 ], Rotate( Conjugate( rot[ 0 ] ), pos[ 1 ] - pos[ 0 ] ), Conjugate( rot[ 0 ] ) * frame[ 1 ] );

	for ( int b = 2; b <= 25; b++ )
	{
		const int parent = kParent[ b ];
		Quat realLocal, sentLocal;
		if ( IsTip( b ) )
		{
			realLocal = sentLocal = Quat{};
		}
		else
		{
			Vec3 seg = pos[ b + 1 ] - pos[ b ];
			Vec3 u = Length( seg ) > 1e-5 ? Normalized( Rotate( Conjugate( real[ parent ] ), seg ) ) : ref.childDir[ b ];
			realLocal = NormalizedQ( FromTo( ref.childDir[ b ], u ) * ref.localRot[ b ] );
			double gain = IsFingerCurlJoint( b ) ? tuning.fingerGain : IsThumbCurlJoint( b ) ? tuning.thumbGain : 1.0;
			Vec3 sent = ScaleBend( u, ref.axisSign, gain, maxBend );
			sentLocal = NormalizedQ( FromTo( ref.childDir[ b ], sent ) * ref.localRot[ b ] );
		}
		real[ b ] = NormalizedQ( real[ parent ] * realLocal );
		frame[ b ] = NormalizedQ( frame[ parent ] * sentLocal );

		Vec3 local;
		if ( parent == 1 )
			local = Rotate( Conjugate( real[ 1 ] ), pos[ b ] - pos[ 1 ] ); // where the bone starts on the palm
		else
			local = Vec3{ ref.axisSign * Distance( pos[ b ], pos[ parent ] ), 0, 0 }; // phalanx length along the parent
		SetBone( out[ b ], local, sentLocal );
	}

	// Aux bones ride on the last phalanx, expressed relative to the root.
	for ( int a = 26; a <= 30; a++ )
		out[ a ] = in[ a ];
	Vec3 outPos[ kBoneCount ];
	Quat outRot[ kBoneCount ];
	ModelSpace( out, outPos, outRot );
	for ( int a = 0; a < 5; a++ )
	{
		int src = kAuxSource[ a ];
		Quat q = NormalizedQ( outRot[ src ] * ref.auxRot[ a ] );
		Vec3 p = outPos[ src ] + Rotate( outRot[ src ], ref.auxPos[ a ] );
		SetBone( out[ 26 + a ], Rotate( Conjugate( rot[ 0 ] ), p - pos[ 0 ] ), Conjugate( rot[ 0 ] ) * q );
	}
}

void MeasureBends( const vr::VRBoneTransform_t *bones, float out[ 15 ] )
{
	Vec3 pos[ kBoneCount ];
	Quat rot[ kBoneCount ];
	ModelSpace( bones, pos, rot );
	// Thumb base: angle between the thumb and index metacarpals; everything else: angle between
	// consecutive segments.
	out[ 0 ] = (float)( AngleBetween( pos[ 3 ] - pos[ 2 ], pos[ 7 ] - pos[ 6 ] ) * 180.0 / kPi );
	out[ 1 ] = (float)( AngleBetween( pos[ 3 ] - pos[ 2 ], pos[ 4 ] - pos[ 3 ] ) * 180.0 / kPi );
	out[ 2 ] = (float)( AngleBetween( pos[ 4 ] - pos[ 3 ], pos[ 5 ] - pos[ 4 ] ) * 180.0 / kPi );
	for ( int f = 0; f < 4; f++ )
	{
		int m = 6 + f * 5; // metacarpal
		for ( int j = 0; j < 3; j++ )
		{
			int b = m + j;
			out[ 3 + f * 3 + j ] = (float)( AngleBetween( pos[ b + 1 ] - pos[ b ], pos[ b + 2 ] - pos[ b + 1 ] ) * 180.0 / kPi );
		}
	}
}

} // namespace handemu
