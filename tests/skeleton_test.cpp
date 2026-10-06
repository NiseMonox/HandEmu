// Unit checks for the skeleton rebuild (src/skeleton.cpp), independent of SteamVR.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

#include "../src/skeleton.h"

using namespace handemu;

namespace
{

int g_failures = 0;
int g_checks = 0;

void Check( bool ok, const char *what )
{
	g_checks++;
	if ( !ok )
	{
		g_failures++;
		printf( "FAIL: %s\n", what );
	}
}

const int kParent[ kBoneCount ] = { -1, 0, 1, 2, 3, 4, 1, 6, 7, 8, 9, 1, 11, 12, 13, 14, 1, 16, 17, 18, 19, 1, 21, 22, 23, 24, 0, 0, 0, 0, 0 };

void ModelSpace( const vr::VRBoneTransform_t *bones, Vec3 *pos, Quat *rot )
{
	for ( uint32_t i = 0; i < kBoneCount; i++ )
	{
		Vec3 local{ bones[ i ].position.v[ 0 ], bones[ i ].position.v[ 1 ], bones[ i ].position.v[ 2 ] };
		Quat q = NormalizedQ( FromVR( bones[ i ].orientation ) );
		int p = kParent[ i ];
		pos[ i ] = p < 0 ? local : pos[ p ] + Rotate( rot[ p ], local );
		rot[ i ] = p < 0 ? q : NormalizedQ( rot[ p ] * q );
	}
}

void FromModelSpace( const Vec3 *pos, const Quat *rot, vr::VRBoneTransform_t *out )
{
	for ( uint32_t i = 0; i < kBoneCount; i++ )
	{
		int p = kParent[ i ];
		Vec3 t = p < 0 ? pos[ i ] : Rotate( Conjugate( rot[ p ] ), pos[ i ] - pos[ p ] );
		Quat q = p < 0 ? rot[ i ] : Conjugate( rot[ p ] ) * rot[ i ];
		out[ i ].position = { { (float)t.x, (float)t.y, (float)t.z, 1.0f } };
		out[ i ].orientation = ToVRf( NormalizedQ( q ) );
	}
}

double QuatDistanceDeg( const Quat &a, const Quat &b ) { return AngleOf( NormalizedQ( Conjugate( a ) * b ) ) * 180.0 / 3.14159265358979323846; }

// Largest joint position error (m) and finger bone (2..25) local rotation error (deg).
void Compare( const vr::VRBoneTransform_t *a, const vr::VRBoneTransform_t *b, double &posErr, double &rotErr, int firstRotBone = 2 )
{
	Vec3 pa[ kBoneCount ], pb[ kBoneCount ];
	Quat ra[ kBoneCount ], rb[ kBoneCount ];
	ModelSpace( a, pa, ra );
	ModelSpace( b, pb, rb );
	posErr = rotErr = 0;
	int worstPos = -1, worstRot = -1;
	for ( uint32_t i = 0; i <= 25; i++ ) // aux bones are derived from the last phalanx; Curl() leaves them behind
	{
		double e = Distance( pa[ i ], pb[ i ] );
		if ( e > posErr )
		{
			posErr = e;
			worstPos = (int)i;
		}
	}
	for ( int i = firstRotBone; i <= 25; i++ )
	{
		double e = QuatDistanceDeg( FromVR( a[ i ].orientation ), FromVR( b[ i ].orientation ) );
		if ( e > rotErr )
		{
			rotErr = e;
			worstRot = i;
		}
	}
	if ( posErr > 0.0005 || rotErr > 0.5 )
	{
		printf( "   worst position bone %d, worst rotation bone %d\n", worstPos, worstRot );
		if ( worstRot >= 0 )
		{
			const vr::HmdQuaternionf_t &qa = a[ worstRot ].orientation, &qb = b[ worstRot ].orientation;
			printf( "   expected (%.3f %.3f %.3f %.3f) got (%.3f %.3f %.3f %.3f)\n", qa.w, qa.x, qa.y, qa.z, qb.w, qb.x, qb.y, qb.z );
		}
	}
}

// The reference hand, placed somewhere in the device frame like a live source would (the aux
// bones, which hang off the root, move with it).
void PlacedReference( int side, vr::VRBoneTransform_t *out )
{
	const vr::VRBoneTransform_t *ref = ReferencePose( side );
	Vec3 pos[ kBoneCount ];
	Quat rot[ kBoneCount ];
	ModelSpace( ref, pos, rot );
	Quat wrist = FromEulerDegrees( 20, side == 0 ? 70 : -70, side == 0 ? 15 : -15 );
	Vec3 wristPos{ side == 0 ? -0.03 : 0.03, 0.04, 0.16 };
	Quat move = NormalizedQ( wrist * Conjugate( rot[ 1 ] ) );
	const Vec3 oldWristPos = pos[ 1 ];
	for ( uint32_t i = 1; i < kBoneCount; i++ )
	{
		pos[ i ] = wristPos + Rotate( move, pos[ i ] - oldWristPos );
		rot[ i ] = NormalizedQ( move * rot[ i ] );
	}
	FromModelSpace( pos, rot, out );
}

// Bends every finger joint by `deg` about its own +Z (SteamVR's curl axis).
void Curl( vr::VRBoneTransform_t *bones, double deg, double thumbDeg )
{
	for ( int b = 2; b <= 25; b++ )
	{
		bool tip = b == 5 || b == 10 || b == 15 || b == 20 || b == 25;
		if ( tip || kParent[ b ] == 1 )
			continue;
		double d = b <= 4 ? thumbDeg : deg;
		Quat q = FromVR( bones[ b ].orientation ) * AxisAngle( { 0, 0, 1 }, d * 3.14159265358979323846 / 180.0 );
		bones[ b ].orientation = ToVRf( NormalizedQ( q ) );
	}
}

// Same hand, every bone frame re-oriented by an arbitrary constant rotation (another driver's
// conventions): joint positions are unchanged, parent-space rotations are not.
void Reframe( const vr::VRBoneTransform_t *in, vr::VRBoneTransform_t *out, unsigned seed )
{
	Vec3 pos[ kBoneCount ];
	Quat rot[ kBoneCount ];
	ModelSpace( in, pos, rot );
	std::mt19937 rng( seed );
	std::uniform_real_distribution<double> u( -1.0, 1.0 );
	for ( uint32_t i = 1; i < kBoneCount; i++ )
	{
		Quat c = NormalizedQ( Quat{ u( rng ), u( rng ), u( rng ), u( rng ) } );
		rot[ i ] = NormalizedQ( rot[ i ] * c );
	}
	FromModelSpace( pos, rot, out );
}

} // namespace

Quat Anat( const vr::VRBoneTransform_t *bones )
{
	Vec3 pos[ kBoneCount ];
	Quat rot[ kBoneCount ];
	ModelSpace( bones, pos, rot );
	Vec3 z = Normalized( pos[ 12 ] - pos[ 1 ] );
	Vec3 y = Normalized( Cross( z, pos[ 7 ] - pos[ 22 ] ) );
	Vec3 x = Cross( y, z );
	return QuatFromBasis( x, y, z );
}

int main()
{
	if ( getenv( "SKEL_DEBUG" ) )
	{
		vr::VRBoneTransform_t placed[ kBoneCount ];
		PlacedReference( 0, placed );
		Quat a = Anat( ReferencePose( 0 ) ), b = Anat( placed );
		Quat w = FromVR( placed[ 1 ].orientation );
		Quat moved = NormalizedQ( w * a );
		printf( "Aref (%.3f %.3f %.3f %.3f)\nAnew (%.3f %.3f %.3f %.3f)\nw*Aref (%.3f %.3f %.3f %.3f)\n", a.w, a.x, a.y, a.z, b.w, b.x, b.y, b.z, moved.w,
			moved.x, moved.y, moved.z );
	}
	SkeletonTuning plain;
	plain.fingerGain = plain.thumbGain = 1.0f;
	char what[ 200 ];

	for ( int side = 0; side < 2; side++ )
	{
		const char *name = side == 0 ? "left" : "right";
		vr::VRBoneTransform_t ref[ kBoneCount ], out[ kBoneCount ];
		double pe, re;

		// 1. The reference hand comes back unchanged.
		PlacedReference( side, ref );
		RetargetSkeleton( side, ref, out, plain );
		Compare( ref, out, pe, re, 1 );
		printf( "%s reference: position error %.2f mm, rotation error %.2f deg\n", name, pe * 1000, re );
		snprintf( what, sizeof( what ), "%s reference hand is reproduced", name );
		Check( pe < 0.0005 && re < 0.5, what );

		// 2. A curled hand in SteamVR conventions comes back unchanged.
		vr::VRBoneTransform_t curled[ kBoneCount ];
		memcpy( curled, ref, sizeof( curled ) );
		Curl( curled, 50, 30 );
		RetargetSkeleton( side, curled, out, plain );
		Compare( curled, out, pe, re, 1 );
		printf( "%s curled: position error %.2f mm, rotation error %.2f deg\n", name, pe * 1000, re );
		snprintf( what, sizeof( what ), "%s curled hand is reproduced", name );
		Check( pe < 0.0005 && re < 0.5, what );

		// 3. The same curled hand in arbitrary bone conventions gives the same result.
		vr::VRBoneTransform_t foreign[ kBoneCount ], out2[ kBoneCount ];
		for ( unsigned seed = 1; seed <= 5; seed++ )
		{
			Reframe( curled, foreign, seed * 7 + side );
			RetargetSkeleton( side, foreign, out2, plain );
			Compare( out, out2, pe, re, 1 );
			snprintf( what, sizeof( what ), "%s foreign conventions (seed %u): %.2f mm, %.2f deg", name, seed, pe * 1000, re );
			Check( pe < 0.0005 && re < 0.5, what );
		}
		printf( "%s foreign conventions: last error %.2f mm, %.2f deg\n", name, pe * 1000, re );

		// 4. Finger gain: 50 deg curls (on top of the reference bend) are scaled, thumb left alone.
		float before[ 15 ], after[ 15 ];
		MeasureBends( out, before );
		SkeletonTuning gain = plain;
		gain.fingerGain = 1.3f;
		RetargetSkeleton( side, curled, out2, gain );
		MeasureBends( out2, after );
		bool scaled = true;
		for ( int j = 3; j < 15; j++ )
		{
			double expect = std::fmin( before[ j ] * 1.3, std::fmax( before[ j ], 115.0 ) );
			if ( std::fabs( after[ j ] - expect ) > 1.5 )
				scaled = false;
		}
		printf( "%s gain 1.3: index %.0f/%.0f/%.0f -> %.0f/%.0f/%.0f, thumb %.0f/%.0f -> %.0f/%.0f\n", name, before[ 3 ], before[ 4 ], before[ 5 ], after[ 3 ],
			after[ 4 ], after[ 5 ], before[ 1 ], before[ 2 ], after[ 1 ], after[ 2 ] );
		snprintf( what, sizeof( what ), "%s finger gain scales finger bends", name );
		Check( scaled, what );
		snprintf( what, sizeof( what ), "%s finger gain leaves the thumb alone", name );
		Check( std::fabs( after[ 1 ] - before[ 1 ] ) < 0.5 && std::fabs( after[ 2 ] - before[ 2 ] ) < 0.5, what );

		// 5. A placeholder (all identity) skeleton is passed through instead of producing NaNs.
		vr::VRBoneTransform_t zero[ kBoneCount ] = {};
		for ( auto &b : zero )
			b.orientation.w = 1;
		RetargetSkeleton( side, zero, out, plain );
		snprintf( what, sizeof( what ), "%s placeholder skeleton passes through", name );
		Check( memcmp( zero, out, sizeof( zero ) ) == 0, what );
	}

	printf( "%d/%d checks passed\n", g_checks - g_failures, g_checks );
	return g_failures == 0 ? 0 : 1;
}
