// Minimal vector / quaternion helpers used by the HandEmu driver.
#pragma once

#include <cmath>

#include <openvr_driver.h>

struct Vec3
{
	double x = 0, y = 0, z = 0;
};

inline Vec3 V3( double x, double y, double z ) { return Vec3{ x, y, z }; }
inline Vec3 operator+( const Vec3 &a, const Vec3 &b ) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline Vec3 operator-( const Vec3 &a, const Vec3 &b ) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline Vec3 operator*( const Vec3 &a, double s ) { return { a.x * s, a.y * s, a.z * s }; }
inline double Dot( const Vec3 &a, const Vec3 &b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double Length( const Vec3 &a ) { return std::sqrt( Dot( a, a ) ); }
inline double Distance( const Vec3 &a, const Vec3 &b ) { return Length( a - b ); }

inline Vec3 Normalized( const Vec3 &a, const Vec3 &fallback = Vec3{ 0, 0, -1 } )
{
	double len = Length( a );
	return len > 1e-9 ? a * ( 1.0 / len ) : fallback;
}

inline Vec3 Cross( const Vec3 &a, const Vec3 &b ) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }

struct Quat
{
	double w = 1, x = 0, y = 0, z = 0;
};

inline Quat operator*( const Quat &a, const Quat &b )
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

inline Quat Conjugate( const Quat &q ) { return { q.w, -q.x, -q.y, -q.z }; }

inline Quat NormalizedQ( const Quat &q )
{
	double n = std::sqrt( q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z );
	if ( n < 1e-12 )
		return Quat{};
	return { q.w / n, q.x / n, q.y / n, q.z / n };
}

inline Vec3 Rotate( const Quat &q, const Vec3 &v )
{
	// v' = q * v * q^-1, expanded.
	Vec3 u{ q.x, q.y, q.z };
	double s = q.w;
	Vec3 cross1{ u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x };
	Vec3 t = cross1 * 2.0;
	Vec3 cross2{ u.y * t.z - u.z * t.y, u.z * t.x - u.x * t.z, u.x * t.y - u.y * t.x };
	return v + t * s + cross2;
}

inline Quat AxisAngle( const Vec3 &axis, double radians )
{
	Vec3 a = Normalized( axis );
	double h = radians * 0.5;
	double s = std::sin( h );
	return { std::cos( h ), a.x * s, a.y * s, a.z * s };
}

// Smallest rotation taking unit vector a onto unit vector b.
inline Quat FromTo( const Vec3 &a, const Vec3 &b )
{
	double d = Dot( a, b );
	if ( d < -0.999999 )
	{
		Vec3 axis = Cross( Vec3{ 1, 0, 0 }, a );
		if ( Length( axis ) < 1e-6 )
			axis = Cross( Vec3{ 0, 1, 0 }, a );
		return AxisAngle( axis, 3.14159265358979323846 );
	}
	Vec3 c = Cross( a, b );
	return NormalizedQ( Quat{ 1 + d, c.x, c.y, c.z } );
}

// Rotation whose matrix has the orthonormal columns x, y, z.
inline Quat QuatFromBasis( const Vec3 &x, const Vec3 &y, const Vec3 &z )
{
	const double m00 = x.x, m01 = y.x, m02 = z.x;
	const double m10 = x.y, m11 = y.y, m12 = z.y;
	const double m20 = x.z, m21 = y.z, m22 = z.z;
	const double trace = m00 + m11 + m22;
	Quat q;
	if ( trace > 0 )
	{
		double s = std::sqrt( trace + 1.0 ) * 2;
		q = { 0.25 * s, ( m21 - m12 ) / s, ( m02 - m20 ) / s, ( m10 - m01 ) / s };
	}
	else if ( m00 > m11 && m00 > m22 )
	{
		double s = std::sqrt( 1.0 + m00 - m11 - m22 ) * 2;
		q = { ( m21 - m12 ) / s, 0.25 * s, ( m01 + m10 ) / s, ( m02 + m20 ) / s };
	}
	else if ( m11 > m22 )
	{
		double s = std::sqrt( 1.0 + m11 - m00 - m22 ) * 2;
		q = { ( m02 - m20 ) / s, ( m01 + m10 ) / s, 0.25 * s, ( m12 + m21 ) / s };
	}
	else
	{
		double s = std::sqrt( 1.0 + m22 - m00 - m11 ) * 2;
		q = { ( m10 - m01 ) / s, ( m02 + m20 ) / s, ( m12 + m21 ) / s, 0.25 * s };
	}
	return NormalizedQ( q );
}

// Pitch (X), yaw (Y), roll (Z) in degrees, applied as yaw * pitch * roll.
inline Quat FromEulerDegrees( double pitch, double yaw, double roll )
{
	const double k = 3.14159265358979323846 / 180.0;
	Quat qx = AxisAngle( { 1, 0, 0 }, pitch * k );
	Quat qy = AxisAngle( { 0, 1, 0 }, yaw * k );
	Quat qz = AxisAngle( { 0, 0, 1 }, roll * k );
	return qy * qx * qz;
}

inline Quat FromVR( const vr::HmdQuaternion_t &q ) { return { q.w, q.x, q.y, q.z }; }
inline Quat FromVR( const vr::HmdQuaternionf_t &q ) { return { q.w, q.x, q.y, q.z }; }
inline vr::HmdQuaternion_t ToVRd( const Quat &q ) { return { q.w, q.x, q.y, q.z }; }
inline vr::HmdQuaternionf_t ToVRf( const Quat &q ) { return { (float)q.w, (float)q.x, (float)q.y, (float)q.z }; }
inline Vec3 FromVR( const double v[ 3 ] ) { return { v[ 0 ], v[ 1 ], v[ 2 ] }; }

inline double Clamp01( double v ) { return v < 0 ? 0 : ( v > 1 ? 1 : v ); }

inline double Dot( const Quat &a, const Quat &b ) { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }

// Rotation angle of a unit quaternion, radians (0..pi).
inline double AngleOf( const Quat &q )
{
	double w = std::fabs( q.w );
	return 2.0 * std::acos( w > 1.0 ? 1.0 : w );
}

// Twist about the vertical (+Y) axis, radians.
inline double YawOf( const Quat &q ) { return 2.0 * std::atan2( q.y, q.w ); }

// Whatever is left after removing the yaw twist (tilt), radians.
inline double SwingAngle( const Quat &q )
{
	Quat twist = NormalizedQ( Quat{ q.w, 0, q.y, 0 } );
	return AngleOf( q * Conjugate( twist ) );
}

// Rigid transform: p' = q * p + t.
struct Rigid
{
	Quat q;
	Vec3 t;
};

inline Vec3 Apply( const Rigid &a, const Vec3 &p ) { return Rotate( a.q, p ) + a.t; }
inline Rigid Compose( const Rigid &a, const Rigid &b ) { return { NormalizedQ( a.q * b.q ), Rotate( a.q, b.t ) + a.t }; }

inline Rigid Inverse( const Rigid &a )
{
	Quat qi = Conjugate( a.q );
	return { qi, Rotate( qi, a.t ) * -1.0 };
}

inline bool IsIdentity( const Rigid &a ) { return AngleOf( a.q ) < 1e-6 && Length( a.t ) < 1e-6; }
