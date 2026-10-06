// End-to-end test without a headset: loads driver_handemu.dll into a fake vrserver, plays the part of
// KRVR (an HMD plus two "krvr_hand" devices publishing skeletons through their own interface
// instances, as a separate driver would), and checks what the emulated controllers report.
//
// Build: see build_tests.bat. Run from anywhere: mock_vrserver.exe <path to driver_handemu.dll>

#include <windows.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <openvr_driver.h>

#include "../src/vrmath.h"

namespace
{

// ------------------------------------------------------------------------------------------------
// Shared fake server state
// ------------------------------------------------------------------------------------------------

struct PropValue
{
	vr::PropertyTypeTag_t tag = 0;
	std::vector<uint8_t> data;
};

struct Component
{
	vr::PropertyContainerHandle_t container;
	std::string name;
};

struct ServerState
{
	std::map<vr::PropertyContainerHandle_t, std::map<int, PropValue>> props;
	std::vector<vr::ITrackedDeviceServerDriver *> devices;
	std::map<vr::VRInputComponentHandle_t, Component> components;
	std::map<vr::VRInputComponentHandle_t, float> values;
	std::map<vr::VRInputComponentHandle_t, int> skeletonUpdates;
	std::map<std::pair<vr::VRInputComponentHandle_t, int>, std::vector<vr::VRBoneTransform_t>> lastSkeleton; // by (handle, motion range)
	std::map<uint32_t, vr::DriverPose_t> poses;
	std::map<std::string, int> calls;
	std::vector<std::string> logs;
	vr::VRInputComponentHandle_t nextHandle = 1000;

	static vr::PropertyContainerHandle_t ContainerOf( uint32_t index ) { return 0x10000 + index; }
};

ServerState g;

// ------------------------------------------------------------------------------------------------
// Fake vrserver interfaces. Function bodies differ on purpose so the linker cannot fold them.
// ------------------------------------------------------------------------------------------------

class FakeProperties : public vr::IVRProperties
{
public:
	vr::ETrackedPropertyError ReadPropertyBatch( vr::PropertyContainerHandle_t c, vr::PropertyRead_t *batch, uint32_t count ) override
	{
		g.calls[ "ReadPropertyBatch" ]++;
		for ( uint32_t i = 0; i < count; i++ )
		{
			vr::PropertyRead_t &r = batch[ i ];
			r.unTag = 0;
			r.unRequiredBufferSize = 0;
			auto ci = g.props.find( c );
			if ( ci == g.props.end() )
			{
				r.eError = vr::TrackedProp_InvalidContainer;
				continue;
			}
			auto pi = ci->second.find( r.prop );
			if ( pi == ci->second.end() )
			{
				r.eError = vr::TrackedProp_UnknownProperty;
				continue;
			}
			r.unTag = pi->second.tag;
			r.unRequiredBufferSize = (uint32_t)pi->second.data.size();
			if ( r.unBufferSize < pi->second.data.size() )
			{
				r.eError = vr::TrackedProp_BufferTooSmall;
				continue;
			}
			memcpy( r.pvBuffer, pi->second.data.data(), pi->second.data.size() );
			r.eError = vr::TrackedProp_Success;
		}
		return vr::TrackedProp_Success;
	}

	vr::ETrackedPropertyError WritePropertyBatch( vr::PropertyContainerHandle_t c, vr::PropertyWrite_t *batch, uint32_t count ) override
	{
		g.calls[ "WritePropertyBatch" ]++;
		for ( uint32_t i = 0; i < count; i++ )
		{
			vr::PropertyWrite_t &w = batch[ i ];
			if ( w.writeType == vr::PropertyWrite_Set )
			{
				PropValue v;
				v.tag = w.unTag;
				const uint8_t *p = static_cast<const uint8_t *>( w.pvBuffer );
				v.data.assign( p, p + w.unBufferSize );
				g.props[ c ][ w.prop ] = v;
			}
			else if ( w.writeType == vr::PropertyWrite_Erase )
			{
				g.props[ c ].erase( w.prop );
			}
			w.eError = vr::TrackedProp_Success;
		}
		return vr::TrackedProp_Success;
	}

	const char *GetPropErrorNameFromEnum( vr::ETrackedPropertyError ) override { return "property error"; }

	vr::PropertyContainerHandle_t TrackedDeviceToPropertyContainer( vr::TrackedDeviceIndex_t index ) override
	{
		g.calls[ "TrackedDeviceToPropertyContainer" ]++;
		return index < g.devices.size() ? ServerState::ContainerOf( index ) : vr::k_ulInvalidPropertyContainer;
	}
};

class FakeSettings : public vr::IVRSettings
{
public:
	const char *GetSettingsErrorNameFromEnum( vr::EVRSettingsError ) override { return "settings error"; }
	void SetBool( const char *, const char *, bool, vr::EVRSettingsError * ) override { g.calls[ "SetBool" ]++; }
	void SetInt32( const char *, const char *, int32_t, vr::EVRSettingsError * ) override { g.calls[ "SetInt32" ]++; }
	void SetFloat( const char *, const char *, float, vr::EVRSettingsError * ) override { g.calls[ "SetFloat" ]++; }
	void SetString( const char *, const char *, const char *, vr::EVRSettingsError * ) override { g.calls[ "SetString" ]++; }

	bool GetBool( const char *, const char *key, vr::EVRSettingsError *err ) override
	{
		if ( strcmp( key, "debugLog" ) == 0 )
		{
			if ( err )
				*err = vr::VRSettingsError_None;
			return true;
		}
		if ( err )
			*err = vr::VRSettingsError_UnsetSettingHasNoDefault;
		return false;
	}

	int32_t GetInt32( const char *, const char *, vr::EVRSettingsError *err ) override
	{
		if ( err )
			*err = vr::VRSettingsError_UnsetSettingHasNoDefault;
		return 0;
	}

	float GetFloat( const char *, const char *, vr::EVRSettingsError *err ) override
	{
		if ( err )
			*err = vr::VRSettingsError_UnsetSettingHasNoDefault;
		return 0.0f;
	}

	void GetString( const char *, const char *, char *value, uint32_t len, vr::EVRSettingsError *err ) override
	{
		if ( len )
			value[ 0 ] = 0;
		if ( err )
			*err = vr::VRSettingsError_UnsetSettingHasNoDefault;
	}

	void RemoveSection( const char *, vr::EVRSettingsError * ) override { g.calls[ "RemoveSection" ]++; }
	void RemoveKeyInSection( const char *, const char *, vr::EVRSettingsError * ) override { g.calls[ "RemoveKeyInSection" ]++; }
};

class FakeLog : public vr::IVRDriverLog
{
public:
	void Log( const char *message ) override
	{
		g.logs.push_back( message );
		printf( "    [driver] %s\n", message );
	}
};

class FakeDriverManager : public vr::IVRDriverManager
{
public:
	uint32_t GetDriverCount() const override { return 1; }
	uint32_t GetDriverName( vr::DriverId_t, char *value, uint32_t size ) override
	{
		if ( size )
			value[ 0 ] = 0;
		return 0;
	}
	vr::DriverHandle_t GetDriverHandle( const char * ) override { return 1; }
	bool IsEnabled( vr::DriverId_t ) const override { return true; }
};

class FakeResources : public vr::IVRResources
{
public:
	uint32_t LoadSharedResource( const char *, char *, uint32_t ) override { return 0; }
	uint32_t GetResourceFullPath( const char *, const char *, char *path, uint32_t len ) override
	{
		if ( len )
			path[ 0 ] = 0;
		return 0;
	}
};

class FakeHost : public vr::IVRServerDriverHost
{
public:
	bool TrackedDeviceAdded( const char *serial, vr::ETrackedDeviceClass deviceClass, vr::ITrackedDeviceServerDriver *driver ) override
	{
		uint32_t index = (uint32_t)g.devices.size();
		g.devices.push_back( driver );
		PropValue v;
		v.tag = vr::k_unInt32PropertyTag;
		int32_t cls = deviceClass;
		v.data.assign( reinterpret_cast<uint8_t *>( &cls ), reinterpret_cast<uint8_t *>( &cls ) + sizeof( cls ) );
		g.props[ ServerState::ContainerOf( index ) ][ vr::Prop_DeviceClass_Int32 ] = v;
		printf( "    [host] TrackedDeviceAdded(%s) -> device %u\n", serial, index );
		driver->Activate( index );
		return true;
	}

	void TrackedDevicePoseUpdated( uint32_t index, const vr::DriverPose_t &pose, uint32_t size ) override
	{
		g.calls[ "TrackedDevicePoseUpdated" ]++;
		vr::DriverPose_t copy{};
		memcpy( &copy, &pose, size < sizeof( copy ) ? size : sizeof( copy ) );
		g.poses[ index ] = copy;
	}

	void VsyncEvent( double ) override { g.calls[ "VsyncEvent" ]++; }
	void VendorSpecificEvent( uint32_t, vr::EVREventType, const vr::VREvent_Data_t &, double ) override { g.calls[ "VendorSpecificEvent" ]++; }
	bool IsExiting() override { return false; }
	bool PollNextEvent( vr::VREvent_t *, uint32_t ) override
	{
		g.calls[ "PollNextEvent" ]++;
		return false;
	}
	void GetRawTrackedDevicePoses( float, vr::TrackedDevicePose_t *, uint32_t ) override { g.calls[ "GetRawTrackedDevicePoses" ]++; }
	void RequestRestart( const char *, const char *, const char *, const char * ) override { g.calls[ "RequestRestart" ]++; }
	uint32_t GetFrameTimings( vr::Compositor_FrameTiming *, uint32_t ) override { return 0; }
	void SetDisplayEyeToHead( uint32_t, const vr::HmdMatrix34_t &, const vr::HmdMatrix34_t & ) override { g.calls[ "SetDisplayEyeToHead" ]++; }
	void SetDisplayProjectionRaw( uint32_t, const vr::HmdRect2_t &, const vr::HmdRect2_t & ) override { g.calls[ "SetDisplayProjectionRaw" ]++; }
	void SetRecommendedRenderTargetSize( uint32_t, uint32_t, uint32_t ) override { g.calls[ "SetRecommendedRenderTargetSize" ]++; }
};

class FakeInput : public vr::IVRDriverInput
{
	vr::EVRInputError Add( vr::PropertyContainerHandle_t c, const char *name, vr::VRInputComponentHandle_t *handle, const char *kind )
	{
		g.calls[ kind ]++;
		*handle = g.nextHandle++;
		g.components[ *handle ] = { c, name };
		return vr::VRInputError_None;
	}

public:
	vr::EVRInputError CreateBooleanComponent( vr::PropertyContainerHandle_t c, const char *name, vr::VRInputComponentHandle_t *h ) override
	{
		return Add( c, name, h, "CreateBoolean" );
	}
	vr::EVRInputError UpdateBooleanComponent( vr::VRInputComponentHandle_t h, bool value, double ) override
	{
		g.calls[ "UpdateBoolean" ]++;
		g.values[ h ] = value ? 1.0f : 0.0f;
		return vr::VRInputError_None;
	}
	vr::EVRInputError CreateScalarComponent( vr::PropertyContainerHandle_t c, const char *name, vr::VRInputComponentHandle_t *h, vr::EVRScalarType, vr::EVRScalarUnits ) override
	{
		return Add( c, name, h, "CreateScalar" );
	}
	vr::EVRInputError UpdateScalarComponent( vr::VRInputComponentHandle_t h, float value, double ) override
	{
		g.calls[ "UpdateScalar" ]++;
		g.values[ h ] = value;
		return vr::VRInputError_None;
	}
	vr::EVRInputError CreateHapticComponent( vr::PropertyContainerHandle_t c, const char *name, vr::VRInputComponentHandle_t *h ) override
	{
		return Add( c, name, h, "CreateHaptic" );
	}
	vr::EVRInputError CreateSkeletonComponent( vr::PropertyContainerHandle_t c, const char *name, const char *, const char *, vr::EVRSkeletalTrackingLevel,
		const vr::VRBoneTransform_t *, uint32_t, vr::VRInputComponentHandle_t *h ) override
	{
		return Add( c, name, h, "CreateSkeleton" );
	}
	vr::EVRInputError UpdateSkeletonComponent( vr::VRInputComponentHandle_t h, vr::EVRSkeletalMotionRange range, const vr::VRBoneTransform_t *bones, uint32_t count ) override
	{
		g.calls[ "UpdateSkeleton" ]++;
		if ( bones && count > 0 )
		{
			g.skeletonUpdates[ h ]++;
			g.lastSkeleton[ { h, (int)range } ].assign( bones, bones + count );
		}
		return vr::VRInputError_None;
	}
	vr::EVRInputError CreatePoseComponent( vr::PropertyContainerHandle_t c, const char *name, vr::VRInputComponentHandle_t *h ) override
	{
		return Add( c, name, h, "CreatePose" );
	}
	vr::EVRInputError UpdatePoseComponent( vr::VRInputComponentHandle_t, const vr::HmdMatrix34_t *, double ) override
	{
		g.calls[ "UpdatePose" ]++;
		return vr::VRInputError_None;
	}
	vr::EVRInputError CreateEyeTrackingComponent( vr::PropertyContainerHandle_t c, const char *name, vr::VRInputComponentHandle_t *h ) override
	{
		return Add( c, name, h, "CreateEyeTracking" );
	}
	vr::EVRInputError UpdateEyeTrackingComponent( vr::VRInputComponentHandle_t, const vr::VREyeTrackingData_t *, double ) override
	{
		g.calls[ "UpdateEyeTracking" ]++;
		return vr::VRInputError_None;
	}
};

class FakeContext : public vr::IVRDriverContext
{
public:
	void *GetGenericInterface( const char *version, vr::EVRInitError *err ) override
	{
		if ( err )
			*err = vr::VRInitError_None;
		if ( !strcmp( version, vr::IVRSettings_Version ) )
			return &settings;
		if ( !strcmp( version, vr::IVRProperties_Version ) )
			return &props;
		if ( !strcmp( version, vr::IVRServerDriverHost_Version ) )
			return &host;
		if ( !strcmp( version, vr::IVRDriverLog_Version ) )
			return &log;
		if ( !strcmp( version, vr::IVRDriverManager_Version ) )
			return &manager;
		if ( !strcmp( version, vr::IVRResources_Version ) )
			return &resources;
		if ( !strcmp( version, vr::IVRDriverInput_Version ) )
			return &input;
		if ( err )
			*err = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}
	vr::DriverHandle_t GetDriverHandle() override { return 1; }

	FakeSettings settings;
	FakeProperties props;
	FakeHost host;
	FakeLog log;
	FakeDriverManager manager;
	FakeResources resources;
	FakeInput input;
};

// "KRVR" talks to vrserver through its own interface objects, like any other driver.
FakeHost krvrHost;
FakeInput krvrInput;
FakeProperties krvrPropsRaw;
vr::CVRPropertyHelpers krvrProps( &krvrPropsRaw );

class FakeKrvrDevice : public vr::ITrackedDeviceServerDriver
{
public:
	enum Kind
	{
		kHmd,
		kLeft,
		kRight
	};
	explicit FakeKrvrDevice( Kind kind ) : m_kind( kind ) {}

	vr::EVRInitError Activate( uint32_t index ) override
	{
		this->index = index;
		vr::PropertyContainerHandle_t c = krvrPropsRaw.TrackedDeviceToPropertyContainer( index );
		if ( m_kind == kHmd )
		{
			krvrProps.SetStringProperty( c, vr::Prop_ControllerType_String, "krvr_hmd" );
			return vr::VRInitError_None;
		}
		const bool left = m_kind == kLeft;
		const char *profile = "{krvr_native}/input/krvr_hand_profile.json";
		krvrProps.SetStringProperty( c, vr::Prop_SerialNumber_String, left ? "KRVR-Native-HAND-L" : "KRVR-Native-HAND-R" );
		krvrProps.SetInt32Property( c, vr::Prop_ControllerRoleHint_Int32, left ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand );
		// As in the real vrserver the controller type is filled in from the profile only after activation.
		// The left hand has its profile path while creating components; the right hand has nothing yet.
		if ( left )
			krvrProps.SetStringProperty( c, vr::Prop_InputProfilePath_String, profile );
		krvrInput.CreateBooleanComponent( c, "/input/system/click", &system );
		krvrInput.CreateScalarComponent( c, "/input/trigger/value", &trigger, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided );
		krvrInput.CreateSkeletonComponent( c, left ? "/input/skeleton/left" : "/input/skeleton/right", left ? "/skeleton/hand/left" : "/skeleton/hand/right",
			"/pose/raw", vr::VRSkeletalTracking_Full, nullptr, 0, &skeleton );
		if ( !left )
			krvrProps.SetStringProperty( c, vr::Prop_InputProfilePath_String, profile );
		krvrProps.SetStringProperty( c, vr::Prop_ControllerType_String, "krvr_hand" );
		return vr::VRInitError_None;
	}
	void Deactivate() override {}
	void EnterStandby() override {}
	void *GetComponent( const char * ) override { return nullptr; }
	void DebugRequest( const char *, char *r, uint32_t n ) override
	{
		if ( n )
			r[ 0 ] = 0;
	}
	vr::DriverPose_t GetPose() override { return vr::DriverPose_t{}; }

	uint32_t index = 0;
	vr::VRInputComponentHandle_t system = 0, trigger = 0, skeleton = 0;

private:
	Kind m_kind;
};

// ------------------------------------------------------------------------------------------------
// Synthetic hands
// ------------------------------------------------------------------------------------------------

const int kParent[ 31 ] = { -1, 0, 1, 2, 3, 4, 1, 6, 7, 8, 9, 1, 11, 12, 13, 14, 1, 16, 17, 18, 19, 1, 21, 22, 23, 24, 0, 0, 0, 0, 0 };

enum HandShape
{
	kOpen,
	kPinchIndex,
	kPinchMiddle,
	kPinchRing,
	kPinchPinky,
	kFist
};

struct Bones
{
	vr::VRBoneTransform_t b[ 31 ];
};

// Right hand in skeleton-root space (palm down, fingers toward -Z); the left hand mirrors X.
Bones MakeHand( HandShape shape, bool left )
{
	Vec3 P[ 31 ];
	const Vec3 W{ 0.034, 0.036, 0.165 };
	const Vec3 mcp[ 4 ] = { { -0.020, 0, -0.090 }, { 0.0, 0, -0.092 }, { 0.020, 0, -0.088 }, { 0.038, 0, -0.080 } };
	const double seg[ 4 ][ 3 ] = { { 0.045, 0.025, 0.020 }, { 0.048, 0.027, 0.020 }, { 0.045, 0.025, 0.020 }, { 0.035, 0.020, 0.018 } };

	P[ 0 ] = { 0, 0, 0 };
	P[ 1 ] = W;
	P[ 2 ] = W + Vec3{ -0.020, -0.010, -0.020 };
	P[ 3 ] = W + Vec3{ -0.045, -0.015, -0.050 };
	P[ 4 ] = W + Vec3{ -0.060, -0.015, -0.075 };
	P[ 5 ] = W + Vec3{ -0.070, -0.015, -0.095 };

	for ( int f = 0; f < 4; f++ )
	{
		int base = 6 + 5 * f;
		P[ base ] = W;
		P[ base + 1 ] = W + mcp[ f ];
		bool pinching = ( shape == kPinchIndex && f == 0 ) || ( shape == kPinchMiddle && f == 1 ) || ( shape == kPinchRing && f == 2 ) || ( shape == kPinchPinky && f == 3 );
		if ( shape == kFist )
		{
			P[ base + 2 ] = P[ base + 1 ] + Vec3{ 0, -seg[ f ][ 0 ], 0 };
			P[ base + 3 ] = P[ base + 2 ] + Vec3{ 0, 0, seg[ f ][ 1 ] };
			P[ base + 4 ] = P[ base + 3 ] + Vec3{ 0, seg[ f ][ 2 ] * 0.8, seg[ f ][ 2 ] * 0.6 };
		}
		else if ( pinching )
		{
			Vec3 target = P[ base + 1 ] + Vec3{ -0.010, -0.050, -0.040 };
			P[ base + 2 ] = P[ base + 1 ] + Vec3{ 0, -0.020, -0.040 };
			P[ base + 3 ] = P[ base + 2 ] + Vec3{ -0.004, -0.018, -0.012 };
			P[ base + 4 ] = target;
			P[ 5 ] = target + Vec3{ 0, -0.006, 0 };
			P[ 4 ] = ( P[ 3 ] + P[ 5 ] ) * 0.5 + Vec3{ -0.010, 0, 0 };
		}
		else
		{
			P[ base + 2 ] = P[ base + 1 ] + Vec3{ 0, 0, -seg[ f ][ 0 ] };
			P[ base + 3 ] = P[ base + 2 ] + Vec3{ 0, 0, -seg[ f ][ 1 ] };
			P[ base + 4 ] = P[ base + 3 ] + Vec3{ 0, 0, -seg[ f ][ 2 ] };
		}
	}
	if ( shape == kFist )
	{
		P[ 5 ] = W + Vec3{ -0.010, -0.065, -0.070 };
		P[ 4 ] = ( P[ 3 ] + P[ 5 ] ) * 0.5;
	}
	for ( int i = 0; i < 5; i++ )
		P[ 26 + i ] = P[ i == 0 ? 5 : 5 + 5 * i ];

	Bones out{};
	for ( int i = 0; i < 31; i++ )
	{
		Vec3 p = P[ i ];
		if ( left )
			p.x = -p.x;
		Vec3 parent{};
		if ( kParent[ i ] >= 0 )
		{
			parent = P[ kParent[ i ] ];
			if ( left )
				parent.x = -parent.x;
		}
		Vec3 local = kParent[ i ] >= 0 ? p - parent : p;
		out.b[ i ].position = { { (float)local.x, (float)local.y, (float)local.z, 1.0f } };
		out.b[ i ].orientation = { 1, 0, 0, 0 };
	}
	return out;
}

vr::DriverPose_t MakePose( const Vec3 &pos, const Quat &rot = Quat{} )
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation.w = 1;
	p.qDriverFromHeadRotation.w = 1;
	p.qRotation = ToVRd( rot );
	p.vecPosition[ 0 ] = pos.x;
	p.vecPosition[ 1 ] = pos.y;
	p.vecPosition[ 2 ] = pos.z;
	p.result = vr::TrackingResult_Running_OK;
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	return p;
}

Vec3 WorldPosOf( const vr::DriverPose_t &p )
{
	Vec3 head = FromVR( p.vecDriverFromHeadTranslation );
	Vec3 driver = Rotate( FromVR( p.qRotation ), head ) + FromVR( p.vecPosition );
	return Rotate( FromVR( p.qWorldFromDriverRotation ), driver ) + FromVR( p.vecWorldFromDriverTranslation );
}

// ------------------------------------------------------------------------------------------------
// Test helpers
// ------------------------------------------------------------------------------------------------

int g_failures = 0;
int g_checks = 0;

void Check( bool ok, const char *what )
{
	g_checks++;
	if ( !ok )
		g_failures++;
	printf( "  [%s] %s\n", ok ? "PASS" : "FAIL", what );
}

bool LogContains( const char *needle )
{
	for ( const std::string &line : g.logs )
	{
		if ( line.find( needle ) != std::string::npos )
			return true;
	}
	return false;
}

float ValueOf( uint32_t device, const char *name )
{
	vr::PropertyContainerHandle_t c = ServerState::ContainerOf( device );
	for ( auto &kv : g.components )
	{
		if ( kv.second.container == c && kv.second.name == name )
		{
			auto v = g.values.find( kv.first );
			return v == g.values.end() ? -1.0f : v->second;
		}
	}
	return -2.0f;
}

vr::VRInputComponentHandle_t HandleOf( uint32_t device, const char *name )
{
	vr::PropertyContainerHandle_t c = ServerState::ContainerOf( device );
	for ( auto &kv : g.components )
	{
		if ( kv.second.container == c && kv.second.name == name )
			return kv.first;
	}
	return 0;
}

// Largest distance between corresponding joints (root .. pinky tip) of two parent-space skeletons.
double MaxJointError( const vr::VRBoneTransform_t *a, const vr::VRBoneTransform_t *b )
{
	Vec3 pa[ 31 ], pb[ 31 ];
	Quat ra[ 31 ], rb[ 31 ];
	const vr::VRBoneTransform_t *src[ 2 ] = { a, b };
	Vec3 *pos[ 2 ] = { pa, pb };
	Quat *rot[ 2 ] = { ra, rb };
	for ( int k = 0; k < 2; k++ )
	{
		for ( int i = 0; i < 31; i++ )
		{
			Vec3 local{ src[ k ][ i ].position.v[ 0 ], src[ k ][ i ].position.v[ 1 ], src[ k ][ i ].position.v[ 2 ] };
			Quat q = NormalizedQ( FromVR( src[ k ][ i ].orientation ) );
			int p = kParent[ i ];
			pos[ k ][ i ] = p < 0 ? local : pos[ k ][ p ] + Rotate( rot[ k ][ p ], local );
			rot[ k ][ i ] = p < 0 ? q : NormalizedQ( rot[ k ][ p ] * q );
		}
	}
	double worst = 0;
	for ( int i = 0; i <= 25; i++ )
		worst = std::fmax( worst, Distance( pa[ i ], pb[ i ] ) );
	return worst;
}

std::string DebugDirOf( const char *dllPath )
{
	std::string dir = dllPath;
	size_t slash = dir.find_last_of( "\\/" );
	dir = slash == std::string::npos ? std::string( "." ) : dir.substr( 0, slash );
	return dir + "\\debug";
}

void WriteText( const std::string &path, const char *text )
{
	FILE *f = nullptr;
	if ( fopen_s( &f, path.c_str(), "wb" ) == 0 && f )
	{
		fputs( text, f );
		fclose( f );
	}
}

std::string ReadText( const std::string &path )
{
	std::string out;
	FILE *f = nullptr;
	if ( fopen_s( &f, path.c_str(), "rb" ) != 0 || !f )
		return out;
	char buf[ 65536 ];
	size_t n;
	while ( ( n = fread( buf, 1, sizeof( buf ), f ) ) > 0 )
		out.append( buf, n );
	fclose( f );
	return out;
}

} // namespace

int main( int argc, char **argv )
{
	setvbuf( stdout, nullptr, _IONBF, 0 );
	const char *dllPath = argc > 1 ? argv[ 1 ] : "driver_handemu.dll";
	HMODULE dll = LoadLibraryA( dllPath );
	if ( !dll )
	{
		printf( "Cannot load %s (error %lu)\n", dllPath, GetLastError() );
		return 2;
	}
	using Factory = void *( * )( const char *, int * );
	Factory factory = reinterpret_cast<Factory>( GetProcAddress( dll, "HmdDriverFactory" ) );
	if ( !factory )
	{
		printf( "HmdDriverFactory not exported\n" );
		return 2;
	}
	int rc = 0;
	auto *provider = static_cast<vr::IServerTrackedDeviceProvider *>( factory( vr::IServerTrackedDeviceProvider_Version, &rc ) );
	Check( provider != nullptr, "factory returns IServerTrackedDeviceProvider" );
	if ( !provider )
		return 1;

	// Leftovers from an earlier run would be picked up as live tuning / a capture request.
	const std::string debugDir = DebugDirOf( dllPath );
	DeleteFileA( ( debugDir + "\\live.txt" ).c_str() );
	DeleteFileA( ( debugDir + "\\request.txt" ).c_str() );

	static FakeContext ctx;
	printf( "== Init\n" );
	Check( provider->Init( &ctx ) == vr::VRInitError_None, "Init succeeds" );
	Check( LogContains( "Hooks installed" ), "hooks installed into the fake vrserver" );

	printf( "== KRVR adds its headset and hands (after all drivers loaded, as in the real logs)\n" );
	FakeKrvrDevice hmd( FakeKrvrDevice::kHmd ), leftSrc( FakeKrvrDevice::kLeft ), rightSrc( FakeKrvrDevice::kRight );
	krvrHost.TrackedDeviceAdded( "KRVR-Native-001", vr::TrackedDeviceClass_HMD, &hmd );
	krvrHost.TrackedDeviceAdded( "KRVR-Native-HAND-L", vr::TrackedDeviceClass_Controller, &leftSrc );
	krvrHost.TrackedDeviceAdded( "KRVR-Native-HAND-R", vr::TrackedDeviceClass_Controller, &rightSrc );
	Check( LogContains( "Hooked left source skeleton" ) && LogContains( "Hooked left source system button" ), "left source components matched at creation by profile path" );
	Check( !LogContains( "Hooked right source" ), "right source components not identifiable yet (no properties at creation)" );

	const Vec3 leftPos{ -0.2, 1.2, -0.3 };
	const Vec3 rightPos{ 0.2, 1.2, -0.3 };
	krvrHost.TrackedDevicePoseUpdated( hmd.index, MakePose( { 0, 1.6, 0 } ), sizeof( vr::DriverPose_t ) );
	krvrHost.TrackedDevicePoseUpdated( leftSrc.index, MakePose( leftPos ), sizeof( vr::DriverPose_t ) );
	krvrHost.TrackedDevicePoseUpdated( rightSrc.index, MakePose( rightPos ), sizeof( vr::DriverPose_t ) );
	Check( LogContains( "Found left hand-tracking source" ) && LogContains( "Found right hand-tracking source" ), "source devices classified from their poses" );
	Check( LogContains( "Hooked right source skeleton" ) && LogContains( "Hooked right source system button" ), "right source components matched later from the creation record" );
	Check( !LogContains( "Adopted skeleton" ), "no fallback adoption needed" );

	printf( "== RunFrame creates the emulated controllers\n" );
	provider->RunFrame();
	Check( g.devices.size() == 5, "two emulated controllers added" );
	if ( g.devices.size() != 5 )
		return 1;
	const uint32_t L = 3, R = 4;
	Check( ValueOf( L, "/input/x/click" ) != -2.0f && ValueOf( R, "/input/a/click" ) != -2.0f, "left has X/Y, right has A/B components" );
	{
		vr::ETrackedPropertyError err;
		vr::CVRPropertyHelpers helpers( &krvrPropsRaw );
		std::string type = helpers.GetStringProperty( ServerState::ContainerOf( R ), vr::Prop_ControllerType_String, &err );
		int32_t prio = helpers.GetInt32Property( ServerState::ContainerOf( R ), vr::Prop_ControllerHandSelectionPriority_Int32, &err );
		Check( type == "handemu_hand" && prio == 100, "controller type handemu_hand with hand priority 100" );
	}

	auto runPoses = [ & ]( HandShape shape, int frames, const vr::DriverPose_t &leftPose, const vr::DriverPose_t &rightPose )
	{
		Bones rb = MakeHand( shape, false );
		Bones lb = MakeHand( shape, true );
		for ( int i = 0; i < frames; i++ )
		{
			krvrHost.TrackedDevicePoseUpdated( rightSrc.index, rightPose, sizeof( vr::DriverPose_t ) );
			krvrInput.UpdateSkeletonComponent( rightSrc.skeleton, vr::VRSkeletalMotionRange_WithController, rb.b, 31 );
			krvrInput.UpdateSkeletonComponent( rightSrc.skeleton, vr::VRSkeletalMotionRange_WithoutController, rb.b, 31 );
			krvrHost.TrackedDevicePoseUpdated( leftSrc.index, leftPose, sizeof( vr::DriverPose_t ) );
			krvrInput.UpdateSkeletonComponent( leftSrc.skeleton, vr::VRSkeletalMotionRange_WithController, lb.b, 31 );
			krvrInput.UpdateSkeletonComponent( leftSrc.skeleton, vr::VRSkeletalMotionRange_WithoutController, lb.b, 31 );
			provider->RunFrame();
		}
	};
	auto runShape = [ & ]( HandShape shape, int frames, Vec3 rightOffset = Vec3{} )
	{
		runPoses( shape, frames, MakePose( leftPos ), MakePose( rightPos + rightOffset ) );
	};

	printf( "== Open hand\n" );
	runShape( kOpen, 15 );
	Check( ValueOf( R, "/input/trigger/click" ) == 0 && ValueOf( R, "/input/grip/click" ) == 0 && ValueOf( R, "/input/a/click" ) == 0 &&
			   ValueOf( R, "/input/b/click" ) == 0 && ValueOf( R, "/input/joystick/touch" ) == 0,
		"nothing pressed" );
	Check( ValueOf( R, "/input/trigger/value" ) < 0.05f, "trigger value near zero" );

	printf( "== Index pinch\n" );
	runShape( kPinchIndex, 15 );
	Check( ValueOf( R, "/input/trigger/click" ) == 1 && ValueOf( R, "/input/trigger/value" ) == 1.0f && ValueOf( R, "/input/trigger/touch" ) == 1, "right trigger clicked" );
	Check( ValueOf( L, "/input/trigger/click" ) == 1, "left trigger clicked" );
	Check( ValueOf( R, "/input/a/click" ) == 0 && ValueOf( R, "/input/grip/click" ) == 0, "no other button" );

	printf( "== Middle pinch\n" );
	runShape( kOpen, 10 );
	runShape( kPinchMiddle, 15 );
	Check( ValueOf( R, "/input/a/click" ) == 1 && ValueOf( R, "/input/trigger/click" ) == 0, "right A clicked, trigger released" );
	Check( ValueOf( L, "/input/x/click" ) == 1, "left X clicked" );

	printf( "== Pinky pinch\n" );
	runShape( kOpen, 10 );
	runShape( kPinchPinky, 15 );
	Check( ValueOf( R, "/input/b/click" ) == 1 && ValueOf( R, "/input/a/click" ) == 0, "right B clicked" );
	Check( ValueOf( L, "/input/y/click" ) == 1, "left Y clicked" );

	printf( "== Ring pinch and drag 5 cm to the right\n" );
	runShape( kOpen, 10 );
	runShape( kPinchRing, 10 );
	Check( ValueOf( R, "/input/joystick/touch" ) == 1 && std::fabs( ValueOf( R, "/input/joystick/x" ) ) < 0.01f, "joystick engaged, centred" );
	runShape( kPinchRing, 5, Vec3{ 0.05, 0, 0 } );
	float jx = ValueOf( R, "/input/joystick/x" ), jy = ValueOf( R, "/input/joystick/y" );
	printf( "    joystick = (%.3f, %.3f)\n", jx, jy );
	Check( std::fabs( jx - 0.804f ) < 0.03f && std::fabs( jy ) < 0.01f, "joystick deflected right (~0.80)" );
	runShape( kPinchRing, 5, Vec3{ 0, 0, -0.10 } );
	Check( ValueOf( R, "/input/joystick/y" ) > 0.99f, "moving the hand forward pushes the stick forward" );
	runShape( kOpen, 10 );
	Check( ValueOf( R, "/input/joystick/touch" ) == 0 && ValueOf( R, "/input/joystick/x" ) == 0 && ValueOf( R, "/input/joystick/y" ) == 0, "joystick released and centred" );

	printf( "== Fist\n" );
	runShape( kFist, 15 );
	Check( ValueOf( R, "/input/grip/click" ) == 1 && ValueOf( R, "/input/grip/value" ) == 1.0f, "grip clicked" );
	Check( ValueOf( R, "/input/trigger/click" ) == 0 && ValueOf( R, "/input/trigger/touch" ) == 0, "fist does not pinch" );
	runShape( kOpen, 15 );
	Check( ValueOf( R, "/input/grip/click" ) == 0, "grip released" );

	printf( "== Pose and skeleton forwarding\n" );
	{
		const vr::DriverPose_t &p = g.poses[ R ];
		Check( std::fabs( p.vecPosition[ 0 ] - rightPos.x ) < 1e-9 && std::fabs( p.vecPosition[ 1 ] - rightPos.y ) < 1e-9 && p.poseIsValid && p.deviceIsConnected,
			"emulated pose mirrors the source pose" );
		Check( g.skeletonUpdates[ HandleOf( R, "/input/skeleton/right" ) ] > 50 && g.skeletonUpdates[ HandleOf( L, "/input/skeleton/left" ) ] > 50, "skeletons forwarded to both controllers" );
	}

	const vr::VRInputComponentHandle_t rightSkeleton = HandleOf( R, "/input/skeleton/right" );
	const vr::VRInputComponentHandle_t leftSkeleton = HandleOf( L, "/input/skeleton/left" );
	printf( "== Skeletons go out in SteamVR conventions\n" );
	{
		// The fake source uses identity bone rotations (its own convention), like a foreign driver might.
		runShape( kFist, 5 );
		const auto &rWith = g.lastSkeleton[ { rightSkeleton, vr::VRSkeletalMotionRange_WithController } ];
		const auto &rWithout = g.lastSkeleton[ { rightSkeleton, vr::VRSkeletalMotionRange_WithoutController } ];
		const auto &lWith = g.lastSkeleton[ { leftSkeleton, vr::VRSkeletalMotionRange_WithController } ];
		bool sizes = rWith.size() == 31 && rWithout.size() == 31 && lWith.size() == 31;
		Check( sizes && memcmp( rWith.data(), rWithout.data(), sizeof( vr::VRBoneTransform_t ) * 31 ) == 0, "both motion ranges get the unclamped hand" );
		if ( sizes )
		{
			bool rightAxes = true, leftAxes = true;
			for ( int b : { 3, 4, 5, 8, 9, 10, 13, 14, 15, 18, 19, 20, 23, 24, 25 } )
			{
				const float *rp = rWith[ b ].position.v;
				const float *lp = lWith[ b ].position.v;
				if ( !( rp[ 0 ] < -0.005f && std::fabs( rp[ 1 ] ) < 1e-4f && std::fabs( rp[ 2 ] ) < 1e-4f ) )
					rightAxes = false;
				if ( !( lp[ 0 ] > 0.005f && std::fabs( lp[ 1 ] ) < 1e-4f && std::fabs( lp[ 2 ] ) < 1e-4f ) )
					leftAxes = false;
			}
			Check( rightAxes && leftAxes, "phalanges run along -X (right) / +X (left) like SteamVR's reference hand" );
			double errR = MaxJointError( MakeHand( kFist, false ).b, rWith.data() );
			double errL = MaxJointError( MakeHand( kFist, true ).b, lWith.data() );
			printf( "    joint position error after rebuild: left %.3f mm, right %.3f mm\n", errL * 1000, errR * 1000 );
			Check( errR < 0.0005 && errL < 0.0005, "rebuilt hands keep the source joint positions" );
		}
	}

	printf( "== Capture and live tuning (debug folder)\n" );
	{
		const std::string request = debugDir + "\\request.txt";
		const std::string csv = debugDir + "\\mocktest.csv";
		const std::string live = debugDir + "\\live.txt";
		DeleteFileA( csv.c_str() );
		WriteText( request, "mocktest 0.3\n" );
		for ( int i = 0; i < 200 && !LogContains( "Capture 'mocktest' saved" ); i++ )
		{
			runShape( kOpen, 1 );
			Sleep( 10 );
		}
		Check( LogContains( "Capture 'mocktest' started" ) && LogContains( "Capture 'mocktest' saved" ), "request.txt starts and finishes a capture" );
		Check( GetFileAttributesA( request.c_str() ) == INVALID_FILE_ATTRIBUTES, "the request is consumed" );
		std::string text = ReadText( csv );
		Check( text.find( "\nS,0" ) != std::string::npos && text.find( ",1,1," ) != std::string::npos && text.find( "\nO," ) != std::string::npos &&
				   text.find( "\nP," ) != std::string::npos,
			"capture holds source skeletons (both ranges), sent skeletons and poses" );

		WriteText( live, "# tuning\nskeletonMode = 0\nfingerCurlGain = 1.4\nbogusKey = 1\n" );
		for ( int i = 0; i < 200 && !LogContains( "Live tuning: skeletonMode=0" ); i++ )
		{
			runShape( kFist, 1 );
			Sleep( 10 );
		}
		Check( LogContains( "Live tuning: skeletonMode=0" ) && LogContains( "fingerCurlGain=1.40" ) && LogContains( "ignored unknown keys: bogusKey" ),
			"live.txt is applied while running" );
		runShape( kFist, 2 );
		const auto &sent = g.lastSkeleton[ { rightSkeleton, vr::VRSkeletalMotionRange_WithController } ];
		Check( sent.size() == 31 && memcmp( sent.data(), MakeHand( kFist, false ).b, sizeof( vr::VRBoneTransform_t ) * 31 ) == 0,
			"skeletonMode 0 forwards the source skeleton unchanged" );

		DeleteFileA( live.c_str() );
		for ( int i = 0; i < 200 && !LogContains( "Live tuning: skeletonMode=1" ); i++ )
		{
			runShape( kOpen, 1 );
			Sleep( 10 );
		}
		Check( LogContains( "Live tuning: skeletonMode=1" ), "deleting live.txt restores the startup settings" );
	}

	printf( "== System button forwarding (KRVR palm tap)\n" );
	krvrInput.UpdateBooleanComponent( rightSrc.system, true, 0 );
	Check( ValueOf( R, "/input/system/click" ) == 1, "system press forwarded" );
	krvrInput.UpdateBooleanComponent( rightSrc.system, false, 0 );
	Check( ValueOf( R, "/input/system/click" ) == 0, "system release forwarded" );

	printf( "== Palm tap: right index fingertip taps the left palm\n" );
	{
		// Right index tip (open hand) sits at (0.014, 0.036, -0.015) from its root; the left palm centre
		// at (-0.034, 0.036, 0.119) from the left root. Put the right hand where the two coincide.
		const Vec3 touch = leftPos + Vec3{ -0.048, 0, 0.134 } - rightPos;
		runShape( kOpen, 10 );
		runShape( kOpen, 6, touch );
		Check( ValueOf( R, "/input/system/click" ) == 0, "no system press while touching" );
		runShape( kOpen, 3 );
		Check( ValueOf( R, "/input/system/click" ) == 1 && LogContains( "right index tapped the left palm" ), "system pressed when the fingertip leaves the palm" );
		Check( ValueOf( L, "/input/system/click" ) != 1, "left system untouched" );
		Sleep( 200 );
		runShape( kOpen, 3 );
		Check( ValueOf( R, "/input/system/click" ) == 0, "system released after the pulse" );
		for ( int i = 0; i < 24; i++ ) // ~1.2 s of continuous tracking with the finger resting on the palm
		{
			runShape( kOpen, 1, touch );
			Sleep( 50 );
		}
		runShape( kOpen, 3 );
		Check( ValueOf( R, "/input/system/click" ) == 0, "resting the finger on the palm for over a second does not fire" );
	}

	// Palms pressed together, fingers forward: the (palm-down) synthetic hands turned palm to palm,
	// their joint planes 3 cm apart.
	const double kPi = 3.14159265358979323846;
	const Quat leftPrayerRot = AxisAngle( { 0, 0, 1 }, kPi / 2 );
	const Quat rightPrayerRot = AxisAngle( { 0, 0, 1 }, -kPi / 2 );
	const Vec3 rightPrayer = leftPos + Vec3{ -0.042, 0, 0 };
	// The same pose as Vision Pro reported it on hardware: fingers touching (1.7 cm apart at the
	// device origin) but the wrists ~9 cm apart, which put the palm centres 6-9 cm apart.
	const double vee = 13.3 * kPi / 180;
	const Quat leftVeeRot = AxisAngle( { 0, 1, 0 }, -vee ) * leftPrayerRot;
	const Quat rightVeeRot = AxisAngle( { 0, 1, 0 }, vee ) * rightPrayerRot;
	const Vec3 rightVee = leftPos + Vec3{ 0.017 - 0.072 * std::cos( vee ), 0, 0 };
	// Hands apart for ~0.4 s (re-arms the gesture), then pressed together for ~1.8 s.
	auto holdPalmsTogether = [ & ]( const vr::DriverPose_t &leftPose, const vr::DriverPose_t &rightPose, const vr::DriverPose_t &leftApart,
								 const vr::DriverPose_t &rightApart )
	{
		for ( int i = 0; i < 8; i++ )
		{
			runPoses( kOpen, 1, leftApart, rightApart );
			Sleep( 50 );
		}
		for ( int i = 0; i < 40; i++ )
		{
			runPoses( kOpen, 1, leftPose, rightPose );
			Sleep( 45 );
		}
	};
	// World position of a joint of the open synthetic hand under a pose.
	auto jointAt = [ & ]( bool left, const vr::DriverPose_t &pose, int joint )
	{
		Bones b = MakeHand( kOpen, left );
		Vec3 pos[ 31 ];
		for ( int i = 0; i < 31; i++ )
		{
			Vec3 local{ b.b[ i ].position.v[ 0 ], b.b[ i ].position.v[ 1 ], b.b[ i ].position.v[ 2 ] };
			pos[ i ] = kParent[ i ] < 0 ? local : pos[ kParent[ i ] ] + local; // the synthetic hands use identity rotations
		}
		Vec3 p = joint < 0 ? ( pos[ 1 ] + pos[ 12 ] ) * 0.5 : pos[ joint ];
		return FromVR( pose.vecPosition ) + Rotate( FromVR( pose.qRotation ), p );
	};

	printf( "== Calibration gesture with no recentre recorded (palms together, wrists apart)\n" );
	{
		const vr::DriverPose_t lp = MakePose( leftPos, leftVeeRot ), rp = MakePose( rightVee, rightVeeRot );
		printf( "    wrists %.1f cm apart, palm centres %.1f cm, middle knuckles %.1f cm, middle finger joints %.1f / %.1f cm\n",
			Distance( jointAt( true, lp, 1 ), jointAt( false, rp, 1 ) ) * 100, Distance( jointAt( true, lp, -1 ), jointAt( false, rp, -1 ) ) * 100,
			Distance( jointAt( true, lp, 12 ), jointAt( false, rp, 12 ) ) * 100, Distance( jointAt( true, lp, 13 ), jointAt( false, rp, 13 ) ) * 100,
			Distance( jointAt( true, lp, 14 ), jointAt( false, rp, 14 ) ) * 100 );
		holdPalmsTogether( lp, rp, MakePose( leftPos ), MakePose( rightPos ) );
		Check( LogContains( "Palms together (joints" ), "palms together recognised with the wrists 9 cm apart" );
		Check( LogContains( "no Crown recentre recorded" ), "palms held together: nothing to undo, hands left alone" );
		Check( !LogContains( "Hand frame jump recorded" ), "one hand moving on its own is never taken for a recentre" );
	}
	runShape( kOpen, 10 );

	// A recentre as KRVR delivers it: the hand frame turns about the vertical through the head and shifts.
	auto recentre = []( double degrees, const Vec3 &shift )
	{
		const Vec3 pivot{ 0.0, 1.6, 0.0 };
		Rigid r;
		r.q = AxisAngle( { 0, 1, 0 }, degrees * 3.14159265358979 / 180.0 );
		r.t = pivot - Rotate( r.q, pivot ) + shift;
		return r;
	};
	auto in = []( const Rigid &frame, const Vec3 &p, const Quat &rot = Quat{} ) { return MakePose( Apply( frame, p ), NormalizedQ( frame.q * rot ) ); };
	auto countLogs = []( const char *needle )
	{
		size_t n = 0;
		for ( const std::string &line : g.logs )
			n += line.find( needle ) != std::string::npos;
		return n;
	};
	auto runRightOnly = [ & ]( int frames, const vr::DriverPose_t &rightPose )
	{
		Bones rb = MakeHand( kOpen, false );
		for ( int i = 0; i < frames; i++ )
		{
			krvrHost.TrackedDevicePoseUpdated( rightSrc.index, rightPose, sizeof( vr::DriverPose_t ) );
			krvrInput.UpdateSkeletonComponent( rightSrc.skeleton, vr::VRSkeletalMotionRange_WithController, rb.b, 31 );
			krvrInput.UpdateSkeletonComponent( rightSrc.skeleton, vr::VRSkeletalMotionRange_WithoutController, rb.b, 31 );
			provider->RunFrame();
		}
	};
	auto checkAligned = [ & ]( const char *what )
	{
		double errR = Distance( WorldPosOf( g.poses[ R ] ), rightPrayer );
		double errL = Distance( WorldPosOf( g.poses[ L ] ), leftPos );
		printf( "    alignment error after calibration: left %.1f mm, right %.1f mm\n", errL * 1000, errR * 1000 );
		Check( errR < 0.002 && errL < 0.002, what );
	};
	const size_t tapsBefore = countLogs( "palm -> system button" );

	printf( "== Crown pressed with the left hand out of view: the right hand alone records the jump\n" );
	const Rigid first = recentre( 20.0, { -0.10, 0, 0.15 } );
	{
		runShape( kOpen, 10 );
		for ( int i = 0; i < 6; i++ ) // left hand leaves the view (no skeleton) for ~250 ms
		{
			runRightOnly( 1, MakePose( rightPos ) );
			Sleep( 40 );
		}
		runRightOnly( 1, MakePose( rightPos ) ); // a fresh sample right before the jump (Sleep granularity)
		runRightOnly( 4, in( first, rightPos ) );
		Check( countLogs( "Hand frame jump recorded (right hand, left hand out of view)" ) == 1, "jump recorded from the right hand alone" );
		runPoses( kOpen, 10, in( first, leftPos ), in( first, rightPos ) ); // the left hand comes back, already shifted
		Check( countLogs( "Hand frame jump recorded" ) == 1, "the left hand reappearing elsewhere is not a second jump" );
		holdPalmsTogether( in( first, leftPos, leftPrayerRot ), in( first, rightPrayer, rightPrayerRot ), in( first, leftPos ), in( first, rightPos ) );
		Check( LogContains( "Calibration gesture: undid 1" ), "palms held together undo it" );
		checkAligned( "controllers are back where the hands really are" );
	}

	printf( "== Second recentre with both hands in view, on top of the first\n" );
	{
		const Rigid both = Compose( recentre( 36.8, { 0.05, 0, -0.30 } ), first );
		runPoses( kOpen, 10, in( first, leftPos ), in( first, rightPos ) );
		runPoses( kOpen, 10, in( both, leftPos ), in( both, rightPos ) );
		Check( LogContains( "Hand frame jump recorded (both hands)" ), "the jump is recorded" );
		Vec3 expected = Apply( Inverse( first ), Apply( both, rightPos ) ); // only the first correction applies so far
		Check( Distance( WorldPosOf( g.poses[ R ] ), expected ) < 1e-6, "nothing changes on its own: the controller follows the jumped hand" );
		holdPalmsTogether( in( both, leftPos, leftPrayerRot ), in( both, rightPrayer, rightPrayerRot ), in( both, leftPos ), in( both, rightPos ) );
		Check( LogContains( "Calibration gesture: undid 2" ), "palms held together undo both jumps" );
		checkAligned( "controllers are back where the hands really are" );
	}
	Check( countLogs( "palm -> system button" ) == tapsBefore, "no palm tap fired while bringing the palms together" );

	printf( "== Tracking lost while pinching\n" );
	runShape( kPinchIndex, 15 );
	Check( ValueOf( R, "/input/trigger/click" ) == 1, "pinching" );
	Sleep( 300 );
	provider->RunFrame();
	Check( ValueOf( R, "/input/trigger/click" ) == 0 && ValueOf( L, "/input/trigger/click" ) == 0, "buttons released after 250 ms without data" );

	printf( "== Cleanup\n" );
	provider->Cleanup();
	krvrInput.UpdateSkeletonComponent( rightSrc.skeleton, vr::VRSkeletalMotionRange_WithoutController, MakeHand( kOpen, false ).b, 31 );
	Check( true, "source keeps publishing after cleanup without crashing" );

	printf( "\n%d/%d checks passed\n", g_checks - g_failures, g_checks );
	return g_failures == 0 ? 0 : 1;
}
