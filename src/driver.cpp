// HandEmu: turns hand-tracking skeletons published by another SteamVR driver (KRVR's "krvr_hand"
// devices) into emulated Touch-style controllers, so games that only understand controllers
// can be played with bare hands.
//
// SteamVR has no API for one driver to read another driver's input, so this driver hooks
// IVRServerDriverHost::TrackedDevicePoseUpdated and a few IVRDriverInput calls inside vrserver
// (the technique OpenVR-SpaceCalibrator uses for poses) and mirrors what the source driver sends.

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <MinHook.h>
#include <openvr_driver.h>

#include "capture.h"
#include "gestures.h"
#include "skeleton.h"
#include "vrmath.h"

using namespace handemu;

namespace
{

const char *const kVersion = "1.0.4";
const char *const kSettingsSection = "driver_handemu";
const char *const kControllerType = "handemu_hand";
const vr::TrackedDeviceIndex_t kInvalidIndex = vr::k_unTrackedDeviceIndexInvalid;
const vr::VRInputComponentHandle_t kInvalidHandle = vr::k_ulInvalidInputComponentHandle;

void Log( const char *fmt, ... )
{
	char buf[ 1024 ];
	va_list args;
	va_start( args, fmt );
	vsnprintf( buf, sizeof( buf ), fmt, args );
	va_end( args );
	if ( vr::VRDriverLog() )
		vr::VRDriverLog()->Log( buf );
}

double NowSeconds()
{
	static const double freq = []
	{
		LARGE_INTEGER f;
		QueryPerformanceFrequency( &f );
		return (double)f.QuadPart;
	}();
	LARGE_INTEGER c;
	QueryPerformanceCounter( &c );
	return (double)c.QuadPart / freq;
}

// ------------------------------------------------------------------------------------------------
// Settings (section "driver_handemu" in steamvr.vrsettings; defaults in resources/settings)
// ------------------------------------------------------------------------------------------------

struct Settings
{
	std::string sourceControllerType = "krvr_hand";
	bool forwardSystemButton = true;
	bool palmTapEnable = true;    // index fingertip taps the other palm -> system button
	float palmTapDistance = 0.035f; // fingertip to palm centre, metres
	bool calibrationGesture = true; // palms pressed together and held -> undo a Crown recentre
	float calibrationHoldTime = 1.5f; // seconds
	bool debugLog = false;
	Vec3 posOffset;    // metres, right hand; mirrored for the left
	Vec3 rotOffsetDeg; // pitch, yaw, roll; mirrored for the left
	GestureConfig gesture;
	SkeletonTuning skeleton; // startup values; debug\live.txt can override them at runtime
};

Settings g_settings;

// Skeleton tuning in effect (startup settings plus debug\live.txt), read on the source's thread.
std::mutex g_tuningMutex;
SkeletonTuning g_tuning;

SkeletonTuning CurrentTuning()
{
	std::lock_guard<std::mutex> lock( g_tuningMutex );
	return g_tuning;
}

DebugFolder g_debug;

float ReadFloat( const char *key, float def )
{
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	float v = vr::VRSettings()->GetFloat( kSettingsSection, key, &err );
	return err == vr::VRSettingsError_None ? v : def;
}

bool ReadBool( const char *key, bool def )
{
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	bool v = vr::VRSettings()->GetBool( kSettingsSection, key, &err );
	return err == vr::VRSettingsError_None ? v : def;
}

std::string ReadString( const char *key, const std::string &def )
{
	char buf[ 256 ] = {};
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	vr::VRSettings()->GetString( kSettingsSection, key, buf, sizeof( buf ), &err );
	return ( err == vr::VRSettingsError_None && buf[ 0 ] ) ? std::string( buf ) : def;
}

void LoadSettings()
{
	Settings s;
	s.sourceControllerType = ReadString( "sourceControllerType", s.sourceControllerType );
	s.forwardSystemButton = ReadBool( "forwardSystemButton", s.forwardSystemButton );
	s.palmTapEnable = ReadBool( "palmTapEnable", s.palmTapEnable );
	s.palmTapDistance = ReadFloat( "palmTapDistance", s.palmTapDistance );
	s.calibrationGesture = ReadBool( "calibrationGesture", s.calibrationGesture );
	s.calibrationHoldTime = ReadFloat( "calibrationHoldTime", s.calibrationHoldTime );
	s.debugLog = ReadBool( "debugLog", s.debugLog );
	s.posOffset = { ReadFloat( "posOffsetX", 0 ), ReadFloat( "posOffsetY", 0 ), ReadFloat( "posOffsetZ", 0 ) };
	s.rotOffsetDeg = { ReadFloat( "rotOffsetX", 0 ), ReadFloat( "rotOffsetY", 0 ), ReadFloat( "rotOffsetZ", 0 ) };

	GestureConfig &g = s.gesture;
	g.pinchOn = ReadFloat( "pinchOn", g.pinchOn );
	g.pinchOff = ReadFloat( "pinchOff", g.pinchOff );
	g.nearOn = ReadFloat( "nearOn", g.nearOn );
	g.nearOff = ReadFloat( "nearOff", g.nearOff );
	g.analogFar = ReadFloat( "analogFar", g.analogFar );
	g.fingerMargin = ReadFloat( "fingerMargin", g.fingerMargin );
	g.fistOn = ReadFloat( "fistOn", g.fistOn );
	g.fistOff = ReadFloat( "fistOff", g.fistOff );
	g.fistIndexOn = ReadFloat( "fistIndexOn", g.fistIndexOn );
	g.fistThumbNear = ReadFloat( "fistThumbNear", g.fistThumbNear );
	{
		vr::EVRSettingsError err = vr::VRSettingsError_None;
		int32_t frames = vr::VRSettings()->GetInt32( kSettingsSection, "fistFrames", &err );
		if ( err == vr::VRSettingsError_None && frames > 0 )
			g.fistFrames = frames;
	}
	g.smoothing = ReadFloat( "smoothing", g.smoothing );
	g.joystickEnable = ReadBool( "joystickEnable", g.joystickEnable );
	g.joystickRadius = ReadFloat( "joystickRadius", g.joystickRadius );
	g.joystickDeadzone = ReadFloat( "joystickDeadzone", g.joystickDeadzone );

	SkeletonTuning &k = s.skeleton;
	{
		vr::EVRSettingsError err = vr::VRSettingsError_None;
		int32_t mode = vr::VRSettings()->GetInt32( kSettingsSection, "skeletonMode", &err );
		if ( err == vr::VRSettingsError_None && ( mode == 0 || mode == 1 ) )
			k.mode = mode;
	}
	k.fullRange = ReadBool( "skeletonFullRange", k.fullRange );
	k.fingerGain = ReadFloat( "fingerCurlGain", k.fingerGain );
	k.thumbGain = ReadFloat( "thumbCurlGain", k.thumbGain );
	k.maxBend = ReadFloat( "maxBend", k.maxBend );
	k.logBends = ReadBool( "logBends", k.logBends );
	g_settings = s;
	std::lock_guard<std::mutex> lock( g_tuningMutex );
	g_tuning = s.skeleton;
}

// debug\live.txt: the same skeleton keys as steamvr.vrsettings, applied on top of the startup values.
std::string ApplyLiveTuning( const std::vector<std::pair<std::string, std::string>> &entries )
{
	SkeletonTuning t = g_settings.skeleton;
	std::string unknown;
	for ( const auto &e : entries )
	{
		const std::string &key = e.first;
		const char *v = e.second.c_str();
		bool on = e.second == "1" || _stricmp( v, "true" ) == 0 || _stricmp( v, "on" ) == 0 || _stricmp( v, "yes" ) == 0;
		if ( key == "skeletonMode" )
			t.mode = atoi( v ) == 0 ? 0 : 1;
		else if ( key == "skeletonFullRange" )
			t.fullRange = on;
		else if ( key == "fingerCurlGain" )
			t.fingerGain = (float)atof( v );
		else if ( key == "thumbCurlGain" )
			t.thumbGain = (float)atof( v );
		else if ( key == "maxBend" )
			t.maxBend = (float)atof( v );
		else if ( key == "logBends" )
			t.logBends = on;
		else
			unknown += ( unknown.empty() ? "" : ", " ) + key;
	}
	t.fingerGain = std::clamp( t.fingerGain, 0.2f, 3.0f );
	t.thumbGain = std::clamp( t.thumbGain, 0.2f, 3.0f );
	{
		std::lock_guard<std::mutex> lock( g_tuningMutex );
		g_tuning = t;
	}
	char buf[ 256 ];
	snprintf( buf, sizeof( buf ), "skeletonMode=%d skeletonFullRange=%d fingerCurlGain=%.2f thumbCurlGain=%.2f maxBend=%.0f logBends=%d",
		t.mode, t.fullRange ? 1 : 0, t.fingerGain, t.thumbGain, t.maxBend, t.logBends ? 1 : 0 );
	std::string msg = buf;
	if ( !unknown.empty() )
		msg += " (ignored unknown keys: " + unknown + ")";
	return msg;
}

bool HasPoseOffset()
{
	const Vec3 &p = g_settings.posOffset;
	const Vec3 &r = g_settings.rotOffsetDeg;
	return p.x != 0 || p.y != 0 || p.z != 0 || r.x != 0 || r.y != 0 || r.z != 0;
}

void PoseOffsetFor( int side, Vec3 &pos, Quat &rot )
{
	Vec3 p = g_settings.posOffset;
	Vec3 r = g_settings.rotOffsetDeg;
	if ( side == 0 )
	{
		p.x = -p.x;
		r.y = -r.y;
		r.z = -r.z;
	}
	pos = p;
	rot = FromEulerDegrees( r.x, r.y, r.z );
}

// Moves the emulated device frame by the configured offset (device-local).
void ApplyPoseOffset( vr::DriverPose_t &pose, int side )
{
	if ( !HasPoseOffset() )
		return;
	Vec3 p;
	Quat q;
	PoseOffsetFor( side, p, q );
	Quat headRot = FromVR( pose.qDriverFromHeadRotation );
	Vec3 headPos = FromVR( pose.vecDriverFromHeadTranslation );
	Vec3 newPos = headPos + Rotate( headRot, p );
	pose.vecDriverFromHeadTranslation[ 0 ] = newPos.x;
	pose.vecDriverFromHeadTranslation[ 1 ] = newPos.y;
	pose.vecDriverFromHeadTranslation[ 2 ] = newPos.z;
	pose.qDriverFromHeadRotation = ToVRd( headRot * q );
}

// Keeps the hand where it was when the device frame is offset: root' = offset^-1 * root.
void ApplySkeletonOffset( vr::VRBoneTransform_t *bones, int side )
{
	if ( !HasPoseOffset() )
		return;
	Vec3 p;
	Quat q;
	PoseOffsetFor( side, p, q );
	Quat inv = Conjugate( q );
	Vec3 rootPos{ bones[ 0 ].position.v[ 0 ], bones[ 0 ].position.v[ 1 ], bones[ 0 ].position.v[ 2 ] };
	Vec3 np = Rotate( inv, rootPos - p );
	bones[ 0 ].position.v[ 0 ] = (float)np.x;
	bones[ 0 ].position.v[ 1 ] = (float)np.y;
	bones[ 0 ].position.v[ 2 ] = (float)np.z;
	bones[ 0 ].orientation = ToVRf( inv * FromVR( bones[ 0 ].orientation ) );
}

Vec3 WorldPosition( const vr::DriverPose_t &pose )
{
	return Rotate( FromVR( pose.qWorldFromDriverRotation ), FromVR( pose.vecPosition ) ) + FromVR( pose.vecWorldFromDriverTranslation );
}

// ------------------------------------------------------------------------------------------------
// Hooks
// ------------------------------------------------------------------------------------------------

using PoseUpdatedFn = void ( * )( void *self, uint32_t device, const vr::DriverPose_t &pose, uint32_t size );
using CreateBooleanFn = vr::EVRInputError ( * )( void *self, vr::PropertyContainerHandle_t container, const char *name, vr::VRInputComponentHandle_t *handle );
using UpdateBooleanFn = vr::EVRInputError ( * )( void *self, vr::VRInputComponentHandle_t handle, bool value, double timeOffset );
using CreateSkeletonFn = vr::EVRInputError ( * )( void *self, vr::PropertyContainerHandle_t container, const char *name, const char *skeletonPath,
	const char *basePosePath, vr::EVRSkeletalTrackingLevel level, const vr::VRBoneTransform_t *gripLimit, uint32_t gripLimitCount, vr::VRInputComponentHandle_t *handle );
using UpdateSkeletonFn = vr::EVRInputError ( * )( void *self, vr::VRInputComponentHandle_t handle, vr::EVRSkeletalMotionRange range,
	const vr::VRBoneTransform_t *bones, uint32_t count );

// Vtable slots from openvr_driver.h; KRVR uses the same interface versions
// (IVRServerDriverHost_006, IVRDriverInput_004).
const int kSlotTrackedDevicePoseUpdated = 1;
const int kSlotCreateBooleanComponent = 0;
const int kSlotUpdateBooleanComponent = 1;
const int kSlotCreateSkeletonComponent = 5;
const int kSlotUpdateSkeletonComponent = 6;

struct Originals
{
	PoseUpdatedFn poseUpdated = nullptr;
	CreateBooleanFn createBoolean = nullptr;
	UpdateBooleanFn updateBoolean = nullptr;
	CreateSkeletonFn createSkeleton = nullptr;
	UpdateSkeletonFn updateSkeleton = nullptr;
};

Originals g_orig;
void *g_host = nullptr;  // our IVRServerDriverHost
void *g_input = nullptr; // our IVRDriverInput
bool g_hooksInstalled = false;
std::atomic<bool> g_active{ false };

// Our own updates go through the trampolines with our own interface pointers, so they skip the detours.
void SendBool( vr::VRInputComponentHandle_t h, bool v )
{
	if ( h == kInvalidHandle )
		return;
	if ( g_orig.updateBoolean )
		g_orig.updateBoolean( g_input, h, v, 0.0 );
	else
		vr::VRDriverInput()->UpdateBooleanComponent( h, v, 0.0 );
}

void SendScalar( vr::VRInputComponentHandle_t h, float v )
{
	if ( h != kInvalidHandle )
		vr::VRDriverInput()->UpdateScalarComponent( h, v, 0.0 );
}

void SendSkeleton( vr::VRInputComponentHandle_t h, vr::EVRSkeletalMotionRange range, const vr::VRBoneTransform_t *bones )
{
	if ( h == kInvalidHandle )
		return;
	if ( g_orig.updateSkeleton )
		g_orig.updateSkeleton( g_input, h, range, bones, kBoneCount );
	else
		vr::VRDriverInput()->UpdateSkeletonComponent( h, range, bones, kBoneCount );
}

void SendPose( uint32_t index, const vr::DriverPose_t &pose )
{
	if ( g_orig.poseUpdated )
		g_orig.poseUpdated( g_host, index, pose, sizeof( pose ) );
	else
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated( index, pose, sizeof( pose ) );
}

// ------------------------------------------------------------------------------------------------
// Device classification
// ------------------------------------------------------------------------------------------------

enum DeviceKind : int8_t
{
	kKindUnknown = 0,
	kKindOther,
	kKindHmd,
	kKindSourceLeft,
	kKindSourceRight,
	kKindOurs,
};

std::atomic<int8_t> g_kind[ vr::k_unMaxTrackedDeviceCount ];
std::atomic<uint16_t> g_kindTries[ vr::k_unMaxTrackedDeviceCount ];

// Returns 0 (left) / 1 (right) if the container belongs to a hand-tracking source device, else -1.
int SourceSideOfContainer( vr::PropertyContainerHandle_t container )
{
	vr::ETrackedPropertyError err = vr::TrackedProp_Success;
	std::string type = vr::VRProperties()->GetStringProperty( container, vr::Prop_ControllerType_String, &err );
	bool match = type == g_settings.sourceControllerType;
	if ( !match )
	{
		// SteamVR fills in the controller type from the input profile only after the device has
		// activated; while the source creates its components only the profile path is set.
		std::string profile = vr::VRProperties()->GetStringProperty( container, vr::Prop_InputProfilePath_String, &err );
		match = !profile.empty() && profile.find( g_settings.sourceControllerType ) != std::string::npos;
	}
	if ( !match )
		return -1;
	int32_t role = vr::VRProperties()->GetInt32Property( container, vr::Prop_ControllerRoleHint_Int32, &err );
	if ( err == vr::TrackedProp_Success )
	{
		if ( role == vr::TrackedControllerRole_LeftHand )
			return 0;
		if ( role == vr::TrackedControllerRole_RightHand )
			return 1;
	}
	std::string serial = vr::VRProperties()->GetStringProperty( container, vr::Prop_SerialNumber_String, &err );
	if ( !serial.empty() )
	{
		char last = serial.back();
		if ( last == 'L' || last == 'l' )
			return 0;
		if ( last == 'R' || last == 'r' )
			return 1;
	}
	return -1;
}

// ------------------------------------------------------------------------------------------------
// Headset frame, so the joystick drag is relative to where the user looks
// ------------------------------------------------------------------------------------------------

struct ViewFrame
{
	Vec3 right{ 1, 0, 0 };
	Vec3 forward{ 0, 0, -1 };
};

std::mutex g_viewMutex;
ViewFrame g_view;
std::atomic<double> g_viewTime{ 0 };

void OnHmdPose( const vr::DriverPose_t &pose )
{
	double now = NowSeconds();
	if ( now - g_viewTime.load( std::memory_order_relaxed ) < 0.02 )
		return;
	g_viewTime.store( now, std::memory_order_relaxed );

	Quat q = FromVR( pose.qWorldFromDriverRotation ) * FromVR( pose.qRotation ) * FromVR( pose.qDriverFromHeadRotation );
	Vec3 fwd = Rotate( q, { 0, 0, -1 } );
	fwd.y = 0;
	fwd = Normalized( fwd, { 0, 0, -1 } );
	ViewFrame v;
	v.forward = fwd;
	v.right = { -fwd.z, 0, fwd.x };
	std::lock_guard<std::mutex> lock( g_viewMutex );
	g_view = v;
}

ViewFrame CurrentView()
{
	std::lock_guard<std::mutex> lock( g_viewMutex );
	return g_view;
}

// ------------------------------------------------------------------------------------------------
// Emulated controller
// ------------------------------------------------------------------------------------------------

vr::DriverPose_t MakeIdlePose()
{
	vr::DriverPose_t pose{};
	pose.qWorldFromDriverRotation.w = 1;
	pose.qDriverFromHeadRotation.w = 1;
	pose.qRotation.w = 1;
	pose.result = vr::TrackingResult_Uninitialized;
	pose.poseIsValid = false;
	pose.deviceIsConnected = true;
	return pose;
}

struct Handles
{
	vr::VRInputComponentHandle_t systemClick = kInvalidHandle, systemTouch = kInvalidHandle;
	vr::VRInputComponentHandle_t triggerValue = kInvalidHandle, triggerClick = kInvalidHandle, triggerTouch = kInvalidHandle;
	vr::VRInputComponentHandle_t gripValue = kInvalidHandle, gripClick = kInvalidHandle, gripTouch = kInvalidHandle;
	vr::VRInputComponentHandle_t joyX = kInvalidHandle, joyY = kInvalidHandle, joyClick = kInvalidHandle, joyTouch = kInvalidHandle;
	vr::VRInputComponentHandle_t aClick = kInvalidHandle, aTouch = kInvalidHandle, bClick = kInvalidHandle, bTouch = kInvalidHandle;
	vr::VRInputComponentHandle_t thumbrestTouch = kInvalidHandle;
	vr::VRInputComponentHandle_t skeleton = kInvalidHandle;
	vr::VRInputComponentHandle_t haptic = kInvalidHandle;
};

std::atomic<vr::VRInputComponentHandle_t> g_ourSkeleton[ 2 ] = { kInvalidHandle, kInvalidHandle };

class HandDevice : public vr::ITrackedDeviceServerDriver
{
public:
	explicit HandDevice( int side ) : m_side( side ), m_serial( side == 0 ? "HANDEMU-L" : "HANDEMU-R" ), m_pose( MakeIdlePose() ) {}

	const char *Serial() const { return m_serial.c_str(); }
	bool IsActive() const { return m_index.load() != kInvalidIndex; }

	vr::EVRInitError Activate( uint32_t objectId ) override
	{
		const bool left = m_side == 0;
		vr::CVRPropertyHelpers *props = vr::VRProperties();
		vr::PropertyContainerHandle_t c = props->TrackedDeviceToPropertyContainer( objectId );
		if ( objectId < vr::k_unMaxTrackedDeviceCount )
			g_kind[ objectId ] = kKindOurs;

		props->SetStringProperty( c, vr::Prop_TrackingSystemName_String, "handemu" );
		props->SetStringProperty( c, vr::Prop_ManufacturerName_String, "HandEmu" );
		props->SetStringProperty( c, vr::Prop_ModelNumber_String, left ? "HandEmu Hand (Left)" : "HandEmu Hand (Right)" );
		props->SetStringProperty( c, vr::Prop_SerialNumber_String, m_serial.c_str() );
		props->SetStringProperty( c, vr::Prop_RegisteredDeviceType_String, left ? "handemu/HANDEMU-L" : "handemu/HANDEMU-R" );
		props->SetStringProperty( c, vr::Prop_RenderModelName_String, left ? "{handemu}/rendermodels/handemu_hand_left" : "{handemu}/rendermodels/handemu_hand_right" );
		props->SetStringProperty( c, vr::Prop_ControllerType_String, kControllerType );
		props->SetStringProperty( c, vr::Prop_InputProfilePath_String, "{handemu}/input/handemu_hand_profile.json" );
		props->SetInt32Property( c, vr::Prop_ControllerRoleHint_Int32, left ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand );
		// The source hands use hand_priority -1; we must win the hand roles or games keep reading them.
		props->SetInt32Property( c, vr::Prop_ControllerHandSelectionPriority_Int32, 100 );
		props->SetBoolProperty( c, vr::Prop_DeviceProvidesBatteryStatus_Bool, false );
		props->SetBoolProperty( c, vr::Prop_Identifiable_Bool, false );
		props->SetBoolProperty( c, vr::Prop_WillDriftInYaw_Bool, false );
		// Legacy (pre-SteamVR Input) view of the device, same as an Oculus Touch.
		props->SetInt32Property( c, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_Joystick );
		props->SetInt32Property( c, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger );
		props->SetInt32Property( c, vr::Prop_Axis2Type_Int32, vr::k_eControllerAxis_Trigger );
		props->SetUint64Property( c, vr::Prop_SupportedButtons_Uint64,
			vr::ButtonMaskFromId( vr::k_EButton_System ) | vr::ButtonMaskFromId( vr::k_EButton_ApplicationMenu ) |
			vr::ButtonMaskFromId( vr::k_EButton_Grip ) | vr::ButtonMaskFromId( vr::k_EButton_A ) |
			vr::ButtonMaskFromId( vr::k_EButton_Axis0 ) | vr::ButtonMaskFromId( vr::k_EButton_Axis1 ) |
			vr::ButtonMaskFromId( vr::k_EButton_Axis2 ) );

		vr::IVRDriverInput *in = vr::VRDriverInput();
		Handles &h = m_h;
		in->CreateBooleanComponent( c, "/input/system/click", &h.systemClick );
		in->CreateBooleanComponent( c, "/input/system/touch", &h.systemTouch );
		in->CreateScalarComponent( c, "/input/trigger/value", &h.triggerValue, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided );
		in->CreateBooleanComponent( c, "/input/trigger/click", &h.triggerClick );
		in->CreateBooleanComponent( c, "/input/trigger/touch", &h.triggerTouch );
		in->CreateScalarComponent( c, "/input/grip/value", &h.gripValue, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided );
		in->CreateBooleanComponent( c, "/input/grip/click", &h.gripClick );
		in->CreateBooleanComponent( c, "/input/grip/touch", &h.gripTouch );
		in->CreateScalarComponent( c, "/input/joystick/x", &h.joyX, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided );
		in->CreateScalarComponent( c, "/input/joystick/y", &h.joyY, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided );
		in->CreateBooleanComponent( c, "/input/joystick/click", &h.joyClick );
		in->CreateBooleanComponent( c, "/input/joystick/touch", &h.joyTouch );
		in->CreateBooleanComponent( c, left ? "/input/x/click" : "/input/a/click", &h.aClick );
		in->CreateBooleanComponent( c, left ? "/input/x/touch" : "/input/a/touch", &h.aTouch );
		in->CreateBooleanComponent( c, left ? "/input/y/click" : "/input/b/click", &h.bClick );
		in->CreateBooleanComponent( c, left ? "/input/y/touch" : "/input/b/touch", &h.bTouch );
		in->CreateBooleanComponent( c, "/input/thumbrest/touch", &h.thumbrestTouch );
		in->CreateSkeletonComponent( c, left ? "/input/skeleton/left" : "/input/skeleton/right", left ? "/skeleton/hand/left" : "/skeleton/hand/right",
			"/pose/raw", vr::VRSkeletalTracking_Full, nullptr, 0, &h.skeleton );
		in->CreateHapticComponent( c, "/output/haptic", &h.haptic );
		g_ourSkeleton[ m_side ] = h.skeleton;

		m_forceSend = true;
		m_index = objectId; // published last: detours only touch us once components exist
		Log( "Emulated %s controller active (device %u)", left ? "left" : "right", objectId );
		return vr::VRInitError_None;
	}

	void Deactivate() override { m_index = kInvalidIndex; }
	void EnterStandby() override {}
	void *GetComponent( const char * ) override { return nullptr; }

	void DebugRequest( const char *, char *response, uint32_t size ) override
	{
		if ( size > 0 )
			response[ 0 ] = 0;
	}

	vr::DriverPose_t GetPose() override
	{
		std::lock_guard<std::mutex> lock( m_poseMutex );
		return m_pose;
	}

	void PostPose( const vr::DriverPose_t &pose )
	{
		uint32_t index = m_index.load();
		if ( index == kInvalidIndex )
			return;
		{
			std::lock_guard<std::mutex> lock( m_poseMutex );
			m_pose = pose;
		}
		SendPose( index, pose );
	}

	void PostSkeleton( vr::EVRSkeletalMotionRange range, const vr::VRBoneTransform_t *bones )
	{
		if ( IsActive() )
			SendSkeleton( m_h.skeleton, range, bones );
	}

	void PostSystem( bool pressed )
	{
		if ( !IsActive() )
			return;
		std::lock_guard<std::mutex> lock( m_inputMutex );
		SendBool( m_h.systemClick, pressed );
		SendBool( m_h.systemTouch, pressed );
	}

	void PostInputs( const GestureOutput &o )
	{
		if ( !IsActive() )
			return;
		std::lock_guard<std::mutex> lock( m_inputMutex );
		const bool force = m_forceSend;
		m_forceSend = false;
		GestureOutput &last = m_last;
		auto b = [ force ]( vr::VRInputComponentHandle_t h, bool v, bool &prev )
		{
			if ( force || v != prev )
				SendBool( h, v );
			prev = v;
		};
		auto s = [ force ]( vr::VRInputComponentHandle_t h, float v, float &prev )
		{
			if ( force || std::fabs( v - prev ) > 0.0005f )
				SendScalar( h, v );
			prev = v;
		};
		s( m_h.triggerValue, o.triggerValue, last.triggerValue );
		b( m_h.triggerClick, o.triggerClick, last.triggerClick );
		b( m_h.triggerTouch, o.triggerTouch, last.triggerTouch );
		s( m_h.gripValue, o.gripValue, last.gripValue );
		b( m_h.gripClick, o.gripClick, last.gripClick );
		b( m_h.gripTouch, o.gripTouch, last.gripTouch );
		b( m_h.aClick, o.aClick, last.aClick );
		b( m_h.aTouch, o.aTouch, last.aTouch );
		b( m_h.bClick, o.bClick, last.bClick );
		b( m_h.bTouch, o.bTouch, last.bTouch );
		s( m_h.joyX, o.joyX, last.joyX );
		s( m_h.joyY, o.joyY, last.joyY );
		b( m_h.joyClick, o.joyClick, last.joyClick );
		b( m_h.joyTouch, o.joyTouch, last.joyTouch );
	}

private:
	const int m_side;
	const std::string m_serial;
	std::atomic<uint32_t> m_index{ kInvalidIndex };
	Handles m_h;

	std::mutex m_poseMutex;
	vr::DriverPose_t m_pose;

	std::mutex m_inputMutex;
	GestureOutput m_last;
	bool m_forceSend = true;
};

// ------------------------------------------------------------------------------------------------
// Per-hand state: what the source driver sent, and the emulated device it feeds
// ------------------------------------------------------------------------------------------------

struct HandState
{
	std::mutex mutex;
	bool sourceSeen = false;
	vr::DriverPose_t srcPose{};
	bool havePose = false;
	double poseTime = 0;
	bool stalePoseSent = false;
	bool haveSkeleton = false;
	double skeletonTime = 0;
	double withoutControllerTime = -1;
	bool staleSkeletonReset = false;
	GestureRecognizer recognizer;
	double lastDebugLog = 0;
	double lastBendLog = 0;
	HandDevice *device = nullptr;

	// Diagnostics
	bool loggedFirstSkeleton = false;
	uint32_t poseFrames = 0;
	uint32_t skeletonFrames = 0;
	int lastPinch = kFingerNone;
	bool lastFist = false;
	bool lastJoy = false;

	std::atomic<uint32_t> sourceIndex{ kInvalidIndex };
	std::atomic<vr::VRInputComponentHandle_t> srcSkeleton{ kInvalidHandle };
	std::atomic<vr::VRInputComponentHandle_t> srcSystem{ kInvalidHandle };
};

HandState g_hands[ 2 ];

const char *SideName( int side ) { return side == 0 ? "left" : "right"; }

// Every boolean / skeleton component created while hooked. A source's properties can still be
// incomplete when it creates its components, so they are matched up once the device is recognised.
struct CreatedComponent
{
	vr::PropertyContainerHandle_t container;
	vr::VRInputComponentHandle_t handle;
	char name[ 40 ];
};

std::mutex g_createdMutex;
std::vector<CreatedComponent> g_created;

void RecordComponent( vr::PropertyContainerHandle_t container, vr::VRInputComponentHandle_t handle, const char *name )
{
	std::lock_guard<std::mutex> lock( g_createdMutex );
	if ( g_created.size() >= 1024 )
		return;
	CreatedComponent c{ container, handle, {} };
	strncpy_s( c.name, name, _TRUNCATE );
	g_created.push_back( c );
}

void ClaimSourceComponent( std::atomic<vr::VRInputComponentHandle_t> &slot, vr::VRInputComponentHandle_t handle, const char *what, int side )
{
	vr::VRInputComponentHandle_t expected = kInvalidHandle;
	if ( slot.compare_exchange_strong( expected, handle ) )
		Log( "Hooked %s source %s", SideName( side ), what );
}

void ResolveSourceComponents( int side, vr::PropertyContainerHandle_t container )
{
	std::lock_guard<std::mutex> lock( g_createdMutex );
	for ( const CreatedComponent &c : g_created )
	{
		if ( c.container != container )
			continue;
		if ( strncmp( c.name, "/input/skeleton/", 16 ) == 0 )
			ClaimSourceComponent( g_hands[ side ].srcSkeleton, c.handle, "skeleton", side );
		else if ( strcmp( c.name, "/input/system/click" ) == 0 )
			ClaimSourceComponent( g_hands[ side ].srcSystem, c.handle, "system button", side );
	}
}

// Two-hand gestures:
//   palm tap        - index fingertip taps the other palm (in and out within a second) -> system button
//   palms together  - both palms pressed together and held -> hand alignment calibration

// Joints paired up between the hands for "palms together": the two finger joints after the index,
// middle and ring knuckles. Pressed palm to palm the fingers meet within a couple of centimetres,
// while Vision Pro tends to report the wrists 9-10 cm apart in that pose (a palm-centre point
// therefore sits right on any sensible threshold). The middle knuckles only need to be near.
const int kTogetherJoints[ 6 ] = { 8, 9, 13, 14, 18, 19 };
const int kKnuckle = 12;           // middle knuckle
const float kTogetherOn = 0.04f;   // typical (median) gap between paired finger joints, metres
const float kKnuckleMax = 0.09f;   // middle knuckles closer than this
const float kTogetherOff = 0.08f;  // fingers clearly apart again: re-arms the gesture
const float kTapBlockGap = 0.07f;  // closer than this the hands are pressed together, not tapping
const double kTogetherGrace = 0.3; // tracking may drop the pose this long without restarting the hold

struct HandPoints
{
	Vec3 tip;     // index fingertip
	Vec3 palm;    // palm centre (wrist .. middle knuckle)
	Vec3 knuckle; // middle knuckle
	Vec3 match[ 6 ];
	bool indexFree = false;
	bool open = false; // not pinching, not a fist
};

struct TapHand
{
	HandPoints pts;
	double time = -1;
	bool touching = false;
	double touchStart = 0;
	float lastDist = 1.0f;
};

std::mutex g_tapMutex;
TapHand g_tap[ 2 ];
std::atomic<double> g_systemPulseUntil[ 2 ];
double g_palmsTogetherSince = -1;
double g_palmsTogetherSeen = -1;
double g_palmsApartSince = -1;
bool g_palmsTogetherFired = false;
float g_palmGap = 1.0f;

struct TwoHandResult
{
	bool tapped = false;    // this hand's fingertip just tapped the other palm
	bool calibrate = false; // palms have been held together long enough
	std::string note;       // palms-together state change, for the debug log
};

// Typical distance between paired joints of the two hands (median of the six pairs).
float PalmGap( const HandPoints &a, const HandPoints &b )
{
	float gaps[ 6 ];
	for ( int k = 0; k < 6; k++ )
		gaps[ k ] = (float)Distance( a.match[ k ], b.match[ k ] );
	std::sort( gaps, gaps + 6 );
	return 0.5f * ( gaps[ 2 ] + gaps[ 3 ] );
}

TwoHandResult UpdateTwoHandGestures( int side, const HandPoints &pts, double now )
{
	TwoHandResult result;
	std::lock_guard<std::mutex> lock( g_tapMutex );
	TapHand &me = g_tap[ side ];
	const TapHand &other = g_tap[ 1 - side ];
	me.pts = pts;
	me.time = now;
	if ( other.time < 0 || now - other.time > 0.1 )
	{
		// The other hand is not being tracked right now. A palms-together hold survives short gaps
		// (see kTogetherGrace) and lapses on its own otherwise.
		me.touching = false;
		return result;
	}

	// Both hands get the same alignment correction, so their relative distances are reliable even
	// while the hands are misaligned with the head.
	const float gap = PalmGap( pts, other.pts );
	g_palmGap = gap;
	if ( g_settings.calibrationGesture )
	{
		char buf[ 160 ];
		const bool knucklesNear = Distance( pts.knuckle, other.pts.knuckle ) < kKnuckleMax;
		if ( gap > kTogetherOff )
		{
			if ( g_palmsApartSince < 0 )
				g_palmsApartSince = now;
		}
		else
		{
			g_palmsApartSince = -1;
		}
		if ( gap < kTogetherOn && knucklesNear && pts.open && other.pts.open )
		{
			if ( g_palmsTogetherSince < 0 || now - g_palmsTogetherSeen > kTogetherGrace )
			{
				g_palmsTogetherSince = now;
				if ( !g_palmsTogetherFired )
				{
					snprintf( buf, sizeof( buf ), "Palms together (joints %.0f mm apart) - hold %.1f s to realign the hands", gap * 1000,
						g_settings.calibrationHoldTime );
					result.note = buf;
				}
			}
			g_palmsTogetherSeen = now;
			if ( !g_palmsTogetherFired && now - g_palmsTogetherSince >= g_settings.calibrationHoldTime )
			{
				g_palmsTogetherFired = true;
				result.calibrate = true;
			}
		}
		// Apart for a moment is not enough: when a calibration lands, the two hands pick up the
		// correction one pose update apart and briefly look far apart.
		else if ( g_palmsApartSince >= 0 && now - g_palmsApartSince >= kTogetherGrace && g_palmsTogetherSince >= 0 )
		{
			if ( !g_palmsTogetherFired )
			{
				snprintf( buf, sizeof( buf ), "Palms apart again after %.1f s (not held long enough)", g_palmsTogetherSeen - g_palmsTogetherSince );
				result.note = buf;
			}
			g_palmsTogetherSince = -1;
			g_palmsTogetherFired = false;
		}
	}

	if ( !g_settings.palmTapEnable )
		return result;
	const float dist = (float)Distance( pts.tip, other.pts.palm );
	me.lastDist = dist;
	const float on = g_settings.palmTapDistance;
	const float off = on + 0.02f;
	const bool indexFree = pts.indexFree;
	if ( gap < kTapBlockGap )
	{
		me.touching = false; // hands pressed together is the calibration pose, never a tap
	}
	else if ( !me.touching )
	{
		if ( dist < on && indexFree )
		{
			me.touching = true;
			me.touchStart = now;
		}
	}
	else if ( dist > off )
	{
		me.touching = false;
		result.tapped = now - me.touchStart < 1.0;
	}
	return result;
}

float PalmTapDistance( int side )
{
	std::lock_guard<std::mutex> lock( g_tapMutex );
	return g_tap[ side ].lastDist;
}

float LastPalmGap()
{
	std::lock_guard<std::mutex> lock( g_tapMutex );
	return g_palmGap;
}

Vec3 DeviceToWorld( const vr::DriverPose_t &pose, const Vec3 &p )
{
	Vec3 head = Rotate( FromVR( pose.qDriverFromHeadRotation ), p ) + FromVR( pose.vecDriverFromHeadTranslation );
	Vec3 driver = Rotate( FromVR( pose.qRotation ), head ) + FromVR( pose.vecPosition );
	return Rotate( FromVR( pose.qWorldFromDriverRotation ), driver ) + FromVR( pose.vecWorldFromDriverTranslation );
}

Quat DeviceWorldRotation( const vr::DriverPose_t &pose )
{
	return NormalizedQ( FromVR( pose.qWorldFromDriverRotation ) * FromVR( pose.qRotation ) * FromVR( pose.qDriverFromHeadRotation ) );
}

// Moves the whole device in world space: world' = c * world.
void ApplyWorldCorrection( vr::DriverPose_t &pose, const Rigid &c )
{
	Quat q = NormalizedQ( c.q * FromVR( pose.qWorldFromDriverRotation ) );
	Vec3 t = Apply( c, FromVR( pose.vecWorldFromDriverTranslation ) );
	pose.qWorldFromDriverRotation = ToVRd( q );
	pose.vecWorldFromDriverTranslation[ 0 ] = t.x;
	pose.vecWorldFromDriverTranslation[ 1 ] = t.y;
	pose.vecWorldFromDriverTranslation[ 2 ] = t.z;
}

// ------------------------------------------------------------------------------------------------
// Hand alignment
//
// After a Digital Crown recentre KRVR keeps the headset continuous but sends the hands turned and
// shifted by one rigid transform (yaw + horizontal shift). Such a jump is only remembered here:
// both hands leap by the same transform at once while the head holds still. Nothing is changed
// until the user holds their palms together, which undoes everything remembered so far.
// ------------------------------------------------------------------------------------------------

class HandAlignment
{
public:
	Rigid Correction()
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		return m_correction;
	}

	void OnHand( int side, const Vec3 &p, const Quat &q, bool tracked, double now )
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		Sample &prev = m_prev[ side ];
		Leap &leap = m_leap[ side ];
		if ( !tracked )
		{
			prev.t = -1;
			leap = Leap{};
			return;
		}
		Sample cur{ p, q, now };
		if ( prev.t > 0 && now > prev.t && now - prev.t < 0.05 )
		{
			double speed = Distance( cur.p, prev.p ) / ( now - prev.t );
			// A new leap replaces a finished one that is still waiting for its partner.
			if ( leap.state != Leap::kMoving && speed > kLeapSpeed )
			{
				leap.state = Leap::kMoving;
				leap.start = now;
				leap.before = prev;
				leap.quiet = 0;
			}
			else if ( leap.state == Leap::kMoving )
			{
				leap.quiet = speed < kQuietSpeed ? leap.quiet + 1 : 0;
				if ( leap.quiet >= 2 )
				{
					leap.state = Leap::kDone;
					leap.end = now;
					leap.after = cur;
					// Pressing the Crown takes one hand out of view. KRVR stops that hand's skeleton
					// while it is out, so then this hand alone has to tell the story.
					if ( now - m_lastSkeleton[ 1 - side ] > 0.15 && m_leap[ 1 - side ].state == Leap::kIdle )
					{
						EvaluateAlone( side );
						leap = Leap{};
					}
				}
				else if ( now - leap.start > 0.25 )
				{
					leap = Leap{}; // an ordinary fast movement, not a teleport
				}
			}
		}
		prev = cur;
		Evaluate( now );
	}

	void OnSkeleton( int side, double now )
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		m_lastSkeleton[ side ] = now;
	}

	void OnHmd( const Vec3 &p, const Quat &q, bool connected, double now )
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		if ( !connected )
		{
			// A new KRVR session lines the frames up again, so drop whatever was remembered.
			if ( m_hmdLostSince < 0 )
				m_hmdLostSince = now;
			else if ( now - m_hmdLostSince > 1.0 && ( m_jumpCount > 0 || !IsIdentity( m_correction ) ) )
			{
				m_jumps = Rigid{};
				m_correction = Rigid{};
				m_jumpCount = 0;
				Log( "Headset disconnected: hand alignment reset" );
			}
			return;
		}
		m_hmdLostSince = -1;
		if ( now - m_hmd[ ( m_hmdNext + kHmdSamples - 1 ) % kHmdSamples ].t < 0.005 )
			return;
		m_hmd[ m_hmdNext ] = Sample{ p, q, now };
		m_hmdNext = ( m_hmdNext + 1 ) % kHmdSamples;
	}

	// The calibration gesture: undo every remembered jump.
	void Calibrate()
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		if ( m_jumpCount == 0 )
		{
			Log( "Calibration gesture: no Crown recentre recorded in this session, hands left as they are" );
			return;
		}
		m_correction = Inverse( m_jumps );
		Log( "Calibration gesture: undid %d recorded jump(s), total yaw %.1f deg, shift at the head %.0f cm", m_jumpCount,
			YawOf( m_correction.q ) * kRadToDeg, ShiftAtHead( m_correction ) * 100 );
	}

private:
	static constexpr double kLeapSpeed = 4.0;  // m/s between two samples: faster than a hand moves
	static constexpr double kQuietSpeed = 0.5; // m/s: the leap is over
	static constexpr double kRadToDeg = 57.29577951308232;
	static const int kHmdSamples = 48;

	struct Sample
	{
		Vec3 p;
		Quat q;
		double t = -1;
	};

	struct Leap
	{
		enum State
		{
			kIdle,
			kMoving,
			kDone
		} state = kIdle;
		double start = 0;
		double end = 0;
		Sample before;
		Sample after;
		int quiet = 0;
	};

	static Rigid JumpOf( const Leap &l )
	{
		Rigid t;
		t.q = NormalizedQ( l.after.q * Conjugate( l.before.q ) );
		t.t = l.after.p - Rotate( t.q, l.before.p );
		return t;
	}

	// A recentre only turns about the vertical and shifts sideways: heights and tilt stay put.
	static bool LooksLikeRecentre( const Rigid &t, const Leap &a, const Leap &b, double minShift )
	{
		Vec3 da = a.after.p - a.before.p;
		Vec3 db = b.after.p - b.before.p;
		double vertical = std::max( std::fabs( da.y ), std::fabs( db.y ) );
		double shift = std::max( std::hypot( da.x, da.z ), std::hypot( db.x, db.z ) );
		double yawDeg = std::fabs( YawOf( t.q ) ) * kRadToDeg;
		double swingDeg = SwingAngle( t.q ) * kRadToDeg;
		return vertical < 0.03 && swingDeg < 6.0 && ( yawDeg >= 3.0 || shift >= minShift );
	}

	double ShiftAtHead( const Rigid &t ) const
	{
		const Sample &head = m_hmd[ ( m_hmdNext + kHmdSamples - 1 ) % kHmdSamples ];
		Vec3 at = head.t > 0 ? head.p : Vec3{};
		return Distance( Apply( t, at ), at );
	}

	bool HeadSteady( double t0, double t1 ) const
	{
		const Sample *first = nullptr;
		const Sample *last = nullptr;
		for ( const Sample &s : m_hmd )
		{
			if ( s.t < 0 )
				continue;
			if ( !first || std::fabs( s.t - t0 ) < std::fabs( first->t - t0 ) )
				first = &s;
			if ( !last || std::fabs( s.t - t1 ) < std::fabs( last->t - t1 ) )
				last = &s;
		}
		if ( !first || !last )
			return true;
		return Distance( first->p, last->p ) < 0.03 && AngleOf( last->q * Conjugate( first->q ) ) * kRadToDeg < 5.0;
	}

	void Remember( const Rigid &jump, const char *how )
	{
		m_jumps = Compose( jump, m_jumps );
		m_jumpCount++;
		Log( "Hand frame jump recorded (%s): yaw %.1f deg, shift at the head %.0f cm - hold your palms together to realign",
			how, YawOf( jump.q ) * kRadToDeg, ShiftAtHead( jump ) * 100 );
	}

	// One tracked hand, the other out of view: only an unmistakable recentre counts - an instant
	// jump with a real turn, no tilt and no height change.
	void EvaluateAlone( int side )
	{
		const Leap &me = m_leap[ side ];
		Rigid t = JumpOf( me );
		Vec3 d = me.after.p - me.before.p;
		double yawDeg = std::fabs( YawOf( t.q ) ) * kRadToDeg;
		double swingDeg = SwingAngle( t.q ) * kRadToDeg;
		bool steady = HeadSteady( me.start - 0.03, me.end );
		if ( me.end - me.start < 0.05 && yawDeg >= 3.0 && swingDeg < 4.0 && std::fabs( d.y ) < 0.02 && steady )
			Remember( t, side == 0 ? "left hand, right hand out of view" : "right hand, left hand out of view" );
		else if ( g_settings.debugLog && Length( d ) > 0.10 )
			Log( "%s hand leapt %.0f cm with the other hand out of view, but not like a recentre (yaw %.1f, tilt %.1f deg, height %.0f mm, head steady %d)",
				side == 0 ? "left" : "right", Length( d ) * 100, yawDeg, swingDeg, d.y * 1000, steady ? 1 : 0 );
	}

	void Evaluate( double now )
	{
		Leap &a = m_leap[ 0 ];
		Leap &b = m_leap[ 1 ];
		if ( a.state == Leap::kDone && b.state == Leap::kDone )
		{
			if ( std::fabs( a.start - b.start ) < 0.06 )
			{
				Rigid ta = JumpOf( a );
				Rigid tb = JumpOf( b );
				double rotDiffDeg = AngleOf( ta.q * Conjugate( tb.q ) ) * kRadToDeg;
				double cross = std::max( Distance( Apply( ta, b.before.p ), b.after.p ), Distance( Apply( tb, a.before.p ), a.after.p ) );
				Rigid t;
				t.q = NormalizedQ( Dot( ta.q, tb.q ) < 0 ? Quat{ ta.q.w - tb.q.w, ta.q.x - tb.q.x, ta.q.y - tb.q.y, ta.q.z - tb.q.z }
														: Quat{ ta.q.w + tb.q.w, ta.q.x + tb.q.x, ta.q.y + tb.q.y, ta.q.z + tb.q.z } );
				t.t = ( ( a.after.p - Rotate( t.q, a.before.p ) ) + ( b.after.p - Rotate( t.q, b.before.p ) ) ) * 0.5;
				bool steady = HeadSteady( std::min( a.start, b.start ) - 0.03, std::max( a.end, b.end ) );
				if ( rotDiffDeg < 8.0 && cross < 0.06 && steady && LooksLikeRecentre( t, a, b, 0.08 ) )
					Remember( t, "both hands" );
				else if ( g_settings.debugLog )
					Log( "Both hands leapt but not as one rigid recentre (rot diff %.1f deg, mismatch %.0f mm, head steady %d)", rotDiffDeg, cross * 1000, steady ? 1 : 0 );
			}
			a = Leap{};
			b = Leap{};
			return;
		}

		// A single hand leaping (moved fast, or re-acquired somewhere else) is never a recentre:
		// give the other hand a moment to leap too, then forget it.
		for ( int side = 0; side < 2; side++ )
		{
			Leap &me = m_leap[ side ];
			if ( me.state != Leap::kDone )
				continue;
			if ( m_leap[ 1 - side ].state == Leap::kMoving && now - me.end < 0.1 )
				continue;
			if ( now - me.end > 0.06 )
			{
				double moved = Distance( me.after.p, me.before.p );
				if ( g_settings.debugLog && moved > 0.10 )
					Log( "%s hand leapt %.0f cm on its own (other hand did not follow) - not a recentre", side == 0 ? "left" : "right", moved * 100 );
				me = Leap{};
			}
		}
	}

	std::mutex m_mutex;
	Rigid m_correction; // applied to every hand pose
	Rigid m_jumps;      // every remembered jump, composed
	int m_jumpCount = 0;
	Sample m_prev[ 2 ];
	Leap m_leap[ 2 ];
	double m_lastSkeleton[ 2 ] = { -1, -1 };
	Sample m_hmd[ kHmdSamples ];
	int m_hmdNext = 0;
	double m_hmdLostSince = -1;
};

HandAlignment g_alignment;

DeviceKind ClassifyDevice( uint32_t index )
{
	if ( index >= vr::k_unMaxTrackedDeviceCount )
		return kKindOther;
	int8_t cached = g_kind[ index ].load( std::memory_order_relaxed );
	if ( cached != kKindUnknown )
		return (DeviceKind)cached;

	vr::PropertyContainerHandle_t c = vr::VRProperties()->TrackedDeviceToPropertyContainer( index );
	if ( c == vr::k_ulInvalidPropertyContainer )
		return kKindOther;

	vr::ETrackedPropertyError err = vr::TrackedProp_Success;
	int32_t deviceClass = vr::VRProperties()->GetInt32Property( c, vr::Prop_DeviceClass_Int32, &err );
	if ( index == vr::k_unTrackedDeviceIndex_Hmd || ( err == vr::TrackedProp_Success && deviceClass == vr::TrackedDeviceClass_HMD ) )
	{
		g_kind[ index ] = kKindHmd;
		return kKindHmd;
	}

	std::string type = vr::VRProperties()->GetStringProperty( c, vr::Prop_ControllerType_String, &err );
	if ( !type.empty() )
	{
		int side = SourceSideOfContainer( c );
		DeviceKind kind = side == 0 ? kKindSourceLeft : side == 1 ? kKindSourceRight : kKindOther;
		g_kind[ index ] = kind;
		if ( side >= 0 )
		{
			g_hands[ side ].sourceIndex = index;
			Log( "Found %s hand-tracking source (device %u, type %s)", SideName( side ), index, type.c_str() );
			ResolveSourceComponents( side, c );
		}
		return kind;
	}

	// Properties may not be set yet on a brand new device; look again later, but not forever.
	if ( g_kindTries[ index ].fetch_add( 1 ) > 500 )
		g_kind[ index ] = kKindOther;
	return kKindOther;
}

void LogGestures( int side, const GestureDebug &d, const GestureOutput &o )
{
	static const char *const kPinchName[] = { "index", "middle", "ring", "pinky" };
	Log( "%s: dist(mm) I=%.0f M=%.0f R=%.0f P=%.0f curl I=%.2f M=%.2f R=%.2f P=%.2f fist=%.2f tap=%.0f palms=%.0f | pinch=%s fist=%d | "
		 "trigger=%.2f%s grip=%.2f%s A/X=%d B/Y=%d joy=(%.2f,%.2f)%s",
		SideName( side ), d.dist[ 0 ] * 1000, d.dist[ 1 ] * 1000, d.dist[ 2 ] * 1000, d.dist[ 3 ] * 1000,
		d.curl[ 0 ], d.curl[ 1 ], d.curl[ 2 ], d.curl[ 3 ], d.fistScore, PalmTapDistance( side ) * 1000, LastPalmGap() * 1000,
		d.pinch >= 0 ? kPinchName[ d.pinch ] : "none", d.fist ? 1 : 0,
		o.triggerValue, o.triggerClick ? "*" : "", o.gripValue, o.gripClick ? "*" : "",
		o.aClick ? 1 : 0, o.bClick ? 1 : 0, o.joyX, o.joyY, o.joyTouch ? "*" : "" );
}

// Gesture state transitions as one log line, so each pinch or fist shows up the moment it happens.
std::string DescribeChanges( int side, HandState &h, const GestureDebug &d, const GestureOutput &o )
{
	static const char *const kPinchName[] = { "index", "middle", "ring", "pinky" };
	std::string events;
	char buf[ 256 ];
	auto add = [ & ]()
	{
		if ( !events.empty() )
			events += " | ";
		events += buf;
	};

	if ( d.pinch != h.lastPinch )
	{
		if ( h.lastPinch != kFingerNone )
		{
			snprintf( buf, sizeof( buf ), "%s: %s pinch OFF (closest %.0f mm)", SideName( side ), kPinchName[ h.lastPinch ], d.pinchMin * 1000 );
			add();
		}
		if ( d.pinch != kFingerNone )
		{
			snprintf( buf, sizeof( buf ), "%s: %s pinch ON (mm I=%.0f M=%.0f R=%.0f P=%.0f)", SideName( side ), kPinchName[ d.pinch ],
				d.dist[ 0 ] * 1000, d.dist[ 1 ] * 1000, d.dist[ 2 ] * 1000, d.dist[ 3 ] * 1000 );
			add();
		}
		h.lastPinch = d.pinch;
	}
	if ( d.fist != h.lastFist )
	{
		snprintf( buf, sizeof( buf ), "%s: fist %s (curl I=%.2f M=%.2f R=%.2f P=%.2f)", SideName( side ), d.fist ? "ON" : "OFF",
			d.curl[ 0 ], d.curl[ 1 ], d.curl[ 2 ], d.curl[ 3 ] );
		add();
		h.lastFist = d.fist;
	}
	if ( o.joyTouch != h.lastJoy )
	{
		snprintf( buf, sizeof( buf ), "%s: joystick drag %s", SideName( side ), o.joyTouch ? "ON" : "OFF" );
		add();
		h.lastJoy = o.joyTouch;
	}
	return events;
}

void OnHmdForAlignment( const vr::DriverPose_t &pose )
{
	g_alignment.OnHmd( DeviceToWorld( pose, {} ), DeviceWorldRotation( pose ), pose.deviceIsConnected, NowSeconds() );
}

void OnSourcePose( int side, const vr::DriverPose_t &in, uint32_t size )
{
	HandState &h = g_hands[ side ];
	vr::DriverPose_t pose{};
	memcpy( &pose, &in, std::min<size_t>( size, sizeof( pose ) ) );
	g_debug.RecordPose( side, pose, NowSeconds() );

	// Jumps are spotted on the pose as KRVR sends it; the correction (if the user calibrated) is
	// applied to everything downstream, including the gesture positions.
	bool tracked = pose.poseIsValid && pose.deviceIsConnected && pose.result == vr::TrackingResult_Running_OK;
	g_alignment.OnHand( side, DeviceToWorld( pose, {} ), DeviceWorldRotation( pose ), tracked, NowSeconds() );
	Rigid correction = g_alignment.Correction();
	if ( !IsIdentity( correction ) )
		ApplyWorldCorrection( pose, correction );

	HandDevice *device;
	{
		std::lock_guard<std::mutex> lock( h.mutex );
		h.srcPose = pose;
		h.havePose = true;
		h.poseTime = NowSeconds();
		h.stalePoseSent = false;
		h.sourceSeen = true;
		h.poseFrames++;
		device = h.device;
	}
	if ( device && device->IsActive() )
	{
		ApplyPoseOffset( pose, side );
		device->PostPose( pose );
	}
}

void LogBends( int side, const vr::VRBoneTransform_t *source, const vr::VRBoneTransform_t *sent )
{
	float a[ 15 ], b[ 15 ];
	MeasureBends( source, a );
	MeasureBends( sent, b );
	char part[ 2 ][ 200 ];
	const float *v[ 2 ] = { a, b };
	for ( int k = 0; k < 2; k++ )
	{
		const float *x = v[ k ];
		snprintf( part[ k ], sizeof( part[ k ] ), "T %.0f/%.0f/%.0f I %.0f/%.0f/%.0f M %.0f/%.0f/%.0f R %.0f/%.0f/%.0f P %.0f/%.0f/%.0f",
			x[ 0 ], x[ 1 ], x[ 2 ], x[ 3 ], x[ 4 ], x[ 5 ], x[ 6 ], x[ 7 ], x[ 8 ], x[ 9 ], x[ 10 ], x[ 11 ], x[ 12 ], x[ 13 ], x[ 14 ] );
	}
	Log( "%s bends (deg) source: %s | sent: %s", SideName( side ), part[ 0 ], part[ 1 ] );
}

void OnSourceSkeleton( int side, vr::EVRSkeletalMotionRange range, const vr::VRBoneTransform_t *in )
{
	HandState &h = g_hands[ side ];
	vr::VRBoneTransform_t bones[ kBoneCount ];
	memcpy( bones, in, sizeof( bones ) );
	const double now = NowSeconds();
	g_alignment.OnSkeleton( side, now );
	g_debug.RecordSkeleton( 'S', side, range, in, now );
	const SkeletonTuning tuning = CurrentTuning();

	bool runGestures;
	bool sendsUnclamped;
	bool logBendsNow = false;
	bool logNow = false;
	bool firstSkeleton = false;
	std::string events;
	GestureOutput out;
	GestureDebug dbg;
	HandDevice *device;
	bool haveTapPoints = false;
	HandPoints tapPoints;
	{
		std::lock_guard<std::mutex> lock( h.mutex );
		h.haveSkeleton = true;
		h.skeletonTime = now;
		h.staleSkeletonReset = false;
		h.sourceSeen = true;
		h.skeletonFrames++;
		if ( !h.loggedFirstSkeleton )
		{
			h.loggedFirstSkeleton = true;
			firstSkeleton = true;
		}

		// Sources send both motion ranges each frame; recognise gestures once, on the unclamped one.
		if ( range == vr::VRSkeletalMotionRange_WithoutController )
			h.withoutControllerTime = now;
		sendsUnclamped = h.withoutControllerTime >= 0 && now - h.withoutControllerTime < 0.25;
		runGestures = !sendsUnclamped || range == vr::VRSkeletalMotionRange_WithoutController;
		if ( runGestures && tuning.logBends && now - h.lastBendLog > 1.0 )
		{
			h.lastBendLog = now;
			logBendsNow = true;
		}
		if ( runGestures )
		{
			bool posValid = h.havePose && h.srcPose.poseIsValid;
			Vec3 handPos = posValid ? WorldPosition( h.srcPose ) : Vec3{};
			ViewFrame view = CurrentView();
			h.recognizer.Update( bones, g_settings.gesture, handPos, posValid, view.right, view.forward );
			out = h.recognizer.Output();
			dbg = h.recognizer.Debug();
			if ( posValid && ( g_settings.palmTapEnable || g_settings.calibrationGesture ) )
			{
				Vec3 joints[ kBoneCount ];
				ComputeJointPositions( bones, joints );
				tapPoints.tip = DeviceToWorld( h.srcPose, joints[ 10 ] );                         // index tip
				tapPoints.palm = DeviceToWorld( h.srcPose, ( joints[ 1 ] + joints[ 12 ] ) * 0.5 ); // wrist .. middle knuckle
				tapPoints.knuckle = DeviceToWorld( h.srcPose, joints[ kKnuckle ] );
				for ( int k = 0; k < 6; k++ )
					tapPoints.match[ k ] = DeviceToWorld( h.srcPose, joints[ kTogetherJoints[ k ] ] );
				tapPoints.open = dbg.pinch == kFingerNone && !dbg.fist;
				tapPoints.indexFree = tapPoints.open && dbg.curl[ kFingerIndex ] < 0.5f;
				haveTapPoints = true;
			}
			if ( g_settings.debugLog )
			{
				events = DescribeChanges( side, h, dbg, out );
				if ( now - h.lastDebugLog > 1.0 )
				{
					h.lastDebugLog = now;
					logNow = true;
				}
			}
		}
		device = h.device;
	}

	TwoHandResult twoHand;
	if ( haveTapPoints )
		twoHand = UpdateTwoHandGestures( side, tapPoints, now );
	const bool tapped = twoHand.tapped;
	if ( !twoHand.note.empty() )
		Log( "%s", twoHand.note.c_str() );
	if ( twoHand.calibrate )
		g_alignment.Calibrate();

	// There is no physical controller in the hand, so by default the unclamped data goes out for
	// both motion ranges (games asking for "with controller" would otherwise get a hand that can
	// only half close).
	bool postSkeleton = true;
	bool bothRanges = false;
	if ( tuning.fullRange && sendsUnclamped )
	{
		postSkeleton = range == vr::VRSkeletalMotionRange_WithoutController;
		bothRanges = true;
	}
	vr::VRBoneTransform_t sent[ kBoneCount ];
	if ( postSkeleton )
	{
		if ( tuning.mode == 1 )
			RetargetSkeleton( side, in, sent, tuning );
		else
			memcpy( sent, in, sizeof( sent ) );
		g_debug.RecordSkeleton( 'O', side, bothRanges ? -1 : range, sent, now );
		if ( logBendsNow )
			LogBends( side, in, sent );
	}

	if ( device && device->IsActive() )
	{
		if ( postSkeleton )
		{
			ApplySkeletonOffset( sent, side );
			if ( bothRanges )
			{
				device->PostSkeleton( vr::VRSkeletalMotionRange_WithoutController, sent );
				device->PostSkeleton( vr::VRSkeletalMotionRange_WithController, sent );
			}
			else
			{
				device->PostSkeleton( range, sent );
			}
		}
		if ( runGestures )
			device->PostInputs( out );
		if ( tapped )
		{
			device->PostSystem( true );
			g_systemPulseUntil[ side ] = now + 0.15; // RunFrame releases it
		}
	}
	if ( tapped )
		Log( "%s index tapped the %s palm -> system button", SideName( side ), SideName( 1 - side ) );
	if ( firstSkeleton )
		Log( "Receiving %s hand skeleton data", SideName( side ) );
	if ( !events.empty() )
		Log( "%s", events.c_str() );
	if ( logNow )
		LogGestures( side, dbg, out );
}

// Fallback for a source whose skeleton component was created before our hooks were in place.
int AdoptSkeletonHandle( vr::VRInputComponentHandle_t handle, const vr::VRBoneTransform_t *bones )
{
	if ( handle == g_ourSkeleton[ 0 ].load() || handle == g_ourSkeleton[ 1 ].load() )
		return -1;
	// The wrist sits on the hand's own side of the skeleton root.
	int side = bones[ 1 ].position.v[ 0 ] < 0 ? 0 : 1;
	HandState &h = g_hands[ side ];
	if ( h.sourceIndex.load() == kInvalidIndex )
		return -1;
	vr::VRInputComponentHandle_t expected = kInvalidHandle;
	if ( !h.srcSkeleton.compare_exchange_strong( expected, handle ) )
		return -1;
	Log( "Adopted skeleton component %llu as the %s source (created before HandEmu loaded)", (unsigned long long)handle, SideName( side ) );
	return side;
}

// ------------------------------------------------------------------------------------------------
// Detours
// ------------------------------------------------------------------------------------------------

void Detour_PoseUpdated( void *self, uint32_t device, const vr::DriverPose_t &pose, uint32_t size )
{
	g_orig.poseUpdated( self, device, pose, size );
	if ( !g_active.load( std::memory_order_relaxed ) )
		return;
	switch ( ClassifyDevice( device ) )
	{
	case kKindSourceLeft:
		OnSourcePose( 0, pose, size );
		break;
	case kKindSourceRight:
		OnSourcePose( 1, pose, size );
		break;
	case kKindHmd:
		OnHmdPose( pose );
		OnHmdForAlignment( pose );
		break;
	default:
		break;
	}
}

vr::EVRInputError Detour_CreateBoolean( void *self, vr::PropertyContainerHandle_t container, const char *name, vr::VRInputComponentHandle_t *handle )
{
	vr::EVRInputError result = g_orig.createBoolean( self, container, name, handle );
	if ( result == vr::VRInputError_None && handle && name && g_active.load() && strcmp( name, "/input/system/click" ) == 0 )
	{
		RecordComponent( container, *handle, name );
		int side = SourceSideOfContainer( container );
		if ( side >= 0 )
			ClaimSourceComponent( g_hands[ side ].srcSystem, *handle, "system button", side );
	}
	return result;
}

vr::EVRInputError Detour_UpdateBoolean( void *self, vr::VRInputComponentHandle_t handle, bool value, double timeOffset )
{
	vr::EVRInputError result = g_orig.updateBoolean( self, handle, value, timeOffset );
	if ( g_active.load( std::memory_order_relaxed ) && g_settings.forwardSystemButton && handle != kInvalidHandle )
	{
		for ( int side = 0; side < 2; side++ )
		{
			if ( handle != g_hands[ side ].srcSystem.load( std::memory_order_relaxed ) )
				continue;
			HandDevice *device;
			{
				std::lock_guard<std::mutex> lock( g_hands[ side ].mutex );
				device = g_hands[ side ].device;
			}
			if ( device )
				device->PostSystem( value );
		}
	}
	return result;
}

vr::EVRInputError Detour_CreateSkeleton( void *self, vr::PropertyContainerHandle_t container, const char *name, const char *skeletonPath,
	const char *basePosePath, vr::EVRSkeletalTrackingLevel level, const vr::VRBoneTransform_t *gripLimit, uint32_t gripLimitCount,
	vr::VRInputComponentHandle_t *handle )
{
	vr::EVRInputError result = g_orig.createSkeleton( self, container, name, skeletonPath, basePosePath, level, gripLimit, gripLimitCount, handle );
	if ( result == vr::VRInputError_None && handle && name && g_active.load() )
	{
		RecordComponent( container, *handle, name );
		int side = SourceSideOfContainer( container );
		if ( side >= 0 )
		{
			if ( strstr( name, "left" ) )
				side = 0;
			else if ( strstr( name, "right" ) )
				side = 1;
			ClaimSourceComponent( g_hands[ side ].srcSkeleton, *handle, "skeleton", side );
		}
	}
	return result;
}

vr::EVRInputError Detour_UpdateSkeleton( void *self, vr::VRInputComponentHandle_t handle, vr::EVRSkeletalMotionRange range,
	const vr::VRBoneTransform_t *bones, uint32_t count )
{
	vr::EVRInputError result = g_orig.updateSkeleton( self, handle, range, bones, count );
	if ( !g_active.load( std::memory_order_relaxed ) || !bones || count < kBoneCount || handle == kInvalidHandle )
		return result;

	int side = -1;
	if ( handle == g_hands[ 0 ].srcSkeleton.load( std::memory_order_relaxed ) )
		side = 0;
	else if ( handle == g_hands[ 1 ].srcSkeleton.load( std::memory_order_relaxed ) )
		side = 1;
	else if ( g_hands[ 0 ].srcSkeleton.load() == kInvalidHandle || g_hands[ 1 ].srcSkeleton.load() == kInvalidHandle )
		side = AdoptSkeletonHandle( handle, bones );

	if ( side >= 0 )
		OnSourceSkeleton( side, range, bones );
	return result;
}

void PinModule()
{
	HMODULE self = nullptr;
	GetModuleHandleExW( GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
		reinterpret_cast<LPCWSTR>( &PinModule ), &self );
}

// <driver root>\debug, where the DLL is installed as <driver root>\bin\win64\driver_handemu.dll.
std::wstring DebugFolderPath()
{
	HMODULE self = nullptr;
	GetModuleHandleExW( GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
		reinterpret_cast<LPCWSTR>( &DebugFolderPath ), &self );
	wchar_t path[ MAX_PATH ] = {};
	DWORD n = GetModuleFileNameW( self, path, MAX_PATH );
	if ( n == 0 || n >= MAX_PATH )
		return std::wstring();
	std::wstring dir( path, n );
	size_t slash = dir.find_last_of( L"\\/" );
	if ( slash == std::wstring::npos )
		return std::wstring();
	dir.erase( slash );
	const std::wstring bin = L"\\bin\\win64";
	if ( dir.size() > bin.size() && _wcsicmp( dir.c_str() + dir.size() - bin.size(), bin.c_str() ) == 0 )
		dir.erase( dir.size() - bin.size() );
	return dir + L"\\debug";
}

bool InstallHooks()
{
	if ( g_hooksInstalled )
		return MH_EnableHook( MH_ALL_HOOKS ) == MH_OK;

	g_host = vr::VRServerDriverHost();
	g_input = vr::VRDriverInput();
	if ( !g_host || !g_input )
	{
		Log( "Missing driver interfaces (host=%p input=%p)", g_host, g_input );
		return false;
	}
	void **hostVtable = *reinterpret_cast<void ***>( g_host );
	void **inputVtable = *reinterpret_cast<void ***>( g_input );

	MH_STATUS status = MH_Initialize();
	if ( status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED )
	{
		Log( "MH_Initialize failed: %s", MH_StatusToString( status ) );
		return false;
	}

	struct HookSpec
	{
		void *target;
		void *detour;
		void **original;
		const char *name;
	};
	HookSpec specs[] = {
		{ hostVtable[ kSlotTrackedDevicePoseUpdated ], (void *)&Detour_PoseUpdated, (void **)&g_orig.poseUpdated, "TrackedDevicePoseUpdated" },
		{ inputVtable[ kSlotCreateBooleanComponent ], (void *)&Detour_CreateBoolean, (void **)&g_orig.createBoolean, "CreateBooleanComponent" },
		{ inputVtable[ kSlotUpdateBooleanComponent ], (void *)&Detour_UpdateBoolean, (void **)&g_orig.updateBoolean, "UpdateBooleanComponent" },
		{ inputVtable[ kSlotCreateSkeletonComponent ], (void *)&Detour_CreateSkeleton, (void **)&g_orig.createSkeleton, "CreateSkeletonComponent" },
		{ inputVtable[ kSlotUpdateSkeletonComponent ], (void *)&Detour_UpdateSkeleton, (void **)&g_orig.updateSkeleton, "UpdateSkeletonComponent" },
	};

	int created = 0;
	for ( HookSpec &spec : specs )
	{
		status = MH_CreateHook( spec.target, spec.detour, spec.original );
		if ( status != MH_OK )
		{
			Log( "MH_CreateHook(%s) failed: %s", spec.name, MH_StatusToString( status ) );
			for ( int i = 0; i < created; i++ )
				MH_RemoveHook( specs[ i ].target );
			g_orig = Originals{};
			return false;
		}
		created++;
	}

	status = MH_EnableHook( MH_ALL_HOOKS );
	if ( status != MH_OK )
	{
		Log( "MH_EnableHook failed: %s", MH_StatusToString( status ) );
		for ( HookSpec &spec : specs )
			MH_RemoveHook( spec.target );
		g_orig = Originals{};
		return false;
	}
	g_hooksInstalled = true;
	return true;
}

// ------------------------------------------------------------------------------------------------
// Provider
// ------------------------------------------------------------------------------------------------

class DeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	vr::EVRInitError Init( vr::IVRDriverContext *context ) override
	{
		VR_INIT_SERVER_DRIVER_CONTEXT( context );
		LoadSettings();
		Log( "HandEmu %s starting; hand-tracking source type '%s'", kVersion, g_settings.sourceControllerType.c_str() );

		// The detours live in this DLL; never let it unload while vrserver might still call them.
		PinModule();
		if ( !InstallHooks() )
		{
			Log( "Could not hook vrserver; hand emulation is disabled" );
			return vr::VRInitError_None;
		}
		g_active = true;
		Log( "Hooks installed; waiting for hand-tracking devices" );
		const SkeletonTuning &k = g_settings.skeleton;
		Log( "Skeleton: %s, %s, finger gain %.2f, thumb gain %.2f", k.mode == 1 ? "rebuilt in SteamVR conventions" : "forwarded unchanged",
			k.fullRange ? "full range for both motion ranges" : "motion ranges as sent", k.fingerGain, k.thumbGain );
		std::wstring debugPath = DebugFolderPath();
		if ( !debugPath.empty() )
		{
			g_debug.Init( debugPath );
			Log( "Debug folder: %s (request.txt = capture, live.txt = live skeleton tuning)", g_debug.PathUtf8().c_str() );
		}
		return vr::VRInitError_None;
	}

	void Cleanup() override
	{
		g_active = false;
		if ( g_hooksInstalled )
			MH_DisableHook( MH_ALL_HOOKS );
		for ( HandState &h : g_hands )
		{
			// Devices are deliberately leaked: a detour on another thread may still hold a pointer.
			std::lock_guard<std::mutex> lock( h.mutex );
			h.device = nullptr;
		}
		VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	}

	const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

	void RunFrame() override
	{
		vr::VREvent_t event;
		while ( vr::VRServerDriverHost()->PollNextEvent( &event, sizeof( event ) ) )
		{
		}
		if ( !g_active )
			return;
		for ( int side = 0; side < 2; side++ )
		{
			AddDeviceIfNeeded( side );
			CheckStale( side );
			ReleaseSystemPulse( side );
		}
		if ( g_settings.debugLog )
			LogStatus();
		PollDebugFolder();
	}

	bool ShouldBlockStandbyMode() override { return false; }
	void EnterStandby() override {}
	void LeaveStandby() override {}

private:
	void AddDeviceIfNeeded( int side )
	{
		HandState &h = g_hands[ side ];
		HandDevice *device;
		{
			std::lock_guard<std::mutex> lock( h.mutex );
			if ( !h.sourceSeen || m_added[ side ] )
				return;
			m_added[ side ] = true;
			device = new HandDevice( side );
			h.device = device;
		}
		bool ok = vr::VRServerDriverHost()->TrackedDeviceAdded( device->Serial(), vr::TrackedDeviceClass_Controller, device );
		Log( "Adding emulated %s controller %s: %s", SideName( side ), device->Serial(), ok ? "ok" : "FAILED" );
	}

	void CheckStale( int side )
	{
		HandState &h = g_hands[ side ];
		const double now = NowSeconds();
		bool resetInputs = false;
		bool invalidatePose = false;
		vr::DriverPose_t pose;
		HandDevice *device;
		{
			std::lock_guard<std::mutex> lock( h.mutex );
			device = h.device;
			// Tracking lost: let go of everything rather than leave a button stuck down.
			if ( h.haveSkeleton && !h.staleSkeletonReset && now - h.skeletonTime > 0.25 )
			{
				h.staleSkeletonReset = true;
				h.recognizer.Reset();
				resetInputs = true;
			}
			if ( h.havePose && !h.stalePoseSent && now - h.poseTime > 1.0 )
			{
				h.stalePoseSent = true;
				pose = h.srcPose;
				pose.poseIsValid = false;
				pose.result = vr::TrackingResult_Running_OutOfRange;
				invalidatePose = true;
			}
		}
		if ( !device || !device->IsActive() )
			return;
		if ( resetInputs )
			device->PostInputs( GestureOutput{} );
		if ( invalidatePose )
		{
			ApplyPoseOffset( pose, side );
			device->PostPose( pose );
		}
	}

	void ReleaseSystemPulse( int side )
	{
		double until = g_systemPulseUntil[ side ].load();
		if ( until <= 0 || NowSeconds() < until )
			return;
		g_systemPulseUntil[ side ] = 0;
		HandDevice *device;
		{
			std::lock_guard<std::mutex> lock( g_hands[ side ].mutex );
			device = g_hands[ side ].device;
		}
		if ( device )
			device->PostSystem( false );
	}

	// Every 5 s: is data arriving from the source, and are the emulated controllers up?
	void LogStatus()
	{
		const double now = NowSeconds();
		if ( m_lastStatus == 0 )
		{
			m_lastStatus = now;
			return;
		}
		const double elapsed = now - m_lastStatus;
		if ( elapsed < 5.0 )
			return;
		m_lastStatus = now;

		char part[ 2 ][ 160 ];
		for ( int side = 0; side < 2; side++ )
		{
			HandState &h = g_hands[ side ];
			uint32_t skeletons, poses;
			bool seen;
			HandDevice *device;
			{
				std::lock_guard<std::mutex> lock( h.mutex );
				skeletons = h.skeletonFrames;
				poses = h.poseFrames;
				h.skeletonFrames = 0;
				h.poseFrames = 0;
				seen = h.sourceSeen;
				device = h.device;
			}
			snprintf( part[ side ], sizeof( part[ side ] ), "%s source %s, skeleton updates %.0f/s, poses %.0f/s, controller %s", SideName( side ),
				seen ? "seen" : "missing", skeletons / elapsed, poses / elapsed, device && device->IsActive() ? "active" : "not active" );
		}
		Log( "status: %s | %s", part[ 0 ], part[ 1 ] );
	}

	void PollDebugFolder()
	{
		const double now = NowSeconds();
		std::string msg = g_debug.PollCapture( now );
		if ( !msg.empty() )
			Log( "%s", msg.c_str() );
		if ( now < m_nextLiveCheck )
			return;
		m_nextLiveCheck = now + 0.5;
		std::vector<std::pair<std::string, std::string>> entries;
		if ( g_debug.PollLiveFile( entries ) )
			Log( "Live tuning: %s", ApplyLiveTuning( entries ).c_str() );
	}

	bool m_added[ 2 ] = { false, false };
	double m_lastStatus = 0;
	double m_nextLiveCheck = 0;
};

DeviceProvider g_provider;

} // namespace

extern "C" __declspec( dllexport ) void *HmdDriverFactory( const char *interfaceName, int *returnCode )
{
	if ( strcmp( interfaceName, vr::IServerTrackedDeviceProvider_Version ) == 0 )
		return &g_provider;
	if ( returnCode )
		*returnCode = vr::VRInitError_Init_InterfaceNotFound;
	return nullptr;
}
