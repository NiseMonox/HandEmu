// Turns a 31-bone SteamVR hand skeleton into Touch-style controller inputs.
//
// Fixed mapping (the per-game binding decides what each output does):
//   thumb + index  pinch -> trigger      (touch = thumb near index)
//   thumb + middle pinch -> A (right) / X (left)
//   thumb + ring   pinch -> joystick: hold and move the hand to deflect it
//   thumb + pinky  pinch -> B (right) / Y (left)
//   fist                 -> grip
#pragma once

#include <cstdint>

#include <openvr_driver.h>

#include "vrmath.h"

namespace handemu
{

constexpr uint32_t kBoneCount = 31;

struct GestureConfig
{
	// Thumb-tip to finger-tip distances, metres. Vision Pro (via KRVR) reports 1-5 mm for a firm
	// pinch and ~20 mm for the thumb resting beside the index in a fist.
	float pinchOn = 0.012f;
	float pinchOff = 0.020f;
	float nearOn = 0.035f;  // trigger "touch"
	float nearOff = 0.045f;
	float analogFar = 0.060f; // trigger value starts rising below this distance
	float fingerMargin = 0.008f; // how much closer a non-index finger must be than the index to win

	// Curl is 0 (straight) .. 1 (fully curled); fist uses middle+ring+pinky average.
	// Hand tracking often reports the index nearly straight inside a fist, so a thumb tucked
	// near the index tip also counts; a pointing hand (index out, thumb away) does not.
	float fistOn = 0.70f;
	float fistOff = 0.55f;
	float fistIndexOn = 0.25f;
	float fistThumbNear = 0.045f;
	int fistFrames = 4; // consecutive frames before a fist registers (filters flicker)

	// Weight of the newest sample in the exponential smoothing (1 = no smoothing).
	float smoothing = 0.5f;

	bool joystickEnable = true;
	float joystickRadius = 0.06f; // hand travel for full deflection, metres
	float joystickDeadzone = 0.15f;
};

enum Finger : int
{
	kFingerNone = -1,
	kFingerIndex = 0,
	kFingerMiddle = 1,
	kFingerRing = 2,
	kFingerPinky = 3,
};

struct GestureOutput
{
	float triggerValue = 0;
	bool triggerClick = false;
	bool triggerTouch = false;

	float gripValue = 0;
	bool gripClick = false;
	bool gripTouch = false;

	bool aClick = false; // A on the right hand, X on the left
	bool aTouch = false;
	bool bClick = false; // B on the right hand, Y on the left
	bool bTouch = false;

	float joyX = 0;
	float joyY = 0;
	bool joyClick = false;
	bool joyTouch = false;
};

struct GestureDebug
{
	float dist[ 4 ] = {}; // smoothed thumb distance per finger, metres
	float curl[ 4 ] = {}; // smoothed curl per finger
	float fistScore = 0;
	int pinch = kFingerNone;
	float pinchMin = 0; // closest the current (or last) pinch got
	bool fist = false;
};

// Model-space joint positions (relative to the skeleton root) from parent-space bone transforms.
void ComputeJointPositions( const vr::VRBoneTransform_t *bones, Vec3 out[ kBoneCount ] );

class GestureRecognizer
{
public:
	// handPos: hand position in world space, used for the joystick drag.
	// viewRight / viewForward: horizontal unit vectors of the headset, so dragging is head-relative.
	void Update( const vr::VRBoneTransform_t *bones, const GestureConfig &cfg,
		const Vec3 &handPos, bool handPosValid, const Vec3 &viewRight, const Vec3 &viewForward );
	void Reset();

	const GestureOutput &Output() const { return m_out; }
	const GestureDebug &Debug() const { return m_dbg; }

private:
	bool m_hasSample = false;
	float m_dist[ 4 ] = {};
	float m_curl[ 4 ] = {};

	int m_pinch = kFingerNone;
	float m_pinchMin = 0;
	bool m_fist = false;
	int m_fistCandidateFrames = 0;
	bool m_near = false;

	bool m_joyActive = false;
	Vec3 m_joyStart;

	GestureOutput m_out;
	GestureDebug m_dbg;
};

} // namespace handemu
