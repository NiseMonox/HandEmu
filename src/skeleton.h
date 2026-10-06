// Rebuilds a source hand skeleton in SteamVR's own bone conventions.
//
// Games such as Virt-A-Mate read finger angles straight out of each bone's parent-space rotation,
// measured against SteamVR's reference hand, so they only work when every bone frame follows
// Valve's layout (bone along +X on the left hand / -X on the right, curl about +Z). The source
// driver's frames need not, so only its joint positions are trusted: each bone is re-aimed at its
// child joint with the smallest rotation away from the reference pose, which keeps the bone's
// twist from the reference and turns the hand's real joint angles into clean bend/spread angles.
#pragma once

#include <openvr_driver.h>

#include "gestures.h"
#include "vrmath.h"

namespace handemu
{

struct SkeletonTuning
{
	int mode = 1;              // 0 = forward the source skeleton unchanged, 1 = rebuild (retarget)
	bool fullRange = true;     // post the unclamped (WithoutController) data for both motion ranges
	float fingerGain = 1.0f;   // scales finger curl (bend away from straight)
	float thumbGain = 1.0f;    // same for the two outer thumb joints
	float maxBend = 115.0f;    // degrees, upper limit for a scaled joint
	bool logBends = false;     // log measured / sent joint bends once a second (diagnostics)
};

// One hand's 31 parent-space bones in, Valve-convention bones out. side: 0 = left, 1 = right.
void RetargetSkeleton( int side, const vr::VRBoneTransform_t *in, vr::VRBoneTransform_t *out, const SkeletonTuning &tuning );

// Bend (curl) angle of every finger joint in degrees, measured the way SteamVR's reference poses
// define it: rotation about the joint's +Z axis, 0 = straight. Order: thumb 2..4, index 7..9,
// middle 12..14, ring 17..19, pinky 22..24 (15 values).
void MeasureBends( const vr::VRBoneTransform_t *bones, float out[ 15 ] );

// SteamVR's reference (bind) pose for one hand, in parent space.
const vr::VRBoneTransform_t *ReferencePose( int side );

} // namespace handemu
