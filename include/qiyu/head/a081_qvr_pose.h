// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <qiyu/head/a081_tracking_wire.h>
#include "xrt/xrt_device.h"
#include <math.h>
#include <string.h>

// Explicit experimental SDK Portrait profile. The raw QVR pose is converted
// to a device pose, not copied from the SXR consumer's view quaternion.
// QVR: x up, y left, z toward the user. XR: x right, y up, z toward the user.
// C = Rz(+90 degrees), p_xr = C p_raw, q_xr = C q_raw C^-1.
// Scale 1 follows the original metric calibration and SDK arithmetic. Physical
// accuracy and the tracker-to-eye origin remain uncalibrated on this device.
static inline bool
a081_qvr_pose(const struct a081_tracking_packet *p, uint64_t now, struct xrt_space_relation *out)
{
	memset(out, 0, sizeof(*out));
	out->pose.orientation.w = 1.0f;
	if (p->flags != A081_TRACKING_KNOWN_FLAGS || p->pose_read_status != A081_POSE_COPIED ||
	    p->pose_quality != 1.0f || p->tracking_state != 4 || p->pose_index >= 80 ||
	    p->pose_boottime_ns != p->gyro_boottime_ns || !p->last_camera_boottime_ns ||
	    p->last_camera_boottime_ns > p->gyro_boottime_ns ||
	    p->gyro_boottime_ns - p->last_camera_boottime_ns >= A081_TRACKING_MAX_AGE_NS ||
	    !p->sample_monotonic_ns || p->sample_monotonic_ns > p->send_monotonic_ns ||
	    p->send_monotonic_ns > now || now - p->sample_monotonic_ns >= A081_TRACKING_MAX_AGE_NS ||
	    p->clock_bracket_ns > A081_TRACKING_MAX_BRACKET_NS) return false;
	float norm = 0.0f;
	for (unsigned i = 0; i < 4; ++i) {
		if (!isfinite(p->rotation_xyzw[i])) return false;
		norm += p->rotation_xyzw[i] * p->rotation_xyzw[i];
	}
	for (unsigned i = 0; i < 3; ++i) {
		if (!isfinite(p->translation[i]) || !isfinite(p->accel[i]) || !isfinite(p->gyro[i])) return false;
	}
	if (!isfinite(norm) || fabsf(norm - 1.0f) > 0.01f) return false;
	out->pose.orientation = (struct xrt_quat){-p->rotation_xyzw[1], p->rotation_xyzw[0],
	                                        p->rotation_xyzw[2], p->rotation_xyzw[3]};
	out->pose.position = (struct xrt_vec3){-p->translation[1], p->translation[0], p->translation[2]};
	out->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                      XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	// No velocity flags, prediction, fallback pose, head model or guessed eye offset.
	return true;
}
