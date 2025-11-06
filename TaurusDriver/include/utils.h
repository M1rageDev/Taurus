#pragma once

#include <iostream>

#include "openvr_driver.h"

#include "protocol/TaurusMessages.pb.h"

namespace utils
{
	vr::DriverPose_t CreateZeroPose();

	vr::TrackedDevicePose_t GetHMDPose();

	void CopyHmdRelativePosition(vr::DriverPose_t* pose, const vr::HmdVector3_t hmdPosition, const messages::Position& position);
	void CopyHmdRelativePosition(vr::DriverPose_t* pose, const vr::HmdVector3_t hmdPosition, const vr::HmdVector3_t position);

	void CopyAbsolutePosition(vr::DriverPose_t* pose, const messages::Position& position);

	void CopyAbsoluteQuaternion(vr::DriverPose_t* pose, const messages::Orientation& orientation);
}
