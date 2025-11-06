#include "utils.h"

vr::DriverPose_t utils::CreateZeroPose() {
	vr::DriverPose_t pose = { 0 };

	// Init the quaternions to identity
	pose.qWorldFromDriverRotation.w = 1.f;
	pose.qDriverFromHeadRotation.w = 1.f;

	// Set some basic flags
	pose.willDriftInYaw = false;
	pose.shouldApplyHeadModel = false;
	pose.poseIsValid = true;
	pose.deviceIsConnected = true;
	pose.result = vr::TrackingResult_Running_OK;

	return pose;
}

vr::TrackedDevicePose_t utils::GetHMDPose() {
	vr::TrackedDevicePose_t hmdPose{};
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, &hmdPose, 1);

	return hmdPose;
}

void utils::CopyHmdRelativePosition(vr::DriverPose_t* pose, const vr::HmdVector3_t hmdPosition, const messages::Position& position) {
	pose->vecPosition[0] = hmdPosition.v[0] + position.x();
	pose->vecPosition[1] = hmdPosition.v[1] + position.y();
	pose->vecPosition[2] = hmdPosition.v[2] + position.z();
}

void utils::CopyHmdRelativePosition(vr::DriverPose_t* pose, const vr::HmdVector3_t hmdPosition, const vr::HmdVector3_t position) {
	pose->vecPosition[0] = hmdPosition.v[0] + position.v[0];
	pose->vecPosition[1] = hmdPosition.v[1] + position.v[1];
	pose->vecPosition[2] = hmdPosition.v[2] + position.v[2];
}

void utils::CopyAbsolutePosition(vr::DriverPose_t* pose, const messages::Position& position) {
	pose->vecPosition[0] = position.x();
	pose->vecPosition[1] = position.y();
	pose->vecPosition[2] = position.z();
}

void utils::CopyAbsoluteQuaternion(vr::DriverPose_t* pose, const messages::Orientation& orientation) {
	pose->qRotation = vr::HmdQuaternion_t{
		orientation.w(),
		orientation.x(),
		orientation.y(),
		orientation.z()
	};
}
