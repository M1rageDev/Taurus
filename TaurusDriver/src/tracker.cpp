#include "tracker.h"

TrackerDevice::TrackerDevice(const messages::TrackerInfo& trackerInfo) {
	isActive = false;
	trackerId = trackerInfo.id();
	trackerPose = trackerInfo.pose();

	// load settings
	modelNumber = "TaurusOne-Tracker";
	serialNumber = std::format("taurus_tracker_{}", trackerId);
}

vr::EVRInitError TrackerDevice::Activate(uint32_t unObjectId) {
	deviceIndex = unObjectId;

	vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(deviceIndex);

	// Basic props
	vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, modelNumber.c_str());
	vr::VRProperties()->SetStringProperty(container, vr::Prop_SerialNumber_String, serialNumber.c_str());
	vr::VRProperties()->SetStringProperty(container, vr::Prop_ModeLabel_String, serialNumber.c_str());
	vr::VRProperties()->SetInt32Property(container, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_TrackingReference);

	// TODO: send these alongside pose, see
	// https://github.com/Timocop/PSMoveServiceEx/blob/1afc1d17d0ef6d958cb33e89f33c6ac97c7bb283/src/psmoveconfigtool/AppStage_DistortionCalibration.cpp#L616
	vr::VRProperties()->SetFloatProperty(container, vr::Prop_FieldOfViewLeftDegrees_Float, 62.61f / 2.f);
	vr::VRProperties()->SetFloatProperty(container, vr::Prop_FieldOfViewRightDegrees_Float, 62.61f / 2.f);
	vr::VRProperties()->SetFloatProperty(container, vr::Prop_FieldOfViewTopDegrees_Float, 49.07f / 2.f);
	vr::VRProperties()->SetFloatProperty(container, vr::Prop_FieldOfViewBottomDegrees_Float, 49.07f / 2.f);
	vr::VRProperties()->SetFloatProperty(container, vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
	vr::VRProperties()->SetFloatProperty(container, vr::Prop_TrackingRangeMaximumMeters_Float, 3.f);
	
	// TODO: create a model
	vr::VRProperties()->SetStringProperty(container, vr::Prop_RenderModelName_String, "generic_tracker");

	// set pose
	currentPose = utils::CreateZeroPose();
	SetInitialPose();

	// Activate
	isActive = true;
	
	return vr::VRInitError_None;
}

void *TrackerDevice::GetComponent(const char *pchComponentNameAndVersion) {
	return nullptr;
}

void TrackerDevice::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) {
	if (unResponseBufferSize >= 1)
		pchResponseBuffer[ 0 ] = 0;
}

// ideally this should be never called
vr::DriverPose_t TrackerDevice::GetPose() {
	return currentPose;
}

void TrackerDevice::EnterStandby() {
	DriverLog("tracker %d has been put on standby", trackerId);
}

void TrackerDevice::Deactivate() {
	isActive = false;
	deviceIndex = vr::k_unTrackedDeviceIndexInvalid;
}

const std::string& TrackerDevice::GetSerialNumber() {
	return serialNumber;
}

void TrackerDevice::SetInitialPose() {
	// retrieve the msg pose
	const messages::Pose& msgPose = trackerPose;
	const messages::Position& msgPosition = msgPose.position();
	const messages::Orientation& msgOrientation = msgPose.orientation();

	// Retrieve the HMD's pose
	vr::TrackedDevicePose_t hmdPose = utils::GetHMDPose();
	const vr::HmdVector3_t hmdPosition = HmdVector3_From34Matrix(hmdPose.mDeviceToAbsoluteTracking);

	// copy msg data to our pose
	utils::CopyHmdRelativePosition(&currentPose, hmdPosition, msgPosition);
	utils::CopyAbsoluteQuaternion(&currentPose, msgOrientation);

	RefreshPose();
}

void TrackerDevice::RefreshPose() const {
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(deviceIndex, currentPose, sizeof(vr::DriverPose_t));
}

void TrackerDevice::SetPose(vr::DriverPose_t pose) {
	currentPose = pose;
	RefreshPose();
}

void TrackerDevice::RunFrame() {
	SetInitialPose();
}

void TrackerDevice::ProcessEvent(const vr::VREvent_t &vrevent) {
	// do nothing, for now
}
