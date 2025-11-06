#include "controller_device.h"

#include "driver_main.h"

ControllerDevice::ControllerDevice(vr::ETrackedControllerRole role) {
	isActive = false;
	controllerRole = role;

	// load settings
	modelNumber = settings::GetModelNumber();
	serialNumber = settings::GetSerialNumber(controllerRole);

	DriverLog("Taurus Controller Model Number: %s", modelNumber.c_str());
	DriverLog("Taurus Controller Serial Number: %s", serialNumber.c_str());
}

vr::EVRInitError ControllerDevice::Activate(uint32_t unObjectId) {
	controllerIndex = unObjectId;

	vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(controllerIndex);

	// Basic props
	vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, modelNumber.c_str());
	vr::VRProperties()->SetStringProperty(container, vr::Prop_SerialNumber_String, serialNumber.c_str());
	vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, controllerRole);
	vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);

	vr::VRProperties()->SetStringProperty(container, vr::Prop_RenderModelName_String, "{taurus}psmove");
	vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String, "{taurus}/input/tauruscontroller_profile.json");

	// Input stuff
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/system/click", &inputHandles[tc_system_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/move/click", &inputHandles[tc_move_click]);

	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/square/click", &inputHandles[tc_square_click ]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/cross/click", &inputHandles[tc_cross_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/triangle/click", &inputHandles[tc_triangle_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/circle/click", &inputHandles[tc_circle_click]);

	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/start/click", &inputHandles[tc_start_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/select/click", &inputHandles[tc_select_click]);

	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/virtual_joystick/click", &inputHandles[tc_virt_joystick_click]);
	vr::VRDriverInput()->CreateScalarComponent(container, "/input/virtual_joystick/x", &inputHandles[tc_virt_joystick_x], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
	vr::VRDriverInput()->CreateScalarComponent(container, "/input/virtual_joystick/y", &inputHandles[tc_virt_joystick_y], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);

	vr::VRDriverInput()->CreateScalarComponent(container, "/input/trigger/value", &inputHandles[tc_trigger_value ], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

	vr::VRDriverInput()->CreateHapticComponent(container, "/output/haptic", &inputHandles[tc_haptic ]);

	// Set initial pose
	SetInitialPose();

	// Activate
	isActive = true;
	
	return vr::VRInitError_None;
}

void *ControllerDevice::GetComponent(const char *pchComponentNameAndVersion) {
	return nullptr;
}

void ControllerDevice::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) {
	if (unResponseBufferSize >= 1)
		pchResponseBuffer[ 0 ] = 0;
}

// ideally this should be never called
vr::DriverPose_t ControllerDevice::GetPose() {
	return currentPose;
}

void ControllerDevice::EnterStandby() {
	DriverLog("%s hand has been put on standby", controllerRole == vr::TrackedControllerRole_LeftHand ? "Left" : "Right");
}

void ControllerDevice::Deactivate() {
	isActive = false;

	// unassign our controller index (we don't want to be calling vrserver anymore after Deactivate() has been called
	controllerIndex = vr::k_unTrackedDeviceIndexInvalid;
}

const std::string& ControllerDevice::GetSerialNumber() {
	return serialNumber;
}

const bool ControllerDevice::MatchesSerialNumber(std::string x) {
	return GetSerialNumber() == x;
}

void ControllerDevice::SetInitialPose() {
	// Initialize the pose holder struct
	vr::DriverPose_t pose = utils::CreateZeroPose();

	// Retrieve the HMD's pose
	vr::TrackedDevicePose_t hmdPose = utils::GetHMDPose();
	const vr::HmdVector3_t hmdPosition = HmdVector3_From34Matrix(hmdPose.mDeviceToAbsoluteTracking);
	const vr::HmdQuaternion_t hmdOrientation = HmdQuaternion_FromMatrix(hmdPose.mDeviceToAbsoluteTracking);

	// pitch the controller 90 degrees so the face of the controller is facing towards us
	const vr::HmdQuaternion_t offsetOrientation = HmdQuaternion_FromEulerAngles(0.f, DEG_TO_RAD(90.f), 0.f);

	// Set the pose orientation to the hmd orientation with the offset applied.
	pose.qRotation = hmdOrientation * offsetOrientation;

	const vr::HmdVector3_t offset_position = {
		controllerRole == vr::TrackedControllerRole_LeftHand ? -0.15f : 0.15f,
		0.1f,
		-0.5f,
	};

	// copy the hmd-relative position with the offset applied
	utils::CopyHmdRelativePosition(&pose, hmdPosition, (offset_position * hmdOrientation));

	// Update the vr server
	SetPose(pose);
}

void ControllerDevice::SetInputByMsgEvent(const messages::InputEvent& event) {
	messages::InputComponent component = event.component();
	float value = event.value();
	bool isClick = value > 0.5f;

	// very ugly switch case but gotta do what you gotta do
	switch (component) {
	case messages::SYSTEM:
		inputValues[tc_system_click] = value;
		vr::VRDriverInput()->UpdateBooleanComponent(inputHandles[tc_system_click], isClick, 0);
		break;
	case messages::MOVE:
		inputValues[tc_move_click] = value;
		vr::VRDriverInput()->UpdateBooleanComponent(inputHandles[tc_move_click], isClick, 0);
		break;
	case messages::SQUARE:
		inputValues[tc_square_click] = value;
		vr::VRDriverInput()->UpdateBooleanComponent(inputHandles[tc_square_click], isClick, 0);
		break;
	case messages::CROSS:
		inputValues[tc_cross_click] = value;
		vr::VRDriverInput()->UpdateBooleanComponent(inputHandles[tc_cross_click], isClick, 0);
		break;
	case messages::TRIANGLE:
		inputValues[tc_triangle_click] = value;
		vr::VRDriverInput()->UpdateBooleanComponent(inputHandles[tc_triangle_click], isClick, 0);
		break;
	case messages::CIRCLE:
		inputValues[tc_circle_click] = value;
		vr::VRDriverInput()->UpdateBooleanComponent(inputHandles[tc_circle_click], isClick, 0);
		break;
	case messages::START:
		inputValues[tc_start_click] = value;
		vr::VRDriverInput()->UpdateBooleanComponent(inputHandles[tc_start_click], isClick, 0);
		break;
	case messages::SELECT:
		inputValues[tc_select_click] = value;
		vr::VRDriverInput()->UpdateBooleanComponent(inputHandles[tc_select_click], isClick, 0);
		break;
	case messages::TRIGGER:
		inputValues[tc_trigger_value] = value;
		vr::VRDriverInput()->UpdateScalarComponent(inputHandles[tc_trigger_value], value, 0);
		break;
	default:
		DriverLog("Unknown input component in input message: %d", component);
		break;
	}
}

void ControllerDevice::UpdateVirtualJoystick() {
	float x = 0.f;
	float y = 0.f;

	if (inputValues[tc_triangle_click] > 0.f) {
		y = 1.f;
	}

	vr::VRDriverInput()->UpdateScalarComponent(inputHandles[tc_virt_joystick_x], x, 0);
	vr::VRDriverInput()->UpdateScalarComponent(inputHandles[tc_virt_joystick_y], y, 0);
}

void ControllerDevice::ProcessPoseMessage(const messages::PoseMessage& poseMsg) {
	if (!isActive) return;

	// Retrieve the HMD's pose
	vr::TrackedDevicePose_t hmdPose = utils::GetHMDPose();
	const vr::HmdVector3_t hmdPosition = HmdVector3_From34Matrix(hmdPose.mDeviceToAbsoluteTracking);
	const vr::HmdQuaternion_t hmdOrientation = HmdQuaternion_FromMatrix(hmdPose.mDeviceToAbsoluteTracking);

	// Retrieve the pose from the message
	const messages::Pose& msgPose = poseMsg.pose();
	const messages::Position& msgPosition = msgPose.position();
	const messages::Orientation& msgOrientation = msgPose.orientation();

	// copy msg data to our pose
	utils::CopyHmdRelativePosition(&currentPose, hmdPosition, msgPosition);
	utils::CopyAbsoluteQuaternion(&currentPose, msgOrientation);

	// Update the vr server
	RefreshPose();
}

void ControllerDevice::ProcessInputMessage(const messages::InputMessage& inputMsg) {
	if (!isActive) return;

	for (int i = 0; i < inputMsg.events_size(); i++) {
		const messages::InputEvent& event = inputMsg.events(i);

		SetInputByMsgEvent(event);
	}

	UpdateVirtualJoystick();
}

void ControllerDevice::ProcessStatusMessage(const messages::StatusMessage& stateMsg) {
	if (!isActive) return;

	const messages::ControllerStatus& state = stateMsg.status();
	
	// battery
	currentBatteryPercent = state.battery_percent();
	isCharging = state.is_charging();
	float battery01 = static_cast<float>(currentBatteryPercent) / 100.f;

	vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(controllerIndex);
	vr::VRProperties()->SetFloatProperty(container, vr::Prop_DeviceBatteryPercentage_Float, battery01);
	vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceIsCharging_Bool, isCharging);

	// device connection
	if (state.is_connected()) {
		currentPose.deviceIsConnected = true;
		currentPose.poseIsValid = true;
	}
	else {
		currentPose.deviceIsConnected = false;
		currentPose.poseIsValid = false;
	}
}

void ControllerDevice::RefreshPose() const {
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(controllerIndex, currentPose, sizeof(vr::DriverPose_t));
}

void ControllerDevice::SetPose(vr::DriverPose_t pose) {
	currentPose = pose;
	RefreshPose();
}

void ControllerDevice::RunFrame() {
	// do nothing, for now
}

void ControllerDevice::ProcessEvent(const vr::VREvent_t &vrevent) {
	switch (vrevent.eventType)
	{
		case vr::VREvent_Input_HapticVibration:
		{
			// check if this event is meant for us
			if (vrevent.data.hapticVibration.componentHandle == inputHandles[tc_haptic])
			{
				float duration = vrevent.data.hapticVibration.fDurationSeconds;
				float frequency = vrevent.data.hapticVibration.fFrequency;
				float amplitude = std::clamp(vrevent.data.hapticVibration.fAmplitude, 0.f, 1.f);

				if ((frequency < 0.f) || (amplitude < 0.f)) {
					return;
				}

				// serialize msg and send to Taurus
				messages::HapticMessage hapticMsg;
				messages::HapticEvent* msgEvent = hapticMsg.mutable_event();
				msgEvent->set_duration(duration);
				msgEvent->set_frequency(frequency);
				msgEvent->set_amplitude(amplitude);

				messages::DriverMessage msg;
				msg.set_serial(serialNumber);
				msg.mutable_haptic_message()->CopyFrom(hapticMsg);

				TaurusDeviceDriver::GetInstance()->SendDriverMessage(msg);
			}
			break;
		}
		default:
			break;
	}
}
