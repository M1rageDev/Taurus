#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <atomic>
#include <thread>
#include "openvr_driver.h"

#include "protocol/TaurusMessages.pb.h"

#include "settings.h"
#include "utils.h"
#include "logging.h"
#include "vrmath.h"

enum TaurusInputComponent
{
	tc_system_click,
	tc_move_click,

	tc_square_click,
	tc_cross_click,
	tc_triangle_click,
	tc_circle_click,

	tc_start_click,
	tc_select_click,

	tc_virt_joystick_click,
	tc_virt_joystick_x,
	tc_virt_joystick_y,

	tc_trigger_value,

	tc_haptic,

	tc_MAX
};

class ControllerDevice : public vr::ITrackedDeviceServerDriver {
	public:
		ControllerDevice(vr::ETrackedControllerRole role);

		// override functions
		vr::EVRInitError Activate(uint32_t unObjectId) override;
		void EnterStandby() override;
		void *GetComponent(const char *pchComponentNameAndVersion) override;
		void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override;
		vr::DriverPose_t GetPose() override;
		void Deactivate() override;

		// our own functions
		const std::string& GetSerialNumber();
		const bool MatchesSerialNumber(std::string x);
		void SetInitialPose();

		void SetInputByMsgEvent(const messages::InputEvent& event);
		void UpdateVirtualJoystick();

		void ProcessPoseMessage(const messages::PoseMessage& poseMsg);
		void ProcessInputMessage(const messages::InputMessage& inputMsg);
		void ProcessStatusMessage(const messages::StatusMessage& stateMsg);

		void RefreshPose() const;
		void SetPose(vr::DriverPose_t pose);

		void RunFrame();
		void ProcessEvent(const vr::VREvent_t &vrevent);

	private:
		vr::TrackedDeviceIndex_t controllerIndex;
		vr::ETrackedControllerRole controllerRole;

		std::string modelNumber;
		std::string serialNumber;

		std::array<float, tc_MAX> inputValues;
		std::array<vr::VRInputComponentHandle_t, tc_MAX> inputHandles;

		vr::DriverPose_t currentPose;

		int currentBatteryPercent;
		bool isCharging;

		bool isActive;
};
