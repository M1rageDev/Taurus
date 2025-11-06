#pragma once

#include <string>
#include <atomic>
#include "openvr_driver.h"

#include "protocol/TaurusMessages.pb.h"

#include "utils.h"
#include "logging.h"
#include "vrmath.h"

class TrackerDevice : public vr::ITrackedDeviceServerDriver {
	public:
		TrackerDevice(const messages::TrackerInfo& trackerInfo);

		// override functions
		vr::EVRInitError Activate(uint32_t unObjectId) override;
		void EnterStandby() override;
		void *GetComponent(const char *pchComponentNameAndVersion) override;
		void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override;
		vr::DriverPose_t GetPose() override;
		void Deactivate() override;

		// our own functions
		const std::string& GetSerialNumber();

		void SetInitialPose();
		void RefreshPose() const;
		void SetPose(vr::DriverPose_t pose);

		void RunFrame();
		void ProcessEvent(const vr::VREvent_t &vrevent);

	private:
		int trackerId;
		messages::Pose trackerPose;

		vr::TrackedDeviceIndex_t deviceIndex;

		std::string modelNumber;
		std::string serialNumber;

		vr::DriverPose_t currentPose;

		bool isActive;
};
