#pragma once

#include "openvr_driver.h"

#include "protocol/TaurusMessages.pb.h"

#include "controller_device.h"
#include "tracker_device.h"
#include "settings.h"
#include "communication.h"
#include "logging.h"

class TaurusDeviceDriver : public vr::IServerTrackedDeviceProvider {
	public:
		// singleton instance
		static TaurusDeviceDriver* GetInstance();

		// override functions
		vr::EVRInitError Init(vr::IVRDriverContext *context) override;
		const char *const *GetInterfaceVersions() override;

		void RunFrame() override;
		bool ShouldBlockStandbyMode() override;
		void EnterStandby() override;
		void LeaveStandby() override;
		void Cleanup() override;

		// our own functions
		void RequestTrackerList();
		void RegisterTracker(const messages::TrackerInfo& trackerInfo);

		void SendDriverMessage(const messages::DriverMessage& msg);
		void MessageThread();

	private:
		static TaurusDeviceDriver* instance;

		std::unique_ptr<ControllerDevice> leftController;
		std::unique_ptr<ControllerDevice> rightController;

		std::vector<TrackerDevice*> trackers;

		std::atomic<bool> isThreadActive;
		std::thread messageThread;

		int udpPort;
		int udpSendPort;
		SOCKET sock_;
		SOCKET sendSock_;

		bool waitingForRequestAnswer;
		float waitingTime;
};