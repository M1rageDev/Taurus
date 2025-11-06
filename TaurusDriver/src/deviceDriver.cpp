#include "deviceDriver.h"

TaurusDeviceDriver* TaurusDeviceDriver::instance = nullptr;

TaurusDeviceDriver* TaurusDeviceDriver::GetInstance() {
	return instance;
}

vr::EVRInitError TaurusDeviceDriver::Init(vr::IVRDriverContext *context) {
	VR_INIT_SERVER_DRIVER_CONTEXT(context);

	// Set the singleton instance
	instance = this;

	// Init the controllers
	leftController = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_LeftHand);
	rightController = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_RightHand);

	if (!vr::VRServerDriverHost()->TrackedDeviceAdded(leftController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, leftController.get())) {
		DriverLog("Failed to create left controller device!");
		return vr::VRInitError_Driver_Unknown;
	}

	if (!vr::VRServerDriverHost()->TrackedDeviceAdded(rightController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, rightController.get())) {
		DriverLog("Failed to create right controller device!");
		return vr::VRInitError_Driver_Unknown;
	}

	// Create the receive socket
	udpPort = settings::GetUdpPort();
	udpSendPort = settings::GetUdpSendPort();
	sock_ = sock::SetupWSASocket();
	sock::BindWSASocket(sock_, udpPort);
	sendSock_ = sock::SetupWSASocket();

	// Start the pose update thread
	DriverLog("Starting message recv thread");
	isThreadActive = true;
	messageThread = std::thread(&TaurusDeviceDriver::MessageThread, this);

	// Request tracker list, and wait until finished
	DriverLog("Requesting tracker list...");
	RequestTrackerList();
	int timeout = settings::GetTrackerListTimeout();
	waitingForRequestAnswer = true;
	waitingTime = 0.f;
	while (waitingForRequestAnswer) {
		Sleep(10);
		waitingTime += 0.01f;

		// handle timeout
		if (waitingTime > timeout) {
			DriverLog("Didn't receive answer for tracker list request in %d seconds, skipping tracker registration", timeout);
			waitingForRequestAnswer = false;
		}
	}

	return vr::VRInitError_None;
}

const char *const *TaurusDeviceDriver::GetInterfaceVersions() {
	return vr::k_InterfaceVersions;
}

// Must be defined to compile
bool TaurusDeviceDriver::ShouldBlockStandbyMode() {
	return false;
}

void TaurusDeviceDriver::RunFrame() {
	// call our devices to run a frame
	if (leftController != nullptr) {
		leftController->RunFrame();
	}

	if (rightController != nullptr) {
		rightController->RunFrame();
	}

	for (int i = 0; i < trackers.size(); i++) {
		trackers[i]->RunFrame();
	}

	// Process events
	vr::VREvent_t vrevent{};
	while (vr::VRServerDriverHost()->PollNextEvent(&vrevent, sizeof(vr::VREvent_t))) {
		if (leftController != nullptr) {
			leftController->ProcessEvent(vrevent);
		}

		if (rightController != nullptr) {
			rightController->ProcessEvent(vrevent);
		}

		for (int i = 0; i < trackers.size(); i++) {
			trackers[i]->ProcessEvent(vrevent);
		}
	}
}

void TaurusDeviceDriver::EnterStandby() { }

void TaurusDeviceDriver::LeaveStandby() { }

void TaurusDeviceDriver::Cleanup() {
	// Stop the pose update thread
	if (isThreadActive.exchange(false)) {
		// forcefully interrupt recv
		DriverLog("Sending interrupt message to recv socket...");
		sock::SendData(sock_, udpPort, "equit", 5);

		messageThread.join();
	}

	// Cleanup comms
	sock::CloseSocket(sock_);
	sock::CloseSocket(sendSock_);
	sock::CleanupComms();

	// Destroy the controllers
	leftController = nullptr;
	rightController = nullptr;

	// Destroy the trackers
	for (int i = 0; i < trackers.size(); i++) {
		trackers[i] = nullptr;
	}
	trackers.clear();
}

void TaurusDeviceDriver::RequestTrackerList() {
	messages::TrackersRequestMessage request;
	messages::DriverMessage msg;
	msg.mutable_trackers_request_message()->CopyFrom(request);

	SendDriverMessage(msg);
}

void TaurusDeviceDriver::RegisterTracker(const messages::TrackerInfo& trackerInfo) {
	DriverLog("Registering tracker with ID %d", trackerInfo.id());
	
	TrackerDevice* tracker = new TrackerDevice(trackerInfo);
	trackers.push_back(tracker);

	vr::VRServerDriverHost()->TrackedDeviceAdded(tracker->GetSerialNumber().c_str(), vr::TrackedDeviceClass_TrackingReference, tracker);
}

void TaurusDeviceDriver::SendDriverMessage(const messages::DriverMessage& msg) {
	char buffer[1024];
	sock::SerializeDriverMsg(msg, buffer);
	sock::SendData(sendSock_, udpSendPort, buffer, msg.ByteSizeLong());
}

void TaurusDeviceDriver::MessageThread() {
	while (isThreadActive) {
		// wait for data (this is a blocking call)
		char buffer[1024];
		int len = 0;
		bool success = sock::ReceiveData(sock_, buffer, sizeof(buffer), &len);

		// check if msg valid
		if (!success) {
			DriverLog("Invalid message!");
			continue;
		}

		// check for interrupt message
		if (len == 5 && std::strcmp(buffer, "equit") == 0) {
			DriverLog("Received interrupt message, exiting recv thread.");
			break;
		}

		// deserialize the data
		messages::TaurusMessage msg;
		if (!msg.ParseFromArray(buffer, len)) {
			DriverLog("Failed to parse message!");
			continue;
		}

		// find target controller by serial
		std::string serial = msg.serial();
		ControllerDevice* targetController = nullptr;
		if (leftController->MatchesSerialNumber(serial)) {
			targetController = leftController.get();
		}
		else if (rightController->MatchesSerialNumber(serial)) {
			targetController = rightController.get();
		}

		// check for message type
		if (msg.has_pose_message()) {
			const messages::PoseMessage& poseMsg = msg.pose_message();

			if (targetController != nullptr) {
				targetController->ProcessPoseMessage(poseMsg);
			}
			else {
				DriverLog("No controller matches msg serial: %s", serial.c_str());
			}
		}
		else if (msg.has_input_message()) {
			const messages::InputMessage& inputMsg = msg.input_message();
			
			if (targetController != nullptr) {
				targetController->ProcessInputMessage(inputMsg);
			}
			else {
				DriverLog("No controller matches msg serial: %s", serial.c_str());
			}
		}
		else if (msg.has_status_message()) {
			const messages::StatusMessage& stateMsg = msg.status_message();

			if (targetController != nullptr) {
				targetController->ProcessStatusMessage(stateMsg);
			}
			else {
				DriverLog("No controller matches msg serial: %s", serial.c_str());
			}
		}
		else if (msg.has_trackers_request_answer_message()) {
			DriverLog("Got tracker list answer! Registering trackers..");
			const messages::TrackersRequestAnswerMessage& answerMsg = msg.trackers_request_answer_message();

			for (int i = 0; i < answerMsg.trackers_size(); i++) {
				const messages::TrackerInfo& tracker = answerMsg.trackers(i);
				
				RegisterTracker(tracker);
			}

			waitingForRequestAnswer = false;
		}
	}
}
