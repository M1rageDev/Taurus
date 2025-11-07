/*
FILE DESCRIPTION:

Subthread which manages the communication (sending tracking/status data, receiving requests, etc.)
*/

#include "app/communication_thread.h"

#include <google/protobuf/util/json_util.h>
#include <psmoveapi/psmove.h>

#include "core/logging.h"
#include "core/utils.h"

#include "app/optical_thread.h"

taurus::CommunicationThread::CommunicationThread() {
	this->config = TaurusConfig::GetInstance();
	this->controllers = ControllerManager::GetInstance();
	this->cameraManager = CameraManager::GetInstance();

	TaurusConfigStorage* storage = config->GetStorage();
	this->recvPort = storage->udpRecvPort.value();
	this->sendPort = storage->udpSendPort.value();

	this->recvSock = sock::SetupWSASocket();
	sock::BindWSASocket(this->recvSock, recvPort);
	this->sendSock = sock::SetupWSASocket();
}

void taurus::CommunicationThread::Start() {
	recvThreadActive.store(true);
	sendThreadActive.store(true);
	recvThread = std::thread(&CommunicationThread::RecvThreadFunc, this);
	sendThread = std::thread(&CommunicationThread::SendThreadFunc, this);

	logging::info("Started comms thread");
}

void taurus::CommunicationThread::Stop() {
	recvThreadActive.store(false);
	sendThreadActive.store(false);

	// we need to force shutdown the recv socket to break it out of the blocking recv call
	InterruptRecvSocket();

	recvThread.join();
	sendThread.join();

	sock::CloseSocket(recvSock);
	sock::CloseSocket(sendSock);
}

void taurus::CommunicationThread::RecvThreadFunc() {
	while (recvThreadActive.load()) {
		// wait for data (this is a blocking call)
		char buffer[1024];
		int len = 0;
		bool success = sock::ReceiveData(recvSock, buffer, sizeof(buffer), &len);

		// check if msg valid
		if (!success) {
			logging::error("Invalid message!");
			continue;
		}

		// check for interrupt message
		if (len == 5 && std::strcmp(buffer, "equit") == 0) {
			logging::info("Received interrupt message, exiting recv thread.");
			break;
		}

		// deserialize the data
		messages::DriverMessage msg;
		if (!msg.ParseFromArray(buffer, len)) {
			logging::error("Failed to parse message!");
			continue;
		}
		
		HandleDriverMessage(msg);
	}
}

void taurus::CommunicationThread::SendThreadFunc() {
	while (sendThreadActive) {
		// for every connected controller
		messages::TaurusMessage msg;
		int i = 0;
		for (auto& serial : controllers->GetConnectedSerials()) {
			Controller* controller = controllers->GetController(serial);

			// send pose
			msg.Clear();
			PreparePoseMessage(msg, controller, serial, i);
			SendMsg(msg);

			// send input
			msg.Clear();
			PrepareInputMessage(msg, controller, serial);
			SendMsg(msg);

			i++;
		}

		// for every connected controller, send status every so often
		long now = psmove_util_get_ticks();
		const static int statusSendIntervalMs = 1000;
		if (now - lastStatusSendTime >= statusSendIntervalMs) {
			// for every allocated controller
			for (auto& serial : controllers->GetAllocatedSerials()) {
				msg.Clear();
				PrepareStatusMessage(msg, controllers->GetController(serial), serial);
				SendMsg(msg);
			}

			lastStatusSendTime = now;
		}

		// wait a ms, we dont need to send data more often than 1000hz
		psmove_util_sleep_ms(1);
	}
}

void taurus::CommunicationThread::HandleDriverMessage(const messages::DriverMessage& msg) {
	switch (msg.message_case()) {
		case messages::DriverMessage::kHapticMessage:
			HandleHapticMessage(msg.haptic_message(), msg.serial());
			break;
		case messages::DriverMessage::kTrackersRequestMessage:
			logging::info("Received tracker list request.");
			HandleTrackersRequest(msg.trackers_request_message());
			break;
		default:
			logging::warning("Invalid DriverMessage received!");
			break;
	}
}

void taurus::CommunicationThread::HandleHapticMessage(const messages::HapticMessage& msg, std::string serial) {
	const messages::HapticEvent& event = msg.event();

	float duration = event.duration();
	float frequency = event.frequency();
	float amplitude = event.amplitude();

	// clamp amplitude
	if (amplitude < 0.35f) {
		amplitude = 0.35f;
	}

	// handle single pulse
	if (duration == 0.f) {
		duration = 0.003f;
	}

	controllers->GetController(serial)->DoRumble(duration, amplitude);
	logging::info("%f amplitude %f duration", amplitude, duration);
}

void taurus::CommunicationThread::HandleTrackersRequest(const messages::TrackersRequestMessage& request) {
	messages::TrackersRequestAnswerMessage answer;

	for (int i = 0; i < cameraManager->GetCameraCount(); i++) {
		Camera& cam = cameraManager->GetCamera(i);

		// decompose T matrix to get pos and rot
		cv::Mat T = cam.GetCalibration().world;
		auto [R, t] = tracking::decomposeTransform(T);
		
		glm::vec3 pos = tracking::cvPoint3fToGlmVec3(t);
		pos *= 0.01f;  // cm to m

		// convert rotation matrix to quaternion
		glm::mat3 R_glm = tracking::cvMat3ToGlmMat3(R);
		glm::quat rot = glm::quat_cast(R_glm);

		// fill in msg
		messages::Pose pose;
		pose.mutable_position()->set_x(pos.x);
		pose.mutable_position()->set_y(pos.y);
		pose.mutable_position()->set_z(pos.z);
		pose.mutable_orientation()->set_x(rot.x);
		pose.mutable_orientation()->set_y(rot.y);
		pose.mutable_orientation()->set_z(rot.z);
		pose.mutable_orientation()->set_w(rot.w);

		messages::TrackerInfo* trackerMsg = answer.add_trackers();
		trackerMsg->set_id(i);
		trackerMsg->mutable_pose()->CopyFrom(pose);
	}

	// send answer
	messages::TaurusMessage msg;
	msg.set_serial("");
	msg.mutable_trackers_request_answer_message()->CopyFrom(answer);
	SendMsg(msg);
}

void taurus::CommunicationThread::PreparePoseMessage(messages::TaurusMessage& msg, Controller* controller, std::string serial, int i) {
	glm::quat vrQuat = controller->GetVrQuat();
	tracking::TrackedObject* trackedObject = controller->GetTrackedObject();;
	
	// offset position by half the controller length in the forward direction, since position is measured at the ball
	// TODO: put in config
	static const float halfControllerLength = 0.08782f;
	glm::vec3 offsetPos = tracking::offsetPosition(trackedObject->filteredPositionM, vrQuat, halfControllerLength);

	// fill in msg
	messages::Pose pose;
	pose.mutable_position()->set_x(offsetPos.x);
	pose.mutable_position()->set_y(offsetPos.y);
	pose.mutable_position()->set_z(offsetPos.z);
	pose.mutable_orientation()->set_x(vrQuat.x);
	pose.mutable_orientation()->set_y(vrQuat.y);
	pose.mutable_orientation()->set_z(vrQuat.z);
	pose.mutable_orientation()->set_w(vrQuat.w);

	messages::PoseMessage poseMsg;
	poseMsg.mutable_pose()->CopyFrom(pose);

	msg.set_serial(serial);
	msg.mutable_pose_message()->CopyFrom(poseMsg);
}

// adds input event to an input msg with the specified component and value
static void AddInputEvent(messages::InputMessage& inputMsg, messages::InputComponent component, float value) {
	messages::InputEvent* event = inputMsg.add_events();
	event->set_component(component);
	event->set_value(value);
}

// same as above, but with bool instead of float
static void AddInputEvent(messages::InputMessage& inputMsg, messages::InputComponent component, bool value) {
	messages::InputEvent* event = inputMsg.add_events();
	event->set_component(component);
	event->set_value(value ? 1.f : 0.f);
}

void taurus::CommunicationThread::PrepareInputMessage(messages::TaurusMessage& msg, Controller* controller, std::string serial) {
	messages::InputMessage inputMsg;

	AddInputEvent(inputMsg, messages::InputComponent::SYSTEM, controller->IsButtonPressed(Btn_PS));
	AddInputEvent(inputMsg, messages::InputComponent::MOVE, controller->IsButtonPressed(Btn_MOVE));

	AddInputEvent(inputMsg, messages::InputComponent::SQUARE, controller->IsButtonPressed(Btn_SQUARE));
	AddInputEvent(inputMsg, messages::InputComponent::CROSS, controller->IsButtonPressed(Btn_CROSS));
	AddInputEvent(inputMsg, messages::InputComponent::TRIANGLE, controller->IsButtonPressed(Btn_TRIANGLE));
	AddInputEvent(inputMsg, messages::InputComponent::CIRCLE, controller->IsButtonPressed(Btn_CIRCLE));

	AddInputEvent(inputMsg, messages::InputComponent::START, controller->IsButtonPressed(Btn_START));
	AddInputEvent(inputMsg, messages::InputComponent::SELECT, controller->IsButtonPressed(Btn_SELECT));

	AddInputEvent(inputMsg, messages::InputComponent::TRIGGER, controller->GetTrigger01());

	msg.set_serial(serial);
	msg.mutable_input_message()->CopyFrom(inputMsg);
}

void taurus::CommunicationThread::PrepareStatusMessage(messages::TaurusMessage& msg, Controller* controller, std::string serial) {
	messages::ControllerStatus status;

	status.set_battery_percent(roundToInt(controller->GetBattery01() * 100.f));
	status.set_is_charging(controller->IsCharging());

	status.set_is_connected(controller->IsConnected());
	status.set_is_tracking(controller->IsConnected());

	messages::StatusMessage statusMsg;
	statusMsg.mutable_status()->CopyFrom(status);

	msg.set_serial(serial);
	msg.mutable_status_message()->CopyFrom(statusMsg);
}

void taurus::CommunicationThread::SendMsg(messages::TaurusMessage& msg) const {
	// serialize msg and send
	char buffer[1024];
	sock::SerializeTaurusMsg(msg, buffer);
	sock::SendData(sendSock, sendPort, buffer, static_cast<int>(msg.ByteSizeLong()));
}

void taurus::CommunicationThread::InterruptRecvSocket() const {
	// send interrupt message to unblock recv
	logging::info("Sending interrupt message to recv socket...");
	sock::SendData(recvSock, recvPort, "equit", 5);
}
