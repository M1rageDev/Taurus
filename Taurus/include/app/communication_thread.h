#pragma once

#include <thread>

#include "core/communication.h"
#include "core/tracking/tracking_utils.h"
#include "core/config.h"
#include "core/psmove.h"
#include "core/cameras.h"

namespace taurus
{
	class CommunicationThread
	{
		public:
			CommunicationThread();

			void Start();
			void Stop();
		private:
			void RecvThreadFunc();
			void SendThreadFunc();

			void HandleDriverMessage(const messages::DriverMessage& msg);
			void HandleHapticMessage(const messages::HapticMessage& msg, std::string serial);
			void HandleTrackersRequest(const messages::TrackersRequestMessage& request);

			void PreparePoseMessage(messages::TaurusMessage& msg, Controller* controller, std::string serial, int i);
			void PrepareInputMessage(messages::TaurusMessage& msg, Controller* controller, std::string serial);
			void PrepareStatusMessage(messages::TaurusMessage& msg, Controller* controller, std::string serial);
			void SendMsg(messages::TaurusMessage& msg) const;

			void InterruptRecvSocket() const;

			TaurusConfig* config;
			ControllerManager* controllers;
			CameraManager* cameraManager;
			std::vector<tracking::TrackedObject>* trackedObjects;

			std::thread recvThread;
			std::thread sendThread;
			std::atomic<bool> recvThreadActive = false;
			std::atomic<bool> sendThreadActive = false;

			SOCKET recvSock;
			SOCKET sendSock;
			int recvPort;
			int sendPort;

			long lastStatusSendTime = 0;
	};
}
