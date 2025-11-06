#pragma once

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <psmoveapi/psmove.h>

#include "modules/communication.h"  // IMPORTANT: needs to be included first, not even sure where is winsock included after this
#include "modules/tracking/detector.h"
#include "modules/logging.h"
#include "modules/utils.h"
#include "modules/cameras.h"
#include "modules/psmove.h"
#include "modules/config.h"

#include "app/communication_thread.h"
#include "app/filter_thread.h"
#include "app/optical_thread.h"

namespace taurus
{
	class TaurusApp
	{
		public:
			TaurusApp();

			void Run();
		private:
			void Init();
			void Stop();
			void MainLoop();

			bool running = false;

			TaurusConfig* configManager;

			std::vector<std::string> expectedControllers;
			std::vector<std::string> connectedControllers;
			ControllerManager* controllers;

			CameraManager* cameraManager;
			size_t cameraCount;
			cv::Mat frame;

			std::vector<tracking::TrackedObject>* trackedControllers;
			
			CommunicationThread* commsThread;
			FilterThread* filterThread;
			OpticalThread* opticalThread;
	};
}
