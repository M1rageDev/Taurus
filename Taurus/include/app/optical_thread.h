#pragma once

#include <thread>

#include "modules/tracking/tracking_utils.h"
#include "modules/tracking/detector.h"
#include "modules/cameras.h"
#include "modules/config.h"
#include "modules/psmove.h"

namespace taurus
{
	class OpticalThread
	{
		public:
			static OpticalThread* GetInstance();

			OpticalThread();

			void Start();
			void Stop();

			int GetFps() const;
			std::vector<tracking::TrackedObject>* GetTrackedObjects();
		private:
			static OpticalThread* instance;

			void ThreadFunc();

			TaurusConfig* config;
			ControllerManager* controllers;
			CameraManager* cameraManager;

			std::vector<tracking::TrackedObject> trackedObjects;
			int fps = 0;

			std::vector<std::string> connectedControllers;
			size_t cameraCount;
			CameraCalibration calib0;
			CameraCalibration calib1;
			cv::Mat frame;

			std::thread thread;
			std::atomic<bool> threadActive = false;
	};
}
