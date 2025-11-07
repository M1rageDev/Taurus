#pragma once

#include <thread>

#include "core/tracking/tracking_utils.h"
#include "core/tracking/detector.h"
#include "core/cameras.h"
#include "core/config.h"
#include "core/psmove.h"

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
			std::vector<tracking::TrackedObject*>* GetTrackedObjects();
		private:
			static OpticalThread* instance;

			void ThreadFunc();

			TaurusConfig* config;
			ControllerManager* controllers;
			CameraManager* cameraManager;

			std::vector<tracking::TrackedObject*> trackedObjects;
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
