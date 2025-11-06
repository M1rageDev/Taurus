#pragma once

#include <thread>

#include "core/tracking/tracking_utils.h"
#include "core/config.h"
#include "core/psmove.h"

namespace taurus
{
	class FilterThread
	{
		public:
			FilterThread();

			void Start();
			void Stop();

			void SetPositionPostOffset(glm::vec3 offset);
		private:
			void ThreadFunc();

			TaurusConfig* config;
			ControllerManager* controllers;
			std::vector<tracking::TrackedObject>* trackedObjects;

			glm::vec3 positionPostOffset = {};
			glm::vec3 lowpassAlpha;

			std::thread thread;
			std::atomic<bool> threadActive = false;
	};
}
