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

			glm::vec3 positionPostOffset = {};
			glm::vec3 lowpassAlpha = {};
			float velocityDecay = 1.f;

			std::thread thread;
			std::atomic<bool> threadActive = false;
	};
}
