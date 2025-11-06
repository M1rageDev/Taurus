#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "core/psmove.h"

namespace taurus
{
	class GyroCalibrator {
		public:
			GyroCalibrator(std::string serial, int sampleCount = 5000);

			void RunCalibration();
		private:
			Controller* controller;
			std::string serial;

			int minSamples;
			std::vector<glm::vec3> samples;

			glm::vec3 resultOffsets;
	};
}
