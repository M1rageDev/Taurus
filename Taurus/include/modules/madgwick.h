#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace taurus
{
	struct MadgwickState {
		glm::quat state = glm::quat();

		long lastSample = 0;
		float freqEstimate = 0.f;
	};

	void madgwickUpdate(MadgwickState* state, const glm::vec3& g, const glm::vec3& a, float timestep, float beta = 0.035f);
}
