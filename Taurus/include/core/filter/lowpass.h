#pragma once

#include <glm/glm.hpp>

namespace taurus::filter
{
	// very simple lowpass filter
	// takes a portion of the last position and the current position
	// alpha parameter determines how much to trust the new position (1 - full trust, 0 - nothing happens [no trust])
	// alpha should be in the range [0.0, 1.0]
	glm::vec3 lowpassFilter(const glm::vec3& last, const glm::vec3& current, const glm::vec3& alpha);

	glm::vec3 improvedLowpassFilter(const glm::vec3& last, const glm::vec3& current, float alpha, float dist);
}
