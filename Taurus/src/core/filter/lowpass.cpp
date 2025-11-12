#include "core/filter/lowpass.h"

#include <algorithm>

glm::vec3 taurus::filter::lowpassFilter(const glm::vec3& last, const glm::vec3& current, const glm::vec3& alpha) {
	return glm::mix(last, current, alpha);
}

glm::vec3 taurus::filter::improvedLowpassFilter(const glm::vec3& last, const glm::vec3& current, float alpha, float dist) {
	float vel = glm::length(last - current);

	float weight = std::clamp(std::lerp(alpha, 1.f, vel / dist), 0.f, 1.f);
	return glm::mix(last, current, weight);
}
