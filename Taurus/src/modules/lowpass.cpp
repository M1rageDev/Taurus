#include "modules/lowpass.h"

#include <algorithm>

glm::vec3 taurus::lowpassFilter(const glm::vec3& last, const glm::vec3& current, const glm::vec3& alpha) {
	return glm::mix(last, current, alpha);
}

glm::vec3 taurus::improvedLowpassFilter(const glm::vec3& last, const glm::vec3& current, const glm::vec3& alpha) {
	float invDelta = 1.f / glm::length(last - current);

	glm::vec3 weight = glm::vec3();
	weight.x = std::lerp(0.01f, 0.75f, invDelta * alpha.x);
	weight.y = std::lerp(0.01f, 0.75f, invDelta * alpha.y);
	weight.z = std::lerp(0.01f, 0.75f, invDelta * alpha.z);
	weight = glm::clamp(weight, glm::vec3(0.f), glm::vec3(1.f));

	glm::vec3 result = glm::vec3();
	result.x = std::lerp(last.x, current.x, weight.x);
	result.y = std::lerp(last.y, current.y, weight.y);
	result.z = std::lerp(last.z, current.z, weight.z);
	return result;
}
