#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/madgwick.h"

namespace taurus::filter
{
	class KinematicObject {
		public:
			KinematicObject();

			void IntegrateIMU(const glm::vec3& accel, const MadgwickState& state, float dt);
			void Integrate(float dt);
			void ResetState();

			glm::vec3& GetPosition();
			glm::vec3& GetVelocity();

			void SetVelocity(const glm::vec3& vel);
			void SetAcceleration(const glm::vec3& acc);
		private:
			glm::vec3 RemoveGravity(const glm::vec3& accel, const glm::quat& orient);

			glm::vec3 position = {};
			glm::vec3 velocity = {};
			glm::vec3 linearAcceleration = {};

			glm::vec3 accelerationWithG = {};
	};
}