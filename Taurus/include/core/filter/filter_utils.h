#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace taurus::filter
{
	class KinematicObject {
		public:
			KinematicObject();

			void Integrate(float dt);
			void ResetState();

			glm::vec3& GetPosition();
			glm::vec3& GetVelocity();

			void SetPosition(const glm::vec3& pos);
			void SetVelocity(const glm::vec3& vel);

			void UpdateIMU(const glm::vec3& accel, const glm::quat& orient);
		private:
			glm::vec3 RemoveGravity(const glm::vec3& accel, const glm::quat& orient);

			glm::vec3 position = {};
			glm::vec3 velocity = {};
			glm::vec3 linearAcceleration = {};
	};
}