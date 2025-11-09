#include "core/filter/filter_utils.h"

#include "core/logging.h"

taurus::filter::KinematicObject::KinematicObject() {
	ResetState();
}

void taurus::filter::KinematicObject::IntegrateIMU(const glm::vec3& accel, const MadgwickState& state, float dt) {
	UpdateIMU(accel, state);
	Integrate(dt);
}

void taurus::filter::KinematicObject::Integrate(float dt) {
	velocity += linearAcceleration * dt;
	position += velocity * dt;
}

void taurus::filter::KinematicObject::ResetState() {
	position = glm::vec3(0.f);
	velocity = glm::vec3(0.f);
	// we're not resetting the accel since it's controller by the controller threads
}

glm::vec3& taurus::filter::KinematicObject::GetPosition() {
	return position;
}

glm::vec3& taurus::filter::KinematicObject::GetVelocity() {
	return velocity;
}

void taurus::filter::KinematicObject::SetVelocity(const glm::vec3& vel) {
	velocity = glm::vec3(vel);
}

void taurus::filter::KinematicObject::SetAcceleration(const glm::vec3& acc) {
	linearAcceleration = glm::vec3(acc);
}

void taurus::filter::KinematicObject::SetRawAcceleration(const glm::vec3& acc) {
	accelerationWithG = acc;
}

void taurus::filter::KinematicObject::UpdateIMU(const glm::vec3& accel, const MadgwickState& state) {
	static const float earthGravity = 9.81f;  // m/s2
	accelerationWithG *= glm::vec3(accel * earthGravity);

	linearAcceleration = RemoveGravity(accel, state.state);
}

glm::vec3 taurus::filter::KinematicObject::RemoveGravity(const glm::vec3& accel, const glm::quat& orient) {
	static const glm::vec3 gravityDir = glm::vec3(0.f, 0.f, 1.f);
	return glm::vec3(accel - (glm::conjugate(orient) * gravityDir));
}
