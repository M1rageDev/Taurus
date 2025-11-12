#include "core/filter/filter_utils.h"

#include "core/logging.h"

taurus::filter::KinematicObject::KinematicObject() {
	ResetState();
}

void taurus::filter::KinematicObject::Integrate(float dt) {
	// integrate acceleration to get velocity, then integrate velocity to get position
	velocity += linearAcceleration * dt;
	position += velocity * dt;
}

void taurus::filter::KinematicObject::ResetState() {
	// we're not resetting the accel since it's controlled by the IMU 
	position = glm::vec3(0.f);
	velocity = glm::vec3(0.f);
}

glm::vec3& taurus::filter::KinematicObject::GetPosition() {
	return position;
}

glm::vec3& taurus::filter::KinematicObject::GetVelocity() {
	return velocity;
}

void taurus::filter::KinematicObject::SetPosition(const glm::vec3& pos) {
	position = glm::vec3(pos);
}

void taurus::filter::KinematicObject::SetVelocity(const glm::vec3& vel) {
	velocity = glm::vec3(vel);
}

void taurus::filter::KinematicObject::UpdateIMU(const glm::vec3& accel, const glm::quat& orient) {
	// internal calculations are in cm, so the value below is in cm/s2
	static const float earthGravity = 981.0f;
	linearAcceleration = RemoveGravity(accel, orient) * earthGravity;
}

glm::vec3 taurus::filter::KinematicObject::RemoveGravity(const glm::vec3& accel, const glm::quat& orient) {
	static const glm::vec3 gravityDir = glm::vec3(0.f, 0.f, 1.f);

	// rotate gravity by the controller orientation and subtract it from the accel measurement, to get rid of it
	glm::vec3 rotatedGravity = (glm::conjugate(orient) * gravityDir);
	return glm::vec3(accel - rotatedGravity);
}
