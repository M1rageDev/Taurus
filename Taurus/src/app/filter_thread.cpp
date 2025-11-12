/*
FILE DESCRIPTION:

Subthread which manages the filtering of the tracking data, as well as post processing
*/

#include "app/filter_thread.h"

#include "core/filter/lowpass.h"
#include "core/logging.h"

taurus::FilterThread::FilterThread() {
	this->config = TaurusConfig::GetInstance();
	this->controllers = ControllerManager::GetInstance();

	this->lowpassAlpha = config->GetStorage()->lowpassAlpha.value_or(0.4f);
	this->lowpassDistance = config->GetStorage()->lowpassDistance.value_or(20.0f);
}

void taurus::FilterThread::Start() {
	threadActive.store(true);
	thread = std::thread(&FilterThread::ThreadFunc, this);

	logging::info("Started filter thread");
}

void taurus::FilterThread::Stop() {
	threadActive.store(false);
	thread.join();
}

void taurus::FilterThread::SetPositionPostOffset(glm::vec3 offset) {
	positionPostOffset = offset;
}

void taurus::FilterThread::ThreadFunc() {
	long lastTick = psmove_util_get_ticks();
	long now;
	float fps, msPassed, secPassed;
	while (threadActive.load()) {
		now = psmove_util_get_ticks();
		msPassed = static_cast<float>(now - lastTick);
		lastTick = now;

		fps = 1000.f / msPassed;
		secPassed = msPassed / 1000.f;

		// for every connected controller
		int i = 0;
		for (auto& serial : controllers->GetConnectedSerials()) {
			Controller* controller = controllers->GetController(serial);
			tracking::TrackedObject* obj = controller->GetTrackedObject();

			// if we've got optical data (reliable but slow), use it and reset the IMU kinematics
			if (obj->newOpticalDataReady) {
				// request the optical position to be the one for filtering
				obj->preFilteredPosition = obj->worldPosition;

				// reset kinematic state and update the velocity with the new reliable optical velocity
				obj->kinematic.SetPosition(glm::vec3(0.f));
				obj->kinematic.SetVelocity(obj->opticalVelocity);

				// we've handled the new data, so reset the flag
				obj->newOpticalDataReady = false;
			}
			else {
				// we are inbetween optical measurements or we've lost tracking
				// integrate IMU kinematics and use it as the position
				obj->kinematic.Integrate(secPassed);
				obj->preFilteredPosition = obj->worldPosition + obj->kinematic.GetPosition();
			}

			// apply a filter, to reduce noise
			obj->filteredPosition = filter::improvedLowpassFilter(
				obj->previousFilteredPosition,
				obj->preFilteredPosition,
				lowpassAlpha,
				lowpassDistance
			);
			obj->previousFilteredPosition = obj->filteredPosition;

			// post processing
			obj->filteredPosition -= positionPostOffset;
			obj->filteredPositionM = obj->filteredPosition * 0.01f;
			
			i++;
		}

		// wait a bit
		psmove_util_sleep_ms(1);
	}
}
