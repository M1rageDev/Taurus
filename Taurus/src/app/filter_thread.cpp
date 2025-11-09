/*
FILE DESCRIPTION:

Subthread which manages the filtering of the tracking data, as well as post processing
*/

#include "app/filter_thread.h"

#include "core/filter/lowpass.h"
#include "core/logging.h"

#include "app/optical_thread.h"

taurus::FilterThread::FilterThread() {
	this->config = TaurusConfig::GetInstance();
	this->controllers = ControllerManager::GetInstance();

	this->lowpassAlpha = config->GetStorage()->lowpassAlpha.value_or(glm::vec3(0.4f, 0.4f, 0.3f));
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

			// handle new optical data
			if (obj->newOpticalDataReady) {
				obj->preFilteredPosition = obj->worldPosition;

				// reset state and plug in the predicted optical velocity
				obj->kinematic.ResetState();
				obj->kinematic.SetVelocity(obj->opticalVelocityM);

				// we've handled the new data
				obj->newOpticalDataReady = false;
			}
			else {
				// get data from dead reckoning, if no data available yet
				obj->kinematic.Integrate(secPassed);
				obj->preFilteredPosition = obj->worldPosition + obj->kinematic.GetPosition();
			}

			// filter
			/*
			obj->filteredPosition = filter::improvedLowpassFilter(
				obj->previousFilteredPosition,
				obj->preFilteredPosition,
				lowpassAlpha
			);
			*/
			obj->filteredPosition = filter::lowpassFilter(
				obj->previousFilteredPosition,
				obj->preFilteredPosition,
				lowpassAlpha
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
