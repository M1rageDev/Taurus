/*
FILE DESCRIPTION:

Subthread which manages the filtering of the tracking data, as well as post processing
*/

#include "app/filter_thread.h"

#include "modules/lowpass.h"
#include "modules/logging.h"

#include "app/optical_thread.h"

taurus::FilterThread::FilterThread() {
	this->config = TaurusConfig::GetInstance();
	this->controllers = ControllerManager::GetInstance();
	this->trackedObjects = OpticalThread::GetInstance()->GetTrackedObjects();

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
	while (threadActive.load()) {
		// for every connected controller
		int i = 0;
		for (auto& serial : controllers->GetConnectedSerials()) {
			Controller* controller = controllers->GetController(serial);
			std::vector<tracking::TrackedObject> objs = *trackedObjects;
			tracking::TrackedObject& obj = (*trackedObjects)[i];

			if (obj.acquired3DPosition) {
				// filter (lowpass)
				obj.preFilteredPosition = taurus::tracking::cvPoint3fToGlmVec3(obj.worldPosition);
				obj.filteredPosition = taurus::improvedLowpassFilter(
					obj.previousFilteredPosition,
					obj.preFilteredPosition,
					lowpassAlpha
				);
				obj.previousFilteredPosition = obj.filteredPosition;

				obj.filteredPosition -= positionPostOffset;
				obj.filteredPositionM = obj.filteredPosition * 0.01f;
			}
			
			i++;
		}

		// wait a bit
		psmove_util_sleep_ms(1);
	}
}
