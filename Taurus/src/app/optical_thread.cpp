/*
FILE DESCRIPTION:

Subthread which manages the optical tracking system
*/

#include "app/optical_thread.h"

#include "modules/utils.h"
#include "modules/logging.h"

taurus::OpticalThread* taurus::OpticalThread::instance = nullptr;

taurus::OpticalThread* taurus::OpticalThread::GetInstance() {
	return instance;
}

taurus::OpticalThread::OpticalThread() {
	instance = this;

	this->config = TaurusConfig::GetInstance();
	this->controllers = ControllerManager::GetInstance();
	this->connectedControllers = controllers->GetConnectedSerials();

	this->cameraManager = CameraManager::GetInstance();
	cameraCount = cameraManager->GetCameraCount();

	Camera& camera0 = cameraManager->GetCamera(0);
	camera0.SetExposureMode(Exposure_DARK);
	Camera& camera1 = cameraManager->GetCamera(1);
	camera1.SetExposureMode(Exposure_DARK);

	calib0 = camera0.GetCalibration();
	calib1 = camera1.GetCalibration();
	frame = camera0.InitFrameMat();

	// init the tracked object list
	trackedObjects = std::vector<tracking::TrackedObject>();
	for (std::string serial : connectedControllers) {
		// run for every connected controller
		Controller* controller = controllers->GetController(serial);

		// add initial per-camera data to the controller
		tracking::TrackedObject obj = tracking::TrackedObject();
		for (int i = 0; i < cameraCount; i++) {
			Camera& cam = cameraManager->GetCamera(i);

			tracking::TrackedObject::PerCameraData data;
			data.acquiredTracking = false;
			data.color = cam.GetHsvColorRange(controller->GetColorName());
			data.roi = tracking::createFrameRoi(frame);

			obj.perCameraData.push_back(data);
		}

		trackedObjects.push_back(obj);
	}
}

void taurus::OpticalThread::Start() {
	threadActive.store(true);
	thread = std::thread(&OpticalThread::ThreadFunc, this);

	logging::info("Started optical thread");
}

void taurus::OpticalThread::Stop() {
	threadActive.store(false);
	thread.join();
}

int taurus::OpticalThread::GetFps() const {
	return fps;
}

std::vector<taurus::tracking::TrackedObject>* taurus::OpticalThread::GetTrackedObjects() {
	return &trackedObjects;
}

void taurus::OpticalThread::ThreadFunc() {
	TaurusConfigStorage* configStorage = config->GetStorage();

	long lastTick = psmove_util_get_ticks();
	while (threadActive.load()) {
		long now = psmove_util_get_ticks();
		long msPassed = now - lastTick;
		lastTick = now;
		fps = roundToInt(1000.f / static_cast<float>(msPassed));

		// do for each cam
		for (int i = 0; i < cameraCount; i++) {
			taurus::Camera& cam = cameraManager->GetCamera(i);
			cam.GetFrame(frame);

			// track the controllers
			taurus::tracking::findMultiBalls(frame, trackedObjects, i);
			for (taurus::tracking::TrackedObject& obj : trackedObjects) {
				auto& thisCameraData = obj.perCameraData[i];

				if (!thisCameraData.acquiredTracking) {
					// lost tracking
					// increase ROI size to try and find the controller
					taurus::tracking::increaseRoiSize(thisCameraData.roi, 100);
					taurus::tracking::clampRoi(frame, thisCameraData.roi);
				}
			}
		}

		// track every controller in 3D
		for (taurus::tracking::TrackedObject& obj : trackedObjects) {
			// if we have tracking data from both cameras, triangulate
			obj.acquired3DPosition = obj.perCameraData[0].acquiredTracking && obj.perCameraData[1].acquiredTracking;
			if (obj.acquired3DPosition) {
				// undistort
				cv::Point2f undistorted0 = taurus::tracking::undistort(obj.perCameraData[0].globalCircleCenter, calib0.K, calib0.distort);
				cv::Point2f undistorted1 = taurus::tracking::undistort(obj.perCameraData[1].globalCircleCenter, calib1.K, calib1.distort);

				// triangulate
				obj.triangulatedPosition = taurus::tracking::triangulate(calib0.P, calib1.P, obj.perCameraData[0], obj.perCameraData[1]);
				obj.worldPosition = taurus::tracking::transform(calib0.world, obj.triangulatedPosition);

				// store last frame pos, for future filtering
				obj.previousWorldPosition = obj.worldPosition;
			}
		}
	}
}
