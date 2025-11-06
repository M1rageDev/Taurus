/*
FILE DESCRIPTION:

This is the main app handler, which manages all the subthreads
*/

#include "app/taurus_app.h"

taurus::TaurusApp::TaurusApp() {
	Init();
}

void taurus::TaurusApp::Run() {
	running = true;

	// start program
	opticalThread->Start();
	filterThread->Start();
	commsThread->Start();
	controllers->StartUpdateThreads();
	MainLoop();

	// cleanup
	Stop();
}

void taurus::TaurusApp::Init() {
	// Initialize config
	configManager = new TaurusConfig();
	configManager->LoadConfig();
	TaurusConfigStorage* configStorage = configManager->GetStorage();

	// Initialize controllers
	expectedControllers = {
		configStorage->leftControllerSerial.value(),
		configStorage->rightControllerSerial.value(),
	};
	controllers = new ControllerManager(expectedControllers);
	controllers->ConnectControllers();
	connectedControllers = controllers->GetConnectedSerials();

	controllers->GetController(expectedControllers[0])->SetColor(configStorage->leftControllerColor.value());
	controllers->GetController(expectedControllers[1])->SetColor(configStorage->rightControllerColor.value());

	// Initialize cameras
	cameraManager = new CameraManager();
	cameraManager->SetupCameras();
	cameraCount = cameraManager->GetCameraCount();

	Camera& camera0 = cameraManager->GetCamera(0);
	camera0.SetExposureMode(Exposure_DARK);
	Camera& camera1 = cameraManager->GetCamera(1);
	camera1.SetExposureMode(Exposure_DARK);

	frame = camera0.InitFrameMat();

	// initialize the subthreads
	opticalThread = new OpticalThread();
	filterThread = new FilterThread();
	commsThread = new CommunicationThread();

	trackedControllers = opticalThread->GetTrackedObjects();

	logging::info("TaurusApp init finished");
}

void taurus::TaurusApp::Stop() {
	commsThread->Stop();
	filterThread->Stop();
	opticalThread->Stop();

	sock::CleanupComms();

	controllers->StopUpdateThreads();
	controllers->DisconnectControllers();

	cameraManager->Stop();
}

void taurus::TaurusApp::MainLoop() {
	TaurusConfigStorage* configStorage = configManager->GetStorage();

	while (running) {
		// show preview for each cam
		if (configStorage->showPreview.value_or(true)) {
			for (int i = 0; i < cameraCount; i++) {
				Camera& cam = cameraManager->GetCamera(i);
				cam.GetFrame(frame);

				// annotations
				if (configStorage->annotatePreview.value_or(true)) {
					cv::putText(frame, std::format("Optical Hz: {}", opticalThread->GetFps()), {0, 20}, cv::FONT_HERSHEY_PLAIN, 1.2, {255, 255, 255});

					for (int controllerI = 0; controllerI < trackedControllers->size(); controllerI++) {
						tracking::TrackedObject& obj = (*trackedControllers)[controllerI];

						// if we have tracking data for this camera, show it
						tracking::TrackedObject::PerCameraData& thisCameraData = obj.perCameraData[i];
						if (thisCameraData.acquiredTracking) {
							if (configStorage->annotatePreview.value_or(true)) {
								cv::rectangle(frame, thisCameraData.roi, { 255, 255, 255 }, 1);
								cv::circle(frame, thisCameraData.globalCircleCenter, 3, cv::Scalar(255, 255, 255), -1);
								cv::putText(frame, connectedControllers[controllerI], thisCameraData.globalCircleCenter, cv::FONT_HERSHEY_PLAIN, 1.2, { 255, 255, 255 });
							}
						}

						// if we have 3D position, show it
						if (obj.acquired3DPosition) {
							std::string posText = std::format(
								"Controller {} Pos - X:{:.2f} Y:{:.2f} Z:{:.2f}",
								controllerI,
								obj.filteredPosition.x,
								obj.filteredPosition.y,
								obj.filteredPosition.z
							);
							cv::putText(frame, posText, { 0, 40 + controllerI * 20 }, cv::FONT_HERSHEY_PLAIN, 1.2, { 255, 255, 255 });
						}
					}
				}

				// show frame
				cv::imshow(std::format("Frame {}", i), frame);
			}
		}

		// handle user input
		// TODO: maybe move this somewhere else?
		int i = 0;
		for (std::string& serial : connectedControllers) {
			Controller* controller = controllers->GetController(serial);

			if (controller->IsButtonPressed(Btn_SELECT)) {
				// relevel
				controller->ResetAhrs();

				if (controller->IsButtonPressed(Btn_START)) {
					filterThread->SetPositionPostOffset(glm::vec3((*trackedControllers)[i].preFilteredPosition));  // pre-filtered position
				}
			}

			i++;
		}

		// wait for keypress
		if (cv::waitKey(1) == 27) break;
	}
}
