#include <iostream>
#include <string>

#include "modules/camera_calibration.h"
#include "modules/imu_calibration.h"
#include "modules/logging.h"
#include "modules/cameras.h"
#include "modules/psmove.h"

namespace logging = taurus::logging;

static void gyroCalibration(std::string controllerSerial) {
	taurus::GyroCalibrator calibrator = taurus::GyroCalibrator(controllerSerial);
	calibrator.RunCalibration();
}

static void colorCalibration(std::string controllerSerial, int cameraId) {
	taurus::ColorCalibrator calibrator = taurus::ColorCalibrator(cameraId, controllerSerial);
	calibrator.RunCalibration();
}

static void intrinsicCalibration(int cameraId) {
	taurus::IntrinsicCalibrator calibrator = taurus::IntrinsicCalibrator(cameraId);
	calibrator.RunCalibration();
}

static void extrinsicCalibration(std::string controllerSerial, int cameraId0, int cameraId1) {
	taurus::ExtrinsicCalibrator calibrator = taurus::ExtrinsicCalibrator(cameraId0, cameraId1, controllerSerial);
	calibrator.RunCalibration();
}

int main() {
	// Initialize controllers
	taurus::ControllerManager controllers({});
	controllers.ConnectControllers();
	controllers.UpdateControllers();

	// Initialize cameras
	taurus::CameraManager cameraManager;
	cameraManager.SetupCameras();
	size_t cameraCount = cameraManager.GetCameraCount();

	// Print available modes;
	logging::info("");
	logging::info("---------------");
	logging::info("Available controllers:");
	for (std::string serial : controllers.GetConnectedSerials()) {
		logging::info("%s Battery: %f %", serial.c_str(), controllers.GetController(serial)->GetBattery01() * 100.f);
	}
	logging::info("---------------");
	logging::info("Available modes:");
	logging::info("[name - letter - args]");
	logging::info("gyro -      g - cSerial");
	logging::info("color -     c - cSerial, camId");
	logging::info("intrinsic - i - camId");
	logging::info("extrinsic - e - cSerial, camId0, camId1");
	logging::info("---------------");
	logging::info("To correctly calibrate from zero, do it in the same order as the mode list.");

	// get input
	logging::info("Enter calibration mode letter with args separated by spaces -> ");
	std::string input;
	std::getline(std::cin, input);
	std::istringstream inputStream = std::istringstream(input);

	// tokenize
	std::vector<std::string> tokens;
	std::string s;
	while (std::getline(inputStream, s, ' ')) {
		tokens.push_back(s);
	}

	// do the calibration
	std::string command = tokens[0];
	if (command == "g") {
		gyroCalibration(tokens[1]);
	}
	else if (command == "c") {
		colorCalibration(tokens[1], std::stoi(tokens[2]));
	}
	else if (command == "i") {
		intrinsicCalibration(std::stoi(tokens[1]));
	}
	else if (command == "e") {
		extrinsicCalibration(tokens[1], std::stoi(tokens[2]), std::stoi(tokens[3]));
	}
	else {
		logging::error("Invalid mode!");
	}

	controllers.DisconnectControllers();
	cameraManager.Stop();
	return 0;
}
