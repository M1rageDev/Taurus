#include "core/imu_calibration.h"

#include "core/json_handler.h"
#include "core/logging.h"

taurus::GyroCalibrator::GyroCalibrator(std::string serial, int sampleCount) {
	this->controller = ControllerManager::GetInstance()->GetController(serial);
	this->serial = serial;

	this->minSamples = sampleCount;
	this->samples = std::vector<glm::vec3>();

	this->resultOffsets = glm::vec3();
}

void taurus::GyroCalibrator::RunCalibration() {
	logging::info("Starting gyro calibration in 2s...");
	logging::info("Place controller on stable platform, without any movement");
	psmove_util_sleep_ms(2000);

	controller->SetColor("yellow");

	logging::info("Starting sample collection");
	while (samples.size() < minSamples) {
		bool newData = controller->Update();

		// only collect if got any new data
		if (newData) {
			samples.push_back(controller->GetGyro());
			logging::info("Captured sample %d", samples.size());
		}
	}

	// average samples
	glm::vec3 sum = glm::vec3();
	for (glm::vec3 sample : samples) {
		sum += sample;
	}

	glm::vec3 offsets = sum / static_cast<float>(samples.size());
	logging::info("Gyro calibration finished.");
	logging::info("Offset X: %f", offsets.x);
	logging::info("Offset Y: %f", offsets.y);
	logging::info("Offset Z: %f", offsets.z);

	// save data
	json data = createGyroData(offsets);
	saveJson(createGyroPath(serial), data);
}
