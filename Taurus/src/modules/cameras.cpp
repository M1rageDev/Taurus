#include "modules/cameras.h"

#include "modules/logging.h"
#include "modules/json_handler.h"

taurus::Camera::Camera(uint8_t id, ps3eye::PS3EYECam::PS3EYERef ps3eyeRef, int width, int height, uint16_t fps, ExposureMode exposureMode) {
	this->ps3eyeRef = ps3eyeRef;
	this->id = id;
	
	ps3eyeRef->init(width, height, fps);
	ps3eyeRef->start();

	this->width = width;
	this->height = height;
	this->fps = fps;

	SetExposureMode(exposureMode);

	LoadData();
}

void taurus::Camera::LoadData() {
	logging::info("Loading camera data for cam %d", id);

	json data;

	// load color
	bool success = readJson(createColorPath(id), &data);
	calibration.hasColor = data.contains("colors");
	if (success && calibration.hasColor) {
		calibration.colorDict = std::unordered_map<std::string, tracking::HsvColorRange>();
		for (auto& color : data["colors"]) {
			std::string name = color["name"];

			cv::Scalar lower, upper;
			jsonReadCvScalar(color, "lower", &lower);
			jsonReadCvScalar(color, "upper", &upper);

			tracking::HsvColorRange colorRange = { lower, upper };
			calibration.colorDict[name] = colorRange;
		}

		logging::info("Successfully loaded color calibration");
	}
	else {
		logging::warning("Color calibration could not be loaded, this may cause issues!");
	}

	// load intrinsic
	data = json();
	success = readJson(createIntrinsicPath(id), &data);
	calibration.hasIntrinsic = data.contains("K") && data.contains("distort");
	if (success && calibration.hasIntrinsic) {
		jsonReadCvMat(data, "K", &calibration.K);
		jsonReadCvMat(data, "distort", &calibration.distort);

		logging::info("Fx: %f", calibration.K.at<double>(0, 0));
		logging::info("Fy: %f", calibration.K.at<double>(1, 1));
		logging::info("Cx: %f", calibration.K.at<double>(0, 2));
		logging::info("Cy: %f", calibration.K.at<double>(1, 2));

		logging::info("Successfully loaded intrinsic calibration");
	}
	else {
		logging::warning("Intrinsic calibration could not be loaded, this may cause issues!");
	}

	// load extrinsic
	data = json();
	success = readJson(createExtrinsicPath(id), &data);
	calibration.hasExtrinsic = data.contains("T") && data.contains("world");
	if (success && calibration.hasExtrinsic) {
		jsonReadCvMat(data, "T", &calibration.T);
		jsonReadCvMat(data, "world", &calibration.world);

		logging::info("Successfully loaded extrinsic calibration");
	}
	else {
		logging::warning("Extrinsic calibration could not be loaded, this may cause issues!");
	}

	// create P matrix
	if (calibration.hasIntrinsic && calibration.hasExtrinsic) {
		calibration.P = calibration.K * calibration.T;
		logging::info("Successfully created projection matrix P");
	}
}

void taurus::Camera::SetExposureMode(ExposureMode mode) {
	exposureMode = mode;

	if (mode == Exposure_AUTO) {
		ps3eyeRef->setAutogain(true);
		ps3eyeRef->setExposure(120);
	} else if (mode == Exposure_DARK) {
		ps3eyeRef->setAutogain(false);
		ps3eyeRef->setGain(0);
		ps3eyeRef->setExposure(10);
	}
}

void taurus::Camera::GetFrame(cv::Mat& frame) {
	ps3eyeRef->getFrame(frame.data);
}

uint8_t taurus::Camera::GetID() const {
	return id;
}

bool taurus::Camera::IsStarted() const {
	return isStarted;
}

taurus::tracking::HsvColorRange taurus::Camera::GetHsvColorRange(std::string color) {
	if (calibration.hasColor) {
		return calibration.colorDict[color];
	}
	else {
		return tracking::HsvColorRange();
	}
}

taurus::CameraCalibration taurus::Camera::GetCalibration() const {
	return calibration;
}

cv::Mat taurus::Camera::InitFrameMat() {

	return cv::Mat(ps3eyeRef->getHeight(), ps3eyeRef->getWidth(), CV_8UC3);
}

void taurus::Camera::Stop() {
	ps3eyeRef->stop();
}

taurus::CameraManager* taurus::CameraManager::instance = nullptr;

taurus::CameraManager* taurus::CameraManager::GetInstance() {
	return instance;
}

taurus::CameraManager::CameraManager() {
	taurus::CameraManager::instance = this;

	// find eye devices
	logging::info("Initializing cameras...");
	ps3eyeReferences = ps3eye::PS3EYECam::getDevices();
	ps3eyeCount = ps3eyeReferences.size();
	logging::info("Found %d PS3 Eye cameras.", ps3eyeCount);
}

void taurus::CameraManager::SetupCameras(int width, int height, uint16_t fps, taurus::ExposureMode exposureMode) {
	// init all the cameras
	for (uint8_t i = 0; i < ps3eyeCount; i++) {
		logging::info("Setting up camera %d ...", i);

		ps3eye::PS3EYECam::PS3EYERef eye = ps3eyeReferences[i];

		Camera camera = Camera(i, eye, width, height, fps, exposureMode);
		cameras.push_back(camera);
	}
}

size_t taurus::CameraManager::GetCameraCount() const {
	return ps3eyeCount;
}

taurus::Camera& taurus::CameraManager::GetCamera(uint8_t id) {
	return cameras[id];
}

void taurus::CameraManager::GetFrame(uint8_t id, cv::Mat& frame) {
	cameras[id].GetFrame(frame);
}

void taurus::CameraManager::Stop() {
	logging::info("Stopping camera manager...");

	for (uint8_t i = 0; i < ps3eyeCount; i++) {
		Camera camera = cameras[i];
		camera.Stop();
	}
}
