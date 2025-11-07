#pragma once

#include <string>
#include <filesystem>
#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

#include "core/tracking/tracking_utils.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace taurus
{
	const fs::path DATA_PATH = "../Taurus_Data/";

	const fs::path CAMERAS_SUBPATH = "Cameras/";
	const fs::path CONTROLLERS_SUBPATH = "Controllers/";

	const std::string CONFIG_FILENAME = "config.json";

	std::string serialToFilename(std::string serial);
	std::string filenameToSerial(std::string filenameSerial);

	fs::path createPath(fs::path subpath, std::string name);

	void saveJson(fs::path path, json data);
	bool readJson(fs::path path, json* data);

	// datatype helpers
	void jsonWriteVec3(const char* key, glm::vec3 vec, json* data);
	bool jsonReadVec3(json data, const char* key, glm::vec3* vec);
	void jsonWriteCvScalar(const char* key, cv::Scalar scalar, json* data);
	bool jsonReadCvScalar(json data, const char* key, cv::Scalar* scalar);
	void jsonWriteCvMat(const char* key, const cv::Mat& mat, json* data);
	bool jsonReadCvMat(json data, const char* key, cv::Mat* mat);

	// path creation functions
	fs::path createGyroPath(std::string serial);
	fs::path createAccelPath(std::string serial);
	fs::path createColorPath(uint8_t cameraId);
	fs::path createIntrinsicPath(uint8_t cameraId);
	fs::path createExtrinsicPath(uint8_t cameraId);

	// data creation functions
	json createGyroData(glm::vec3 offsets);
	json createAccelData(glm::vec3 bias, glm::vec3 scale);
	json createColorData(std::unordered_map<std::string, tracking::HsvColorRange> colorDict);
	json createIntrinsicData(const cv::Mat& K, const cv::Mat& distort);
	json createExtrinsicData(const cv::Mat& T, const cv::Mat& world);
}
