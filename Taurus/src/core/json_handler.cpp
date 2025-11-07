#include "core/json_handler.h"

#include <algorithm>  // std::replace
#include <fstream>  // ofstream, ifstream

#include "core/logging.h"

std::string taurus::serialToFilename(std::string serial) {
	std::string newStr = std::string(serial);
	std::replace(newStr.begin(), newStr.end(), ':', '-');
	return newStr;
}

std::string taurus::filenameToSerial(std::string filenameSerial) {
	std::string newStr = std::string(filenameSerial);
	std::replace(newStr.begin(), newStr.end(), '-', ':');
	return newStr;
}

fs::path taurus::createPath(fs::path subpath, std::string name) {
	return DATA_PATH / subpath / (name + ".json");
}

void taurus::saveJson(fs::path path, json data) {
	logging::info("Saving data to file %s", path.generic_string().c_str());

	// check if data has formatting key
	if (!data.contains("format")) {
		logging::error("Data has no formatting key!");
		return;
	}
	
	// make sure dir exists
	fs::path parentPath = path.parent_path();
	if (!fs::is_directory(parentPath)) {
		fs::create_directories(parentPath);
	}

	// try saving
	try {
		std::ofstream stream = std::ofstream(path);
		stream << std::setw(4) << data << std::endl;
		stream.close();

		logging::info("Successfully saved data");
	}
	catch (std::exception e) {
		logging::error("Error while saving data! %s", e.what());
	}
}

bool taurus::readJson(fs::path path, json* data) {
	logging::info("Reading data from file %s", path.generic_string().c_str());

	// make sure file exists
	if (!fs::exists(path)) {
		logging::error("File doesn't exist!");
		return false;
	}

	// try reading
	try {
		std::ifstream stream = std::ifstream(path);
		stream >> *data;
		stream.close();

		logging::info("Successfully read data");

		return true;
	}
	catch (std::exception e) {
		logging::error("Error while reading data! %s", e.what());

		return false;
	}
}

void taurus::jsonWriteVec3(const char* key, glm::vec3 vec, json* data) {
	(*data)[key] = {
		{ "x", vec.x },
		{ "y", vec.y },
		{ "z", vec.z }
	};
}

bool taurus::jsonReadVec3(json data, const char* key, glm::vec3* vec) {
	if (!data.contains(key)) {
		return false;
	}

	vec->x = data[key]["x"];
	vec->y = data[key]["y"];
	vec->z = data[key]["z"];

	return true;
}

void taurus::jsonWriteCvScalar(const char* key, cv::Scalar scalar, json* data) {
	(*data)[key] = {
		{ "x", scalar[0] },
		{ "y", scalar[1] },
		{ "z", scalar[2] },
		{ "w", scalar[3] }
	};
}

bool taurus::jsonReadCvScalar(json data, const char* key, cv::Scalar* scalar) {
	if (!data.contains(key)) {
		return false;
	}

	(*scalar)[0] = data[key]["x"];
	(*scalar)[1] = data[key]["y"];
	(*scalar)[2] = data[key]["z"];
	(*scalar)[3] = data[key]["w"];

	return true;
}

void taurus::jsonWriteCvMat(const char* key, const cv::Mat& mat, json* data) {
	int rows = mat.rows;
	int cols = mat.cols;
	int matType = mat.type();
	size_t elemSize = mat.elemSize();
	size_t dataLength = rows * cols * elemSize;

	(*data)[key] = {
		{ "rows", rows },
		{ "cols", cols },
		{ "type", matType },
		{ "elemSize", elemSize },
		{ "dataLength", dataLength }
	};

	for (int i = 0; i < dataLength; i++) {
		(*data)[key]["data"][i] = mat.data[i];
	}
}

bool taurus::jsonReadCvMat(json data, const char* key, cv::Mat* mat) {
	if (!data.contains(key)) {
		return false;
	}

	int rows = data[key]["rows"];
	int cols = data[key]["cols"];
	int matType = data[key]["type"];
	size_t elemSize = data[key]["elemSize"];
	size_t dataLength = data[key]["dataLength"];

	(*mat) = cv::Mat(rows, cols, matType);
	for (int i = 0; i < dataLength; i++) {
		(*mat).data[i] = static_cast<uchar>(data[key]["data"][i]);
	}

	return true;
}

fs::path taurus::createGyroPath(std::string serial) {
	return createPath(CONTROLLERS_SUBPATH / serialToFilename(serial), "gyro");
}

fs::path taurus::createAccelPath(std::string serial) {
	return createPath(CONTROLLERS_SUBPATH / serialToFilename(serial), "accel");
}

fs::path taurus::createColorPath(uint8_t cameraId) {
	return createPath(CAMERAS_SUBPATH / std::to_string(cameraId), "color");
}

fs::path taurus::createIntrinsicPath(uint8_t cameraId) {
	return createPath(CAMERAS_SUBPATH / std::to_string(cameraId), "intrinsic");
}

fs::path taurus::createExtrinsicPath(uint8_t cameraId) {
	return createPath(CAMERAS_SUBPATH / std::to_string(cameraId), "extrinsic");
}

json taurus::createGyroData(glm::vec3 offsets) {
	json data;
	data["format"] = "gyro";
	jsonWriteVec3("offsets", offsets, &data);
	return data;
}

json taurus::createAccelData(glm::vec3 bias, glm::vec3 scale) {
	json data;
	data["format"] = "accel";
	jsonWriteVec3("bias", bias, &data);
	jsonWriteVec3("scale", scale, &data);
	return data;
}

json taurus::createColorData(std::unordered_map<std::string, tracking::HsvColorRange> colorDict) {
	json data;
	data["format"] = "color";

	auto array = json::array();
	for (auto& [name, color] : colorDict) {
		auto colorObj = json::object();
		colorObj["name"] = name;
		jsonWriteCvScalar("lower", color.lower, &colorObj);
		jsonWriteCvScalar("upper", color.upper, &colorObj);
		array.push_back(colorObj);
	}
	data["colors"] = array;

	return data;
}

json taurus::createIntrinsicData(const cv::Mat& K, const cv::Mat& distort) {
	json data;
	data["format"] = "intrinsic";
	jsonWriteCvMat("K", K, &data);
	jsonWriteCvMat("distort", distort, &data);
	return data;
}

json taurus::createExtrinsicData(const cv::Mat& T, const cv::Mat& world) {
	json data;
	data["format"] = "extrinsic";
	jsonWriteCvMat("T", T, &data);
	jsonWriteCvMat("world", world, &data);
	return data;
}
