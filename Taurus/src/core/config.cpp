#include "core/config.h"

#include "core/json_handler.h"
#include "core/logging.h"

taurus::TaurusConfig* taurus::TaurusConfig::instance = nullptr;

taurus::TaurusConfig* taurus::TaurusConfig::GetInstance() {
	return instance;
}

taurus::TaurusConfig::TaurusConfig() {
	taurus::TaurusConfig::instance = this;

	configPath = taurus::DATA_PATH / taurus::CONFIG_FILENAME;
	hasData = false;

	storage = TaurusConfigStorage();
}

void taurus::TaurusConfig::LoadConfig() {
	logging::info("Loading config...");

	hasData = readJson(configPath, &configData);
	ParseConfig();
}

taurus::TaurusConfigStorage* taurus::TaurusConfig::GetStorage() {
	return &storage;
}

template<typename T>
static std::optional<T> tryGetJsonValue(const json& j, const std::string& key) {
    if (!j.contains(key)) {
        return std::nullopt;
    }

    try {
        return std::optional<T>(j.at(key).get<T>());
    } catch (const json::exception&) {
        return std::nullopt;
    }
}

static std::optional<glm::vec3> tryGetJsonVec3(const json& j, const std::string& key) {
	if (!j.contains(key)) {
		return std::nullopt;
	}

	glm::vec3 vec;
	if (taurus::jsonReadVec3(j, key.c_str(), &vec)) {
		return std::optional<glm::vec3>(vec);
	} else {
		return std::nullopt;
	}
}

void taurus::TaurusConfig::ParseConfig() {
	if (!hasData) {
		logging::warning("Couldn't load config file!");
		return;
	}

	storage = TaurusConfigStorage();
	
	storage.leftControllerSerial = tryGetJsonValue<std::string>(configData, "left_controller_serial");
	storage.rightControllerSerial = tryGetJsonValue<std::string>(configData, "right_controller_serial");
	storage.leftControllerColor = tryGetJsonValue<std::string>(configData, "left_controller_color");
	storage.rightControllerColor = tryGetJsonValue<std::string>(configData, "right_controller_color");
	storage.commsEnabled = tryGetJsonValue<bool>(configData, "comms_enabled");
	storage.udpRecvPort = tryGetJsonValue<int>(configData, "udp_recv_port");
	storage.udpSendPort = tryGetJsonValue<int>(configData, "udp_send_port");
	storage.showPreview = tryGetJsonValue<bool>(configData, "show_preview");
	storage.annotatePreview = tryGetJsonValue<bool>(configData, "annotate_preview");
	storage.lowpassAlpha = tryGetJsonValue<float>(configData, "lowpass_alpha");
	storage.lowpassDistance = tryGetJsonValue<float>(configData, "lowpass_distance");

	logging::info("Successfully parsed config file.");
}
