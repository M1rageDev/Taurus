#pragma once

#include <optional>
#include <string>
#include <filesystem>

#include <glm/glm.hpp>

#include "core/json_handler.h"

namespace taurus
{
	struct TaurusConfigStorage {
		std::optional<std::string> leftControllerSerial;
		std::optional<std::string> rightControllerSerial;

		std::optional<std::string> leftControllerColor;
		std::optional<std::string> rightControllerColor;

		std::optional<bool> commsEnabled;
		std::optional<int> udpRecvPort;
		std::optional<int> udpSendPort;

		std::optional<bool> showPreview;
		std::optional<bool> annotatePreview;

		std::optional<glm::vec3> lowpassAlpha;
		std::optional<float> velocityDecay;
	};

	class TaurusConfig {
		public:
			static TaurusConfig* GetInstance();

			TaurusConfig();

			void LoadConfig();
			TaurusConfigStorage* GetStorage();
		private:
			static TaurusConfig* instance;

			void ParseConfig();

			fs::path configPath;

			bool hasData;
			json configData;

			TaurusConfigStorage storage;
	};
}
