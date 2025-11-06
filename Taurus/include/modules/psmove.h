#pragma once

#include <unordered_map>
#include <string>
#include <thread>
#include <memory>

#include "psmoveapi/psmove.h"

#include "modules/madgwick.h"
#include "modules/tracking/tracking_utils.h"

namespace taurus
{
	struct RGB_char {
		unsigned char r = 0;
		unsigned char g = 0;
		unsigned char b = 0;
	};

	inline const RGB_char RGB_OFF{ 0, 0, 0 };

	const RGB_char RGB_fromName(std::string_view colorName);
	const std::vector<std::string> RGB_colorNames();

	struct RumbleState {
		bool active;

		int durationMs;
		float strength;

		long startTimeMs;
		int elapsedTimeMs;
	};

	struct ImuCalibration {
		bool hasGyro;
		glm::vec3 gyroOffsets;
	};

	class Controller {
		public:
			Controller(std::string serial = "");

			PSMove* GetMoveHandle();

			void LoadData();

			void Connect(PSMove* move);
			bool Update();  // returns if poll successful (new data)
			void Disconnect();

			void StartUpdateThread();
			void StopUpdateThread();
			
			void SetColor(std::string_view colorName);
			void SetColorRaw(RGB_char color);
			std::string GetColorName();

			void DoRumble(float durationSeconds, float strength);

			bool IsConnected() const;

			float GetBattery01() const;
			bool IsCharging() const;

			float GetTrigger01() const;
			bool IsButtonPressed(PSMove_Button button) const;

			glm::vec3 GetGyro() const;
			glm::vec3 GetAccel() const;

			MadgwickState* GetAhrsState();
			glm::quat GetVrQuat() const;
			void ResetAhrs();

		private:
			void HandlePoll();
			void HandleBattery(long now);
			void HandleInput(long now);
			void HandleAhrs(long now);

			void HandleRumble(long now);

			void UpdateThreadFunction();

			// update thread
			std::atomic<bool> updateThreadRunning;
			std::thread updateThread;

			// basic info
			PSMove* moveHandle;
			PSMove_Connection_Type connectionType;
			std::string serial;
			bool connected;

			// color info
			std::string colorName;
			RGB_char color;
			bool colorDirty;

			// haptics
			RumbleState rumbleState;

			// updating controller state
			long lastControllerWrite;
			bool controllerWriteUrgent;

			// battery
			PSMove_Battery_Level batteryState;
			float battery01;
			bool isCharging;

			// input
			unsigned char trigger;
			float trigger01;

			unsigned int buttonBitfield;

			// IMU and AHRS
			ImuCalibration imuCalibration;
			glm::vec3 gVec;
			glm::vec3 aVec;

			glm::quat initialQuat;
			MadgwickState ahrsState;
			glm::quat vrSpaceQuat;
	};

	class ControllerManager {
		public:
			static ControllerManager* GetInstance();

			ControllerManager();
			ControllerManager(std::vector<std::string> expectedSerials);

			static void InitPSMoveAPI();

			void StartUpdateThreads();
			void StopUpdateThreads();

			void ConnectControllers();
			void UpdateControllers();
			void DisconnectControllers();

			Controller* GetController(std::string serial);
			std::vector<std::string> GetAllocatedSerials() const;
			std::vector<std::string> GetConnectedSerials() const;

		private:
			static ControllerManager* instance;

			std::unordered_map<std::string, std::unique_ptr<Controller>> controllers;
			std::vector<std::string> allocatedSerials;
			std::vector<std::string> connectedSerials;
	};
}
