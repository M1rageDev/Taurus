#pragma once

#include <opencv2/opencv.hpp>
#include <ps3eye.h>

#include "modules/tracking/tracking_utils.h"

namespace taurus
{
	enum ExposureMode {
		Exposure_AUTO,
		Exposure_DARK
	};

	struct CameraCalibration {
		bool hasColor = false;
		std::unordered_map<std::string, tracking::HsvColorRange> colorDict;

		bool hasIntrinsic = false;
		cv::Mat K;
		cv::Mat distort;

		bool hasExtrinsic = false;
		cv::Mat T;
		cv::Mat world;

		bool hasProjection = false;
		cv::Mat P;
	};

	class Camera {
		public:
			Camera(uint8_t id, ps3eye::PS3EYECam::PS3EYERef ps3eyeRef, int width = 640, int height = 480, uint16_t fps = 60, ExposureMode exposureMode = Exposure_AUTO);

			void LoadData();

			void SetExposureMode(ExposureMode mode);
			uint8_t GetID() const;
			bool IsStarted() const;
			tracking::HsvColorRange GetHsvColorRange(std::string color);
			CameraCalibration GetCalibration() const;

			void GetFrame(cv::Mat& frame);
			cv::Mat InitFrameMat();

			void Stop();

		private:
			ps3eye::PS3EYECam::PS3EYERef ps3eyeRef;
			uint8_t id;
			bool isStarted;

			int width;
			int height;
			uint16_t fps;
			ExposureMode exposureMode;

			CameraCalibration calibration;
	};

	class CameraManager {
		public:
			static CameraManager* GetInstance();

			CameraManager();

			void SetupCameras(int width = 640, int height = 480, uint16_t fps = 60, taurus::ExposureMode exposureMode = taurus::ExposureMode::Exposure_AUTO);

			size_t GetCameraCount() const;
			Camera& GetCamera(uint8_t id);
			void GetFrame(uint8_t id, cv::Mat& frame);

			void Stop();

		private:
			static CameraManager* instance;

			std::vector<Camera> cameras;

			std::vector<ps3eye::PS3EYECam::PS3EYERef> ps3eyeReferences;
			size_t ps3eyeCount;
	};
}
