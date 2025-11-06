#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/tracking/tracking_utils.h"
#include "modules/cameras.h"
#include "modules/psmove.h"

namespace taurus
{
	class ColorCalibrator {
		public:
			ColorCalibrator(uint8_t cameraId, std::string serial);

			void RunCalibration();
		private:
			void Calibrate(cv::Mat& frame);

			CameraManager* cameraManager;
			uint8_t cameraId;

			Controller* controller;
			std::string serial;

			bool isCalibrating = false;
			bool doneCalibration = false;

			std::unordered_map<std::string, tracking::HsvColorRange> colorDict;
	};

	class IntrinsicCalibrator {
		public:
			IntrinsicCalibrator(uint8_t cameraId, cv::Size chessboardSize = cv::Size(9, 6), float chessboardSquareMm = 27, int sampleCount = 30);

			void RunCalibration();
		private:
			void CaptureSample(const cv::Mat& frame);
			void Calibrate(const cv::Mat& frame);

			CameraManager* cameraManager;
			uint8_t cameraId;

			bool isCalibrating = false;
			bool doneCalibration = false;

			int minSamples;
			int sampleCount;
			cv::Size chessboardSize;
			float chessboardSquareMm;
			std::vector<std::vector<cv::Point2f>> samples;

			cv::Mat resultK;
			cv::Mat resultDistort;
	};

	class ExtrinsicCalibrator {
		public:
			ExtrinsicCalibrator(uint8_t cameraId0, uint8_t cameraId1, std::string serial, int sampleCount = 30);

			void RunCalibration();
		private:
			enum CalibStage {
				Stage_PnP,
				Stage_Correspondences
			};
			
			void CaptureSample(cv::Mat& frame0, cv::Mat& frame1);
			void Calibrate(cv::Mat& frame0, cv::Mat& frame1);

			CameraManager* cameraManager;
			uint8_t cameraId0;
			uint8_t cameraId1;

			Controller* controller;
			std::string serial;

			bool isCalibrating = false;
			bool doneCalibration = false;

			std::vector<std::string> samplePointNames;
			std::vector<cv::Point3f> samplePointObj;

			int minSamples, capturedSampleCount;
			std::vector<cv::Point2f> samples0;
			std::vector<cv::Point2f> samples1;

			cv::Mat resultWorld0;
			cv::Mat resultWorld1;
			cv::Mat resultT0;
			cv::Mat resultT1;
			cv::Mat resultF;
	};
}
