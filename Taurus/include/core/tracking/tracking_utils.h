#pragma once

#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>

#include "core/filter/filter_utils.h"

namespace taurus::tracking
{
	struct HsvColorRange {
		cv::Scalar lower = {};
		cv::Scalar upper = {};
	};

	struct TrackedObject {
		struct PerCameraData {
			cv::Rect roi = {};
			HsvColorRange color = {};

			bool acquiredTracking = false;

			cv::Point2f inRoiCircleCenter = {};
			cv::Rect inRoiBounds = {};
			float circleRadius = 0.f;

			cv::Point2f globalCircleCenter = {};
			cv::Rect globalBounds = {};
		};

		std::vector<PerCameraData> perCameraData;

		// 3d tracking
		cv::Point3f triangulatedPosition = {};
		bool acquired3DPosition = false;
		bool newOpticalDataReady = false;

		// after world transform
		glm::vec3 worldPosition = {};
		glm::vec3 previousWorldPosition = {};

		// optical prediction
		glm::vec3 opticalVelocity = {};

		// filtering
		glm::vec3 preFilteredPosition = {};
		glm::vec3 filteredPosition = {};
		glm::vec3 previousFilteredPosition = {};
		glm::vec3 filteredPositionM = {};

		filter::KinematicObject kinematic = {};
	};

	cv::Point2f rectCenter(cv::Rect& rect);

	cv::Point2f roiPointToGlobal(cv::Point2f& p, cv::Rect& roi);
	cv::Rect roiRectToGlobal(cv::Rect& rect, cv::Rect& roi);

	cv::Rect createFrameRoi(const cv::Mat& frame);
	void clampRoi(const cv::Mat& frame, cv::Rect& roi);
	void increaseRoiSize(cv::Rect& roi, int sizeChange);

	bool findRefineChessboardCorners(const cv::Mat& frame, cv::Size& patternSize, std::vector<cv::Point2f>& corners);

	void fundamentalFromProjections(const cv::Mat& P1, const cv::Mat& P2, cv::Mat& F);
	void make4x4Matrix(const cv::Mat& mat3x4, cv::Mat& mat4x4);

	void drawEpilines(cv::Mat& frame, std::vector<cv::Point3f>& lines, cv::Scalar color);

	cv::Point2f undistort(const cv::Point2f& point, const cv::Mat& K, const cv::Mat& distort);
	cv::Point3f triangulate(const cv::Mat& P1, const cv::Mat& P2, TrackedObject::PerCameraData& cam0Data, TrackedObject::PerCameraData& cam1Data);
	cv::Point3f transform(const cv::Mat& mat4x4, const cv::Point3f& point);

	glm::vec3 cvPoint3fToGlmVec3(const cv::Point3f& point);
	cv::Point3f glmVec3ToCvPoint3f(const glm::vec3& vec);

	glm::mat3 cvMat3ToGlmMat3(const cv::Mat& mat);

	glm::vec3 offsetPosition(const glm::vec3& position, const glm::quat& rotation, float length);

	std::pair<cv::Mat, cv::Point3f> decomposeTransform(const cv::Mat& T);
}
