#pragma once

#include <opencv2/opencv.hpp>

#include "modules/tracking/tracking_utils.h"

#define CONTOUR_T std::vector<cv::Point>
#define CONTOURLIST_T std::vector<CONTOUR_T>

namespace taurus::tracking
{
	void maskBrightBlobs(const cv::Mat& frame, cv::Mat& masked, cv::Mat& mask);
	CONTOURLIST_T findContours(const cv::Mat& mask);
	cv::Rect fitNewRoi(cv::Point2f& globalCenter, int roiSize=240);

	cv::Rect findLargestBlob(const cv::Mat& mask, cv::Point2f& circleCenter, float& circleRadius);
	bool findSingleBall(const cv::Mat& frame, TrackedObject& obj, int cameraIndex);
	void findMultiBalls(const cv::Mat& frame, std::vector<TrackedObject>& objects, int cameraIndex);
}
