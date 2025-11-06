#include "modules/tracking/detector.h"

#include <vector>

#include "modules/utils.h"
#include "modules/logging.h"

void taurus::tracking::maskBrightBlobs(const cv::Mat& frame, cv::Mat& masked, cv::Mat& mask) {
	// convert image to grayscale, erode/dilate to reduce noise
	cv::cvtColor(frame, mask, cv::COLOR_BGR2GRAY);
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

	// do the thresholding
	cv::threshold(mask, mask, 30, 255, cv::THRESH_BINARY);
	cv::bitwise_and(frame, frame, masked, mask);
}

CONTOURLIST_T taurus::tracking::findContours(const cv::Mat& mask) {
	CONTOURLIST_T contours;
	cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	return contours;
}

cv::Rect taurus::tracking::fitNewRoi(cv::Point2f& globalCenter, int roiSize) {
	int halfRoiSize = roiSize / 2;

	int x1 = roundToInt(globalCenter.x) - halfRoiSize;
	int y1 = roundToInt(globalCenter.y) - halfRoiSize;
	return cv::Rect(x1, y1, roiSize, roiSize);
}

cv::Rect taurus::tracking::findLargestBlob(const cv::Mat& mask, cv::Point2f& circleCenter, float& circleRadius) {
	CONTOURLIST_T contours = findContours(mask);
	size_t contourCount = contours.size();

	// return early if no contours found
	if (contourCount < 1) return cv::Rect();

	// sort by area
	double largestArea = 0.0;
	int largestIndex = 0;
	for (int i = 0; i < contourCount; i++) {
		double area = cv::contourArea(contours[i]);

		if (area > largestArea) {
			largestArea = area;
			largestIndex = i;
		}
	}

	cv::minEnclosingCircle(contours[largestIndex], circleCenter, circleRadius);
	return cv::boundingRect(contours[largestIndex]);
}

// private helper function
static bool findSingleBallHsvMasked(const cv::Mat& hsvMaskedBright, const cv::Mat& mask, taurus::tracking::TrackedObject::PerCameraData& obj) {
	// optimization - check if there's anything in the image before going straight to color filtering
	CONTOURLIST_T contours = taurus::tracking::findContours(mask);
	size_t contourCount = contours.size();

	if (contourCount < 1) {
		obj.acquiredTracking = false;
		return false;
	}

	// find new contours after color filtering
	cv::Mat roiHsvTested;
	cv::inRange(hsvMaskedBright(obj.roi), obj.color.lower, obj.color.upper, roiHsvTested);
	contours = taurus::tracking::findContours(roiHsvTested);
	contourCount = contours.size();

	// sort by size, and discard bad contours
	double largestArea = 0.0;
	int largestIndex = -1;
	for (int i = 0; i < contourCount; i++) {
		CONTOUR_T contour = contours[i];
		cv::Rect bounds = cv::boundingRect(contour);
		double area = cv::contourArea(contour);

		// discard bad contours
		if (area < 8) continue;  // discard very small contours
		if (std::abs(bounds.width - bounds.height) > (bounds.height / 2)) continue;  // discard oddly shaped contours

		if (area > largestArea) {
			largestArea = area;
			largestIndex = i;
		}
	}

	// have we found anything?
	obj.acquiredTracking = (largestIndex != -1);
	if (obj.acquiredTracking) {
		cv::Point2f center;
		float radius;
		cv::minEnclosingCircle(contours[largestIndex], center, radius);
		cv::Rect bounds = cv::boundingRect(contours[largestIndex]);

		obj.inRoiCircleCenter = center;
		obj.circleRadius = radius;
		obj.inRoiBounds = bounds;
		obj.globalCircleCenter = taurus::tracking::roiPointToGlobal(center, obj.roi);
		obj.globalBounds = taurus::tracking::roiRectToGlobal(bounds, obj.roi);

		obj.roi = taurus::tracking::fitNewRoi(obj.globalCircleCenter);
		taurus::tracking::clampRoi(hsvMaskedBright, obj.roi);
	}

	return obj.acquiredTracking;
}

bool taurus::tracking::findSingleBall(const cv::Mat& frame, TrackedObject& obj, int cameraIndex) {
	cv::Mat hsvMaskedFrame, maskedFrame, mask;
	maskBrightBlobs(frame, maskedFrame, mask);
	cv::cvtColor(maskedFrame, hsvMaskedFrame, cv::COLOR_BGR2HSV);

	return findSingleBallHsvMasked(hsvMaskedFrame, mask, obj.perCameraData[cameraIndex]);
}

void taurus::tracking::findMultiBalls(const cv::Mat& frame, std::vector<TrackedObject>& objects, int cameraIndex) {
	cv::Mat hsvMaskedFrame, maskedFrame, mask;
	maskBrightBlobs(frame, maskedFrame, mask);
	cv::cvtColor(maskedFrame, hsvMaskedFrame, cv::COLOR_BGR2HSV);

	for (TrackedObject& obj : objects) {
		bool found = findSingleBallHsvMasked(hsvMaskedFrame, mask, obj.perCameraData[cameraIndex]);
	}
}
