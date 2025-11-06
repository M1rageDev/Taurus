#include "modules/tracking/tracking_utils.h"

#include <glm/gtc/quaternion.hpp>

cv::Point2f taurus::tracking::rectCenter(cv::Rect& rect) {
	return cv::Point2f(
		rect.x + static_cast<float>(rect.width) / 2.f,
		rect.y + static_cast<float>(rect.height) / 2.f
	);
}

cv::Point2f taurus::tracking::roiPointToGlobal(cv::Point2f& p, cv::Rect& roi) {
	return cv::Point2f(p.x + roi.x, p.y + roi.y);
}

cv::Rect taurus::tracking::roiRectToGlobal(cv::Rect& rect, cv::Rect& roi) {
	return cv::Rect(rect.x + roi.x, rect.y + roi.y, rect.width, rect.height);
}

cv::Rect taurus::tracking::createFrameRoi(const cv::Mat& frame) {
	return cv::Rect(0, 0, frame.cols, frame.rows);
}

void taurus::tracking::clampRoi(const cv::Mat& frame, cv::Rect& roi) {
	int clampedX = std::clamp(roi.x, 0, frame.cols - 1);
	int clampedY = std::clamp(roi.y, 0, frame.rows - 1);
	int maxWidth = frame.cols - clampedX;
	int maxHeight = frame.rows - clampedY;

	roi.x = clampedX;
	roi.y = clampedY;
	roi.width = std::clamp(roi.width, 0, maxWidth);
	roi.height = std::clamp(roi.height, 0, maxHeight);
}

void taurus::tracking::increaseRoiSize(cv::Rect& roi, int sizeChange) {
	int halfSizeChange = sizeChange / 2;

	roi.x = roi.x - halfSizeChange;
	roi.y = roi.y - halfSizeChange;
	roi.width = roi.width + sizeChange;
	roi.height = roi.height + sizeChange;
}

bool taurus::tracking::findRefineChessboardCorners(const cv::Mat& frame, cv::Size& patternSize, std::vector<cv::Point2f>& corners) {
	bool ret = cv::findChessboardCorners(frame, patternSize, corners);
	if (ret) {
		cv::cornerSubPix(frame, corners, cv::Size(11, 11), cv::Size(-1, -1), 
			cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001)
		);

		return true;
	}
	else {
		return false;
	}
}

// https://github.com/opencv/opencv_contrib/blob/4.x/modules/sfm/src/fundamental.cpp#L109
void taurus::tracking::fundamentalFromProjections(const cv::Mat& P1, const cv::Mat& P2, cv::Mat& F) {
	cv::Mat X[3];
	vconcat(P1.row(1), P1.row(2), X[0]);
	vconcat(P1.row(2), P1.row(0), X[1]);
	vconcat(P1.row(0), P1.row(1), X[2]);

	cv::Mat Y[3];
	vconcat(P2.row(1), P2.row(2), Y[0]);
	vconcat(P2.row(2), P2.row(0), Y[1]);
	vconcat(P2.row(0), P2.row(1), Y[2]);

	F = cv::Mat::zeros(3, 3, CV_64F);

	cv::Mat XY;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			vconcat(X[j], Y[i], XY);

			F.at<double>(i, j) = cv::determinant(XY);
		}
}

void taurus::tracking::make4x4Matrix(const cv::Mat& mat3x4, cv::Mat& mat4x4) {
	mat4x4 = cv::Mat::eye(4, 4, CV_64F);

	cv::Mat mat_64;
	mat3x4.convertTo(mat_64, CV_64F);
	mat_64.copyTo(mat4x4(cv::Range(0, 3), cv::Range(0, 4)));
}

void taurus::tracking::drawEpilines(cv::Mat& frame, std::vector<cv::Point3f>& lines, cv::Scalar color) {
	float c = static_cast<float>(frame.cols);

	for (cv::Point3f r : lines) {
		cv::Point p0 = cv::Point(0, -r.z / r.y);
		cv::Point p1 = cv::Point(c, -(r.z + r.x * c) / r.y);

		cv::line(frame, p0, p1, color);
	}
}

cv::Point2f taurus::tracking::undistort(const cv::Point2f& point, const cv::Mat& K, const cv::Mat& distort) {
	static std::vector<cv::Point2f> points = { point };
	static std::vector<cv::Point2f> undistorted;

	cv::undistortPoints(points, undistorted, K, distort);
	return undistorted[0];
}

cv::Point3f taurus::tracking::triangulate(const cv::Mat& P0, const cv::Mat& P1, TrackedObject::PerCameraData& cam0Data, TrackedObject::PerCameraData& cam1Data) {
	static cv::Mat points0 = cv::Mat(2, 1, CV_64FC1);
	static cv::Mat points1 = cv::Mat(2, 1, CV_64FC1);

	points0.at<double>(0, 0) = static_cast<double>(cam0Data.globalCircleCenter.x);
	points0.at<double>(1, 0) = static_cast<double>(cam0Data.globalCircleCenter.y);
	points1.at<double>(0, 0) = static_cast<double>(cam1Data.globalCircleCenter.x);
	points1.at<double>(1, 0) = static_cast<double>(cam1Data.globalCircleCenter.y);

	static cv::Mat homogeneous = cv::Mat(4, 1, CV_64FC1);
	cv::triangulatePoints(P0, P1, points0, points1, homogeneous);

	double w = homogeneous.at<double>(3, 0);
	return cv::Point3f(homogeneous.at<double>(0, 0) / w, homogeneous.at<double>(1, 0) / w, homogeneous.at<double>(2, 0) / w);
}

cv::Point3f taurus::tracking::transform(const cv::Mat& mat4x4, const cv::Point3f& point) {
	// multiply
	static cv::Mat hom = cv::Mat(4, 1, CV_64FC1);
	hom.at<double>(0, 0) = static_cast<double>(point.x);
	hom.at<double>(1, 0) = static_cast<double>(point.y);
	hom.at<double>(2, 0) = static_cast<double>(point.z);
	hom.at<double>(3, 0) = 1.0;
	cv::Mat out = mat4x4 * hom;

	// write back
	cv::Point3f transformed;
	double w = out.at<double>(3, 0);
	transformed.x = out.at<double>(0, 0) / w;
	transformed.y = out.at<double>(1, 0) / w;
	transformed.z = out.at<double>(2, 0) / w;

	return transformed;
}

glm::vec3 taurus::tracking::cvPoint3fToGlmVec3(const cv::Point3f& point) {
	return glm::vec3(point.x, point.y, point.z);
}

cv::Point3f taurus::tracking::glmVec3ToCvPoint3f(const glm::vec3& vec) {
	return cv::Point3f(vec.x, vec.y, vec.z);
}

glm::mat3 taurus::tracking::cvMat3ToGlmMat3(const cv::Mat& mat) {
	if (mat.empty() || mat.rows != 3 || mat.cols != 3) {
		return glm::mat3(1.0f);
	}

	glm::mat3 out = glm::mat3(0.0f);
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			double v = mat.at<double>(r, c);

			// glm is column-major: out[col][row]
			out[c][r] = static_cast<float>(v);
		}
	}

	return out;
}

glm::vec3 taurus::tracking::offsetPosition(const glm::vec3& position, const glm::quat& rotation, float length) {
	static const glm::vec3 axis = glm::vec3(0.f, 0.f, 1.f);
	return position + (rotation * axis) * length;
}

std::pair<cv::Mat, cv::Point3f> taurus::tracking::decomposeTransform(const cv::Mat& T) {
	if (T.empty()) {
		cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
		cv::Point3f t = cv::Point3f(0.f, 0.f, 0.f);
		return { R, t };
	}

	// extract R
	cv::Mat R = T(cv::Range(0, 3), cv::Range(0, 3)).clone();

	// extract t
	cv::Point3f t = cv::Point3f(0.f, 0.f, 0.f);
	double tx = T.at<double>(0, 3);
	double ty = T.at<double>(1, 3);
	double tz = T.at<double>(2, 3);
	t = cv::Point3f(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));

	return { R, t };
}
