#include "core/calibration/camera_calibration.h"

#include "core/tracking/detector.h"
#include "core/json_handler.h"
#include "core/logging.h"

taurus::ColorCalibrator::ColorCalibrator(uint8_t cameraId, std::string serial) {
	this->cameraManager = CameraManager::GetInstance();
	this->cameraId = cameraId;

	this->controller = ControllerManager::GetInstance()->GetController(serial);
	this->serial = serial;

	this->colorDict = std::unordered_map<std::string, tracking::HsvColorRange>();
}

void taurus::ColorCalibrator::RunCalibration() {
	logging::info("Started ColorCalibration for cam %d", cameraId);

	cameraManager->GetCamera(cameraId).SetExposureMode(Exposure_DARK);
	controller->SetColor("yellow");

	cv::Mat frame = cameraManager->GetCamera(cameraId).InitFrameMat();

	isCalibrating = true;
	while (isCalibrating) {
		// read frame
		cameraManager->GetFrame(cameraId, frame);
		controller->Update();

		// show frame
		cv::imshow(std::format("ColorCalibration {}", cameraId), frame);

		// wait for key and calibrate
		int key = cv::waitKey(1);
		if (key == 'e') {
			cv::destroyAllWindows();
			Calibrate(frame);
		}
		else if (key == 27) {
			// escape
			isCalibrating = false;
			cv::destroyAllWindows();
		}
	}
}

void taurus::ColorCalibrator::Calibrate(cv::Mat& frame) {
	std::vector<std::string> colorNames = RGB_colorNames();
	for (std::string name : colorNames) {
		if (name == "off") continue;

		controller->SetColor(name);

		// wait 100ms while updating, after setting the color
		long t = 0;
		while (t < 100) {
			controller->Update();
			psmove_util_sleep_ms(1);
			t += 1;
		}

		// retrieve 5 frames to make sure the controller was found
		for (int i = 0; i < 5; i++) {
			cameraManager->GetFrame(cameraId, frame);
		}

		// find the ball
		cv::Mat maskedFrame, mask;
		cv::Point2f circleCenter;
		float circleRadius;

		tracking::maskBrightBlobs(frame, maskedFrame, mask);
		cv::Rect rect = tracking::findLargestBlob(mask, circleCenter, circleRadius);

		if (rect.width < 1) {
			logging::error("Couldn't find ball! Calibration failed");
			isCalibrating = false;
			return;
		}

		// find the square from which the colors will be sampled
		int centerX = rect.x + rect.width / 2;
		int centerY = rect.y + rect.height / 2;
		int measurementLength = rect.width / 2;
		int measurementHalf = measurementLength / 2;
		cv::Rect measurementRect = cv::Rect(centerX - measurementHalf, centerY - measurementHalf, measurementLength, measurementLength);

		// crop and find the mean hsv color
		cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
		cv::Mat croppedFrame = frame(measurementRect);
		cv::Scalar avg = cv::mean(croppedFrame);

		// get the hsv bounds
		cv::Scalar lowerBound = avg - cv::Scalar(30, 40, 45);
		cv::Scalar upperBound = avg + cv::Scalar(30, 40, 45);

		logging::info("%s LOWER:%f %f %f UPPER:%f %f %f", name.c_str(), lowerBound[0], lowerBound[1], lowerBound[2], upperBound[0], upperBound[1], upperBound[2]);

		tracking::HsvColorRange colorRange = { lowerBound, upperBound };
		colorDict.emplace(name, colorRange);
	}

	// report and save
	logging::info("Color calibration successful, recorded %d colors", colorNames.size());

	json data = createColorData(colorDict);
	saveJson(createColorPath(cameraId), data);
	isCalibrating = false;
	doneCalibration = true;
}

taurus::IntrinsicCalibrator::IntrinsicCalibrator(uint8_t cameraId, cv::Size chessboardSize, float chessboardSquareMm, int sampleCount) {
	this->cameraManager = CameraManager::GetInstance();
	this->cameraId = cameraId;

	this->minSamples = sampleCount;
	this->sampleCount = 0;
	this->chessboardSize = chessboardSize;
	this->chessboardSquareMm = chessboardSquareMm;
	this->samples = std::vector<std::vector<cv::Point2f>>();

	this->resultK = cv::Mat();
	this->resultDistort = cv::Mat();
}

void taurus::IntrinsicCalibrator::RunCalibration() {
	logging::info("Started IntrinsicCalibration for cam %d", cameraId);

	cv::Mat frame = cameraManager->GetCamera(cameraId).InitFrameMat();

	isCalibrating = true;
	while (isCalibrating) {
		// read frame
		cameraManager->GetFrame(cameraId, frame);

		cv::Mat gray;
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		// show frame
		cv::imshow(std::format("IntrinsicCalibration {}", cameraId), gray);

		// wait for key and capture sample
		int key = cv::waitKey(1);
		if (key == 'e') {
			CaptureSample(gray);
		}
		else if (key == 27) {
			// escape
			isCalibrating = false;
			cv::destroyAllWindows();
		}
	}
}

void taurus::IntrinsicCalibrator::CaptureSample(const cv::Mat& frame) {
	// capture corners and record
	std::vector<cv::Point2f> corners;
	bool ret = tracking::findRefineChessboardCorners(frame, chessboardSize, corners);
	if (ret) {
		samples.push_back(corners);
		logging::info("Chessboard sample %d successful.", sampleCount);

		// handle min samplecount
		sampleCount++;
		if (sampleCount >= minSamples) {
			Calibrate(frame);
		}
	}
	else {
		logging::warning("Chessboard sample failed, try again!");
	}
}

void taurus::IntrinsicCalibrator::Calibrate(const cv::Mat& frame) {
	cv::destroyAllWindows();
	logging::info("Trying to calibrate...");

	// create obj points
	std::vector<std::vector<cv::Point3f>> objPoints;
	for (int i = 0; i < sampleCount; i++) {
		std::vector<cv::Point3f> chessboardObj;
		for (int y = 0; y < chessboardSize.height; y++) {
			for (int x = 0; x < chessboardSize.width; x++) {
				chessboardObj.push_back(cv::Point3f(x * chessboardSquareMm, y * chessboardSquareMm, 0));
			}
		}
		objPoints.push_back(chessboardObj);
	}

	// calibrate
	cv::Mat rvecs, tvecs;
	double rms = cv::calibrateCamera(objPoints, samples, frame.size(), resultK, resultDistort, rvecs, tvecs);
	bool ok = rms < 10.0;

	if (ok) {
		logging::info("Calibration successful!");
		logging::info("RMS: %f", rms);
		logging::info("Fx: %f", resultK.at<double>(0, 0));
		logging::info("Fy: %f", resultK.at<double>(1, 1));
		logging::info("Cx: %f", resultK.at<double>(0, 2));
		logging::info("Cy: %f", resultK.at<double>(1, 2));

		json data = createIntrinsicData(resultK, resultDistort);
		saveJson(createIntrinsicPath(cameraId), data);

		isCalibrating = false;
		doneCalibration = true;
	}
	else {
		isCalibrating = false;
		doneCalibration = false;
	}
}

taurus::ExtrinsicCalibrator::ExtrinsicCalibrator(uint8_t cameraId0, uint8_t cameraId1, std::string serial, int sampleCount) {
	this->cameraManager = CameraManager::GetInstance();
	this->cameraId0 = cameraId0;
	this->cameraId1 = cameraId1;

	this->controller = ControllerManager::GetInstance()->GetController(serial);
	this->serial = serial;

	this->minSamples = sampleCount;
	this->capturedSampleCount = 0;

	// need to calibrate intrinsics first
	for (int i = 0; i < 2; i++) {
		if (!this->cameraManager->GetCamera(i).GetCalibration().hasIntrinsic) {
			logging::error("Camera %d has no intrinsic calibration! Extrinsic calibration cannot proceed.", i);
			exit(-1);
			return;
		}
	}

	this->samplePointNames = {
		"center",
		"x+z+",
		"x+z-",
		"x-z+",
		"x-z-",
	};
	this->samplePointObj = {
		// A4 paper is 29.8 x 21.0 cm
		// Half size is 14.9 x 10.5 cm
		cv::Point3f(0.f,    0.f, 0.f),
		cv::Point3f(14.9f,  0.f, 10.5f),
		cv::Point3f(14.9f,  0.f, -10.5f),
		cv::Point3f(-14.9f, 0.f, 10.5f),
		cv::Point3f(-14.9f, 0.f, -10.5f),
	};
}

void taurus::ExtrinsicCalibrator::RunCalibration() {
	logging::info("Started ExtrinsicCalibration for cams %d %d", cameraId0, cameraId1);
	logging::info("Move the controller to position [%s]", samplePointNames[0].c_str());

	cv::Mat frame0 = cameraManager->GetCamera(cameraId0).InitFrameMat();
	cv::Mat frame1 = cameraManager->GetCamera(cameraId1).InitFrameMat();

	controller->SetColor("cyan");
	cameraManager->GetCamera(cameraId0).SetExposureMode(Exposure_DARK);
	cameraManager->GetCamera(cameraId1).SetExposureMode(Exposure_DARK);

	cv::Mat maskedFrame, mask;
	cv::Point2f circleCenter0;
	float circleRadius0;

	isCalibrating = true;
	while (isCalibrating) {
		controller->Update();

		// read frames
		cameraManager->GetFrame(cameraId0, frame0);
		cameraManager->GetFrame(cameraId1, frame1);

		if (resultF.empty() == false) {
			// draw epilines if we have a result fundamental matrix
			tracking::maskBrightBlobs(frame0, maskedFrame, mask);
			cv::Rect rect0 = tracking::findLargestBlob(mask, circleCenter0, circleRadius0);

			std::vector<cv::Point3f> lines;
			std::vector<cv::Point2f> points = { circleCenter0 };
			cv::computeCorrespondEpilines(points, 1, resultF, lines);
			tracking::drawEpilines(frame1, lines, cv::Scalar(0, 255, 0));
		}

		// show frames
		cv::imshow(std::format("ExtrinsicCalibration {}", cameraId0), frame0);
		cv::imshow(std::format("ExtrinsicCalibration {}", cameraId1), frame1);

		// wait for key and capture sample
		int key = cv::waitKey(1);
		if (key == 'e') {
			CaptureSample(frame0, frame1);
		}
		else if (key == 27) {
			// escape
			isCalibrating = false;
			cv::destroyAllWindows();
		}
	}
}

void taurus::ExtrinsicCalibrator::CaptureSample(cv::Mat& frame0, cv::Mat& frame1) {
	cv::Mat maskedFrame, mask;

	// find brightest blob in frame0 and frame1
	cv::Point2f circleCenter0, circleCenter1;
	float circleRadius0, circleRadius1;
	tracking::maskBrightBlobs(frame0, maskedFrame, mask);
	cv::Rect rect0 = tracking::findLargestBlob(mask, circleCenter0, circleRadius0);
	tracking::maskBrightBlobs(frame1, maskedFrame, mask);
	cv::Rect rect1 = tracking::findLargestBlob(mask, circleCenter1, circleRadius1);

	// capture sample
	samples0.push_back(circleCenter0);
	samples1.push_back(circleCenter1);
	capturedSampleCount++;
	logging::info("Captured sample %d", capturedSampleCount);

	// if we haven't captured all points yet, prompt for next
	// if we have then calibrate
	if (capturedSampleCount < samplePointNames.size()) {
		logging::info("Move the controller to position [%s]", samplePointNames[capturedSampleCount].c_str());
	}
	else {
		cv::destroyAllWindows();
		Calibrate(frame0, frame1);
	}
}

static bool ExtrinsicPnPSingle(std::vector<cv::Point3f> objPoints, std::vector<cv::Point2f> imgPoints, cv::Mat& K, cv::Mat& distort, cv::Mat& rvec, cv::Mat& tvec, cv::Mat& T, cv::Mat& P) {
	bool ret = cv::solvePnP(objPoints, imgPoints, K, distort, rvec, tvec);

	if (ret) {
		cv::Mat R;
		cv::Rodrigues(rvec, R);

		// get the inverse of the marker transformation, to get the camera pose
		cv::Mat R_inv = R.t();
		cv::Mat t_inv = -R_inv * tvec;

		// create T matrix
		cv::hconcat(R_inv, t_inv, T);

		// calculate projection matrix
		P = K * T;

		return true;
	}
	else {
		return false;
	}
}

void taurus::ExtrinsicCalibrator::Calibrate(cv::Mat& frame0, cv::Mat& frame1) {
	cv::Mat marker_rvec0, marker_t0;
	cv::Mat marker_rvec1, marker_t1;
	cv::Mat T0, P0;
	cv::Mat T1, P1;

	// solve pnp on the mat for both cams
	CameraCalibration calib0 = cameraManager->GetCamera(cameraId0).GetCalibration();
	CameraCalibration calib1 = cameraManager->GetCamera(cameraId1).GetCalibration();
	bool ret0 = ExtrinsicPnPSingle(samplePointObj, samples0, calib0.K, calib0.distort, marker_rvec0, marker_t0, T0, P0);
	bool ret1 = ExtrinsicPnPSingle(samplePointObj, samples1, calib1.K, calib1.distort, marker_rvec1, marker_t1, T1, P1);

	if ((!ret0) || (!ret1)) {
		logging::error("Extrinsic calibration failed during PnP!");
		isCalibrating = false;
		doneCalibration = false;
		return;
	}

	// find relative transform
	cv::Mat ctw0, ctw1;
	tracking::make4x4Matrix(T0, ctw0);  // cam0 -> world
	tracking::make4x4Matrix(T1, ctw1);  // cam1 -> world
	cv::Mat c0c1 = ctw1.inv() * ctw0;  // cam0 -> cam1

	// find new relative transform
	T0 = cv::Mat::eye(3, 4, CV_64F);
	T1 = c0c1(cv::Rect(0, 0, 4, 3));
	P0 = calib0.K * T0;
	P1 = calib1.K * T1;

	// find F
	cv::Mat F;
	tracking::fundamentalFromProjections(P0, P1, F);

	// store results
	resultWorld0 = ctw0;  // 0 -> world
	resultWorld1 = ctw1;  // 1 -> world
	resultT0 = T0;  // identity (origin is here)
	resultT1 = T1;  // 0 -> 1
	resultF = F;  // fundamental

	// save results to files
	json data0 = createExtrinsicData(resultT0, resultWorld0);
	json data1 = createExtrinsicData(resultT1, resultWorld1);
	saveJson(createExtrinsicPath(cameraId0), data0);
	saveJson(createExtrinsicPath(cameraId1), data1);

	// draw for verification
	cameraManager->GetCamera(cameraId0).SetExposureMode(Exposure_AUTO);
	cameraManager->GetCamera(cameraId1).SetExposureMode(Exposure_AUTO);

	for (int i = 0; i < 5; i++) {
		// capture 5 frames to stabilize exposure
		cameraManager->GetFrame(cameraId0, frame0);
		cameraManager->GetFrame(cameraId1, frame1);
	}

	std::vector<cv::Point3f> lines;
	cv::computeCorrespondEpilines(samples0, 1, resultF, lines);
	tracking::drawEpilines(frame1, lines, cv::Scalar(0, 255, 0));

	cv::drawFrameAxes(frame0, calib0.K, calib0.distort, marker_rvec0, marker_t0, 20);
	cv::drawFrameAxes(frame1, calib1.K, calib1.distort, marker_rvec1, marker_t1, 20);

	cv::imshow(std::format("ExtrinsicCalibration {}", cameraId0), frame0);
	cv::imshow(std::format("ExtrinsicCalibration {}", cameraId1), frame1);
	logging::info("Verify the markings on the images look correct, then press any key to continue...");
	cv::waitKey(0);

	cameraManager->GetCamera(cameraId0).SetExposureMode(Exposure_DARK);
	cameraManager->GetCamera(cameraId1).SetExposureMode(Exposure_DARK);

	logging::info("Verify that the green line matches the corresponding images, then press any key to continue...");
}
