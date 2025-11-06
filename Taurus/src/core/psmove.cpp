#include "core/psmove.h"

#include <iterator>

#include "core/utils.h"
#include "core/logging.h"
#include "core/json_handler.h"

// readonly static color map, created at compile-time
static constexpr std::pair<std::string_view, taurus::RGB_char> colorTable[] = {
	{"cyan", { 0, 255, 255 }},
	{"purple", { 255, 0, 255 }},
	{"red", { 255, 0, 0 }},
	{"blue", { 25, 25, 255 }},
	{"green", { 10, 255, 10 }},
	{"yellow", { 255, 255, 0 }},
	{"white", { 255, 255, 255 }},
	{"off", { 0, 0, 0 }}
};

const taurus::RGB_char taurus::RGB_fromName(std::string_view colorName) {
	for (auto& [name, rgb] : colorTable) {
		if (name == colorName) {
			return rgb;
		}
	}

	return RGB_OFF;
}

const std::vector<std::string> taurus::RGB_colorNames() {
	static std::vector<std::string> vec;

	for (auto& [name, rgb] : colorTable) {
		vec.push_back(std::string(name));
	}
	return vec;
}

taurus::Controller::Controller(std::string serial) {
	this->serial = serial;
	this->connected = false;

	this->colorName = "off";
	this->color = RGB_OFF;

	this->imuCalibration = ImuCalibration();

	this->initialQuat = glm::angleAxis(glm::radians(90.f), glm::vec3(1.f, 0.f, 0.f));
	this->ahrsState = MadgwickState();
	this->ahrsState.state = glm::quat(initialQuat);
}

PSMove* taurus::Controller::GetMoveHandle() {
	return moveHandle;
}

void taurus::Controller::LoadData() {
	logging::info("Loading controller data for %s", serial.c_str());

	json data;

	// load gyro
	bool success = readJson(createGyroPath(serial), &data);
	if (success) {
		imuCalibration.gyroOffsets = glm::vec3();
		imuCalibration.hasGyro = jsonReadVec3(data, "offsets", &imuCalibration.gyroOffsets);

		if (imuCalibration.hasGyro) {
			logging::info("Successfully loaded gyro calibration");
		}
		else {
			logging::warning("Couldn't load gyro calibration from json!");
		}
	}
	else {
		logging::warning("Gyro calibration could not be loaded, this may cause issues!");
	}
}

void taurus::Controller::Connect(PSMove* move) {
	connected = true;
	connectionType = psmove_connection_type(move);
	moveHandle = move;
}

bool taurus::Controller::Update() {
	// only poll if connected via bluetooth
	bool hadNewData = false;
	if (connectionType != Conn_USB) {
		while (psmove_poll(moveHandle)) {
			HandlePoll();
			hadNewData = true;
		}
	}

	long now = psmove_util_get_ticks();

	// if failed to set color, check if we're connected and try again
	if (colorDirty && connected) {
		colorDirty = false;
		SetColorRaw(color);
	}

	// do rubmle stuff
	HandleRumble(now);
	
	// send led and rumble updates (only every 100ms, or if it's urgent)
	if ((now - lastControllerWrite) >= 100 || controllerWriteUrgent) {
		psmove_update_leds(moveHandle);
		controllerWriteUrgent = false;
		lastControllerWrite = now;
	}

	return hadNewData;
}

void taurus::Controller::Disconnect() {
	if (updateThreadRunning.load()) {
		// someone forgot to StopUpdateThread
		// handle it gracefully
		logging::warning("StopUpdateThread not called before Disconnect-ing the controller! (%s)", serial.c_str());
		StopUpdateThread();
	}

	psmove_disconnect(moveHandle);
}

void taurus::Controller::StartUpdateThread() {
	updateThreadRunning.store(true);
	updateThread = std::thread(&Controller::UpdateThreadFunction, this);
}

void taurus::Controller::StopUpdateThread() {
	updateThreadRunning.store(false);
	updateThread.join();
}

void taurus::Controller::SetColor(std::string_view colorName) {
	this->colorName = colorName;
	color = RGB_fromName(colorName);

	SetColorRaw(color);
}

void taurus::Controller::SetColorRaw(RGB_char color) {
	if (moveHandle != nullptr) {
		psmove_set_leds(moveHandle, color.r, color.g, color.b);
	}
	else {
		colorDirty = true;
	}
}

std::string taurus::Controller::GetColorName() {
	return colorName;
}

void taurus::Controller::DoRumble(float durationSeconds, float strength) {
	rumbleState.active = true;

	rumbleState.durationMs = static_cast<int>(durationSeconds * 1000.f);
	rumbleState.strength = strength;

	rumbleState.startTimeMs = psmove_util_get_ticks();
	rumbleState.elapsedTimeMs = 0;
}

bool taurus::Controller::IsConnected() const {
	return connected;
}

float taurus::Controller::GetBattery01() const {
	return battery01;
}

bool taurus::Controller::IsCharging() const {
	return isCharging;
}

float taurus::Controller::GetTrigger01() const {
	return trigger01;
}

bool taurus::Controller::IsButtonPressed(PSMove_Button button) const {
	return buttonBitfield & button;
}

glm::vec3 taurus::Controller::GetGyro() const {
	return gVec;
}

glm::vec3 taurus::Controller::GetAccel() const {
	return aVec;
}

taurus::MadgwickState* taurus::Controller::GetAhrsState() {
	return &ahrsState;
}

glm::quat taurus::Controller::GetVrQuat() const {
	return vrSpaceQuat;
}

void taurus::Controller::ResetAhrs() {
	ahrsState.state = glm::quat(initialQuat);
}

void taurus::Controller::HandlePoll() {
	long now = psmove_util_get_ticks();
	
	HandleBattery(now);
	HandleInput(now);
	HandleAhrs(now);
}

void taurus::Controller::HandleBattery(long now) {
	batteryState = psmove_get_battery(moveHandle);

	isCharging = (batteryState == Batt_CHARGING) || (batteryState == Batt_CHARGING_DONE);

	float percent = 0.f;
	switch (batteryState) {
		case Batt_MIN:
			percent = 1.f;
			break;
		case Batt_20Percent:
			percent = 20.f;
			break;
		case Batt_40Percent:
			percent = 40.f;
			break;
		case Batt_60Percent:
			percent = 60.f;
			break;
		case Batt_80Percent:
			percent = 80.f;
			break;
		case Batt_MAX:
			percent = 100.f;
			break;
		case Batt_CHARGING:
			percent = 90.f;
			break;
		case Batt_CHARGING_DONE:
			percent = 100.f;
			break;
		default:
			break;
	}

	battery01 = percent / 100.f;
}

void taurus::Controller::HandleInput(long now) {
	buttonBitfield = psmove_get_buttons(moveHandle);

	trigger = psmove_get_trigger(moveHandle);
	trigger01 = static_cast<float>(trigger) / 255.f;
}

void taurus::Controller::HandleAhrs(long now) {
	if (ahrsState.lastSample == 0) {
		// first sample, we have nothing to work with
		ahrsState.lastSample = now;
		return;
	}

	// calculate timestep and half timestep (sensor output gives 2 half-frames)
	long timestepMs = now - ahrsState.lastSample;
	float timestepS = static_cast<float>(timestepMs) / 1000.f;
	float halfTimestepS = timestepS * 0.5f;

	// ignore bad samples
	if (timestepMs < 1) {
		return;
	}

	// estimate frequency, for diagnostics
	ahrsState.freqEstimate = 1.f / timestepS;

	// 2 sensor halves
	for (int frameHalf = 0; frameHalf < 2; frameHalf++) {
		// get the accelerometer and gyroscope for the current frame half
		psmove_get_accelerometer_frame(moveHandle, static_cast<PSMove_Frame>(frameHalf), &aVec.x, &aVec.y, &aVec.z);
		psmove_get_gyroscope_frame(moveHandle, static_cast<PSMove_Frame>(frameHalf), &gVec.x, &gVec.y, &gVec.z);

		// use the imu calibration to correct the sensor readings
		if (imuCalibration.hasGyro) {
			gVec -= imuCalibration.gyroOffsets;
		}

		// perform the ahrs update
		madgwickUpdate(&ahrsState, gVec, aVec, halfTimestepS);

		// swizzle around the axes to get a quaternion in the vr space
		vrSpaceQuat = glm::quat(ahrsState.state.w, ahrsState.state.x, ahrsState.state.z, -ahrsState.state.y);
	}

	ahrsState.lastSample = now;
}

void taurus::Controller::HandleRumble(long now) {
	if (rumbleState.active) {
		psmove_set_rumble(moveHandle, static_cast<unsigned char>(rumbleState.strength * 255.f));

		// handle duration
		rumbleState.elapsedTimeMs = now - rumbleState.startTimeMs;
		if (rumbleState.elapsedTimeMs >= rumbleState.durationMs) {
			rumbleState.strength = 0.f;
			rumbleState.active = false;
		}

		controllerWriteUrgent = true;
	}
	else {
		psmove_set_rumble(moveHandle, 0);
	}
}

void taurus::Controller::UpdateThreadFunction() {
	if (!connected) return;

	while (updateThreadRunning.load()) {
		bool polled = Update();

		// TODO: investigate
		//psmove_util_sleep_ms(1);
		//std::this_thread::yield();
	}
}

taurus::ControllerManager* taurus::ControllerManager::instance = nullptr;

taurus::ControllerManager* taurus::ControllerManager::GetInstance() {
	return instance;
}

taurus::ControllerManager::ControllerManager() {
	taurus::ControllerManager::instance = this;
}

taurus::ControllerManager::ControllerManager(std::vector<std::string> expectedSerials) {
	taurus::ControllerManager::instance = this;

	taurus::ControllerManager::InitPSMoveAPI();

	// pregenerate controller map
	for (int i = 0; i < expectedSerials.size(); i++) {
		std::string serial = expectedSerials[i];

		Controller controller = Controller(serial);
		controllers[serial] = std::make_unique<Controller>(serial);

		controllers[serial]->LoadData();

		allocatedSerials.push_back(serial);
	}

	taurus::logging::info("Init controller manager");
}

void taurus::ControllerManager::InitPSMoveAPI() {
	if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
		taurus::logging::error("PS Move API init failed!");
		exit(1);
	}
}

void taurus::ControllerManager::StartUpdateThreads() {
	for (std::string& p : connectedSerials) {
		controllers[p]->StartUpdateThread();
	}
}

void taurus::ControllerManager::StopUpdateThreads() {
	for (std::string& p : connectedSerials) {
		controllers[p]->StopUpdateThread();
	}
}

void taurus::ControllerManager::ConnectControllers() {
	for (int i = 0; i < psmove_count_connected(); i++) {
		PSMove* move;
		move = psmove_connect_by_id(i);
		char* serial = psmove_get_serial(move);
		std::string serialStr = std::string(serial);
		psmove_free_mem(serial);

		// check if serial in any expected controllers
		// if found then connect it, if not then create a new one
		auto found = controllers.find(serialStr);
		if (found != controllers.end()) {
			found->second->Connect(move);
		}
		else {
			controllers[serial] = std::make_unique<Controller>(serial);
			controllers[serial]->Connect(move);

			allocatedSerials.push_back(serial);
		}

		connectedSerials.push_back(serialStr);
		taurus::logging::info("Connected %s", serial);
	}
}

void taurus::ControllerManager::UpdateControllers() {
	for (std::string& p : connectedSerials) {
		controllers[p]->Update();
	}
}

void taurus::ControllerManager::DisconnectControllers() {
	taurus::logging::info("Disconnecting controllers...");

	for (std::string& p : connectedSerials) {
		controllers[p]->Disconnect();
	}

	allocatedSerials.clear();
	connectedSerials.clear();
}

taurus::Controller* taurus::ControllerManager::GetController(std::string serial) {
	auto found = controllers.find(serial);
	if (found != controllers.end()) {
		return found->second.get();
	}

	return nullptr;
}

std::vector<std::string> taurus::ControllerManager::GetAllocatedSerials() const {
	return allocatedSerials;
}

std::vector<std::string> taurus::ControllerManager::GetConnectedSerials() const {
	return connectedSerials;
}
