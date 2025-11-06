#pragma once

#include "openvr_driver.h"

namespace settings
{
	static const char* sectionMain = "driver_taurus";
	static const char* sectionLeftController = "driver_taurus_left_controller";
	static const char* sectionRightController = "driver_taurus_right_controller";

	static const char* keyModelNumber = "model_number";
	static const char* keyUdpPort = "udp_port";
	static const char* keyUdpSendPort = "udp_send_port";
	static const char* keyTrackerListTimeout = "tracker_list_timeout";

	static const char* keySerialNumber = "serial";

	// main keys
	char* GetModelNumber();
	int GetUdpPort();
	int GetUdpSendPort();
	int GetTrackerListTimeout();

	// controller keys
	char* GetSerialNumber(vr::ETrackedControllerRole role);
}
