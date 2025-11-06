#include "settings.h"

char* settings::GetModelNumber() {
	static char buffer[128];
	vr::VRSettings()->GetString(settings::sectionMain, settings::keyModelNumber, buffer, sizeof(buffer));
	return buffer;
}

int settings::GetUdpPort() {
	return vr::VRSettings()->GetInt32(settings::sectionMain, settings::keyUdpPort);
}

int settings::GetUdpSendPort() {
	return vr::VRSettings()->GetInt32(settings::sectionMain, settings::keyUdpSendPort);
}

int settings::GetTrackerListTimeout() {
	return vr::VRSettings()->GetInt32(settings::sectionMain, settings::keyTrackerListTimeout);
}

char* settings::GetSerialNumber(vr::ETrackedControllerRole role) {
	static char buffer[128];
	vr::VRSettings()->GetString(
		(role == vr::TrackedControllerRole_LeftHand) ? sectionLeftController : sectionRightController,
		settings::keySerialNumber, buffer, sizeof(buffer)
	);
	return buffer;
}
