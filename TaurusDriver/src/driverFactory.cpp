#include "deviceDriver.h"
#include "openvr_driver.h"
#include <cstring>

#if defined( _WIN32 )
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#define HMD_DLL_IMPORT extern "C" __declspec(dllimport)
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C"
#else
#error "Unsupported Platform."
#endif

TaurusDeviceDriver deviceDriver;

// driver entry point
HMD_DLL_EXPORT void *HmdDriverFactory(const char *interfaceName, int *returnCode) {
	if (strcmp(vr::IServerTrackedDeviceProvider_Version, interfaceName) == 0) {
		return &deviceDriver;
	}

	if (returnCode) {
		*returnCode = vr::VRInitError_Init_InterfaceNotFound;
	}

	return NULL;
}