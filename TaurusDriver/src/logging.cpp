#include "logging.h"

#include <stdarg.h>
#include <stdio.h>

// compat with non-windows platforms
#if !defined( WIN32 )
#define vsnprintf_s vsnprintf
#endif

void DriverLog(const char* format, ...)
{
	va_list args;
	char buf[1024];

	va_start(args, format);
	
	vsnprintf_s(buf, sizeof(buf), format, args);
	vr::VRDriverLog()->Log(buf);

	va_end(args);
}
