#pragma once

#include <stdarg.h>

namespace taurus::logging
{
	constexpr const char* LOG_PROGRAM_NAME = "Taurus";

	//void debug(const char* format, ...);
	void info(const char* format, ...);
	void warning(const char* format, ...);
	void error(const char* format, ...);

	void log(const char* level, const char* format, va_list args);
	void current_time(char* buffer);
}
