#include "modules/logging.h"

#include <iostream>
#include <chrono>

// compat with non-windows platforms
#if !defined( WIN32 )
#define vsnprintf_s vsnprintf
#endif

/*
void taurus::logging::debug(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log("DEBUG", format, args);
	va_end(args);
}
*/

void taurus::logging::info(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log("INFO", format, args);
	va_end(args);
}

void taurus::logging::warning(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log("WARNING", format, args);
	va_end(args);
}

void taurus::logging::error(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log("ERROR", format, args);
	va_end(args);
}

void taurus::logging::log(const char* level, const char* format, va_list args) {
	char buf[1024];
	vsnprintf_s(buf, sizeof(buf), format, args);

	char timeBuf[16];
	current_time(timeBuf);

	std::cout << "["
		<< LOG_PROGRAM_NAME << "] ["
		<< level << "] ["
		<< timeBuf << "] "
		<< buf <<
		std::endl;
}

void taurus::logging::current_time(char* buffer) {
	std::chrono::time_point now = std::chrono::system_clock::now();
	time_t now_c = std::chrono::system_clock::to_time_t(now);
	tm now_tm;
	localtime_s(&now_tm, &now_c);
	
	strftime(buffer, 16, "%H:%M:%S", &now_tm);
}
