/*
FILE DESCRIPTION:

Main entry-point for the application. See "app/taurus_app.cpp" for more interesting content
*/

#include "app/taurus_app.h"

namespace logging = taurus::logging;

int main() {
	taurus::TaurusApp app;
	app.Run();

	return 0;
}
