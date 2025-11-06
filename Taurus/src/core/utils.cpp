#include "core/utils.h"

#include <iostream>

int taurus::roundToInt(float x) {
	return static_cast<int>(std::round(x));
}
