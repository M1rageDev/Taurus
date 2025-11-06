#pragma once

#include <vector>

namespace taurus
{
	int roundToInt(float x);

	template<typename T>
	bool vectorContains(std::vector<T>& vec, T& x) {
		return std::find(vec.begin(), vec.end(), x) != vec.end();
	}
}
