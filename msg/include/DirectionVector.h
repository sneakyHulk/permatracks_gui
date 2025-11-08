#pragma once

#include <ostream>

struct DirectionVector {
	double mx;
	double my;
	double mz;

	friend std::ostream& operator<<(std::ostream& os, DirectionVector const& direction) {
		os << direction.mx << "," << direction.my << "," << direction.mz;

		return os;
	}
};