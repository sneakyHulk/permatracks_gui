#pragma once

#include <ostream>

struct Position {
	double x;
	double y;
	double z;

	friend std::ostream& operator<<(std::ostream& os, Position const& position) {
		os << position.x << "," << position.y << "," << position.z;

		return os;
	}
};