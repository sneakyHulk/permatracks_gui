#pragma once

#include <ostream>

struct MagneticFluxDensityIntegerData {
	int x;
	int y;
	int z;
};

std::ostream& operator<<(std::ostream& os, MagneticFluxDensityIntegerData const& d);