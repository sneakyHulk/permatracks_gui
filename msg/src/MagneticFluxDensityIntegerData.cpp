#include "MagneticFluxDensityIntegerData.h"

std::ostream& operator<<(std::ostream& os, MagneticFluxDensityIntegerData const& d) {
	os << d.x << "," << d.y << "," << d.z;

	return os;
}