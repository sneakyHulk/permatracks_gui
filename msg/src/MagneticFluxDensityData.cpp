#include "MagneticFluxDensityData.h"

std::ostream& operator<<(std::ostream& os, MagneticFluxDensityData const& d) {
	os << d.x << "," << d.y << "," << d.z;

	return os;
}
std::ostream& operator<<(std::ostream& os, MagneticFluxDensityDataRaw const& d) {
	os << d.x << "," << d.y << "," << d.z;

	return os;
}