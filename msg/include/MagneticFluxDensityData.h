#pragma once

#include <map>
#include <ostream>

struct MagneticFluxDensityData {
	double x;
	double y;
	double z;
};

std::ostream& operator<<(std::ostream& os, MagneticFluxDensityData const& d);

struct MagneticFluxDensityDataRaw {
	union {
		struct {
			float x;
			float y;
			float z;
		};
		std::uint8_t bytes[12];
	};
};

std::ostream& operator<<(std::ostream& os, MagneticFluxDensityDataRaw const& d);