#pragma once

#include <map>
#include <ostream>

#pragma pack(push, 1)
struct MagneticFluxDensityDataRawLIS3MDL {
	union {
		struct {
			std::int16_t x;
			std::int16_t y;
			std::int16_t z;
		};
		std::uint8_t bytes[6];
	};
};
#pragma pack(pop)

inline std::ostream& operator<<(std::ostream& os, MagneticFluxDensityDataRawLIS3MDL const& d) {
	os << d.x << "," << d.y << "," << d.z;

	return os;
}