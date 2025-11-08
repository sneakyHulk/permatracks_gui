#pragma once

#include <map>
#include <ostream>

#pragma pack(push, 1)
struct MagneticFluxDensityDataRawMMC5983MA {
	union {
		struct {
			std::int32_t x : 19;
			std::int32_t y : 19;
			std::int32_t z : 18;
		};
		struct {
			std::uint8_t bytes[7];
		};
	};
};
#pragma pack(pop)

inline std::ostream& operator<<(std::ostream& os, MagneticFluxDensityDataRawMMC5983MA const& d) {
	os << d.x << "," << d.y << "," << d.z;

	return os;
}