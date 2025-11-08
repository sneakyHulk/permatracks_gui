#pragma once

#include <array>
#include <cstdint>
#include <ostream>

struct RawMagnetometerArrayOutput {
	std::size_t bytes_transferred;
	std::uint32_t timestamp;
	union {
		char string[512];
		std::array<std::uint8_t, 512> buffer;
	};
};