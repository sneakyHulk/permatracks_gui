#pragma once

#include <array>
#include <cstdint>
#include <ostream>

template <typename Type, std::size_t N>
struct Array : public std::array<Type, N> {};

template <typename Type, std::size_t N>
std::ostream& operator<<(std::ostream& os, Array<Type, N> const& msg) {
	for (char space[]{0, 0}; auto const& e : msg) {
		os << space << e, *space = ',';
	}

	return os;
}