#pragma once
#include <array>
#include <cstdint>
#include <ranges>

template <std::size_t N>
std::array<double, N> linspace(double from, double to) {
	std::array<double, N> result;

#if defined(__cpp_lib_ranges_enumerate)
	for (double const step = (to - from) / (N - 1); auto const& [i, e] : std::ranges::views::enumerate(result)) {
#else
	auto i = -1;
	for (double const step = (to - from) / (N - 1); auto& e : result) {
		++i;
#endif
		e = from + i * step;
	}

	return result;
}

template <std::size_t N>
std::array<double, N> linspace_exclusive(double from, double to) {
	std::array<double, N> result;

#if defined(__cpp_lib_ranges_enumerate)
	for (double const step = (to - from) / N; auto const& [i, e] : std::ranges::views::enumerate(result)) {
#else
	auto i = -1;
	for (double const step = (to - from) / N; auto& e : result) {
		++i;
#endif
		e = from + i * step;
	}

	return result;
}