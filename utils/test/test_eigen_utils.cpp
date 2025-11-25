#include <fstream>
#include <iostream>
#include <ranges>

#include "EigenJsonUtils.h"

int main() {
	Eigen::VectorXd v(5);
	v << 1, std::numeric_limits<double>::quiet_NaN(), 3, std::numeric_limits<double>::quiet_NaN(), 5;

	Eigen::Vector<bool, -1> mask = !v.array().isNaN();

	std::cout << mask.count() << std::endl;

	for (auto const [value, filter] : std::ranges::views::zip(v, mask) | std::ranges::views::filter([](auto const &v) { return !std::get<1>(v); })) {
		// std::cout << value << std::endl;
	}

	auto filtered = std::views::zip(v, mask) | std::views::filter([](auto pair) { return std::get<1>(pair); }) | std::views::transform([](auto pair) { return std::get<0>(pair); });

	for (auto const [i, j] :
	    std::views::enumerate(std::ranges::views::zip(mask, std::ranges::views::iota(0)) | std::ranges::views::filter([](auto const &pair) { return std::get<0>(pair); }) | std::ranges::views::transform([](auto const &pair) { return std::get<1>(pair); }))) {
		std::cout << i << ", " << j << std::endl;
	}

	// std::cout << v << std::endl;
}