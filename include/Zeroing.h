#pragma once
#include <Calibration.h>

#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <numeric>
#include <ranges>
#include <so3.hpp>

template <typename Tuple>
auto zip_from_tuple(Tuple&& tup) {
	return std::apply([](auto&&... args) { return std::views::zip(std::forward<decltype(args)>(args)...); }, std::forward<Tuple>(tup));
}

template <std::size_t N>
class Zeroing : protected virtual Calibration<N> {
	bool _zeroed = false;

   protected:
	std::array<Eigen::Vector<double, 3>, N> _zeroings;
	std::array<Sophus::SO3<double>, N> _earth_rotations;

   public:
	Zeroing() { std::cout << "Zeroing()" << std::endl; }
	~Zeroing() = default;

	void zero(std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, N> const& zeroing_data) {
		for (auto const& [calibration, zeroing, earth_rotation, sensor_zeroing_data] : std::ranges::views::zip(Calibration<N>::_calibrations, _zeroings, _earth_rotations, zeroing_data)) {
			// calculate mean of applied calibration zeroing data
			auto const& [x, y, z] = sensor_zeroing_data;
			zeroing(0) = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
			zeroing(1) = std::accumulate(y.begin(), y.end(), 0.0) / y.size();
			zeroing(2) = std::accumulate(z.begin(), z.end(), 0.0) / z.size();

			earth_rotation = Sophus::SO3<double>::exp(zeroing.normalized());
		}

		for (auto const& [zeroing, earth_rotation] : std::ranges::views::zip(_zeroings, _earth_rotations)) {
			zeroing = earth_rotation * zeroing;
		}

		_zeroed = true;
	}

	void reset_zeroing() { _zeroed = false; }

	bool zeroed() const { return _zeroed; }
};