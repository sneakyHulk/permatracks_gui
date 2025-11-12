#pragma once
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <numeric>
#include <ranges>

template <std::size_t N>
class Zeroing {
	bool _zeroed = false;

   protected:
	std::array<Eigen::Vector<double, 3>, N> _zeroings;

   public:
	Zeroing() { std::cout << "Zeroing()" << std::endl; }
	~Zeroing() = default;

	void zero(std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, N> const& zeroing_data) {
		for (auto const& [sensor_zeroing_data, zeroing] : std::ranges::views::zip(zeroing_data, _zeroings)) {
			auto const& [x, y, z] = sensor_zeroing_data;

			zeroing(0) = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
			zeroing(1) = std::accumulate(y.begin(), y.end(), 0.0) / y.size();
			zeroing(2) = std::accumulate(z.begin(), z.end(), 0.0) / z.size();
		}

		_zeroed = true;
	}

	void reset_zeroing() { _zeroed = false; }

	bool zeroed() const { return _zeroed; }
};