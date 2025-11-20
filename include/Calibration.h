#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <ranges>

#include "ellipsoid_fit.h"

template <std::size_t N>
class Calibration {
	bool _calibrated = false;

   protected:
	std::array<EllipsoidFitResult, N> _calibrations;

   public:
	Calibration() { std::cout << "Calibration()" << std::endl; }
	~Calibration() = default;

	void calibrate(std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, N> const& calibration_data) {
		for (auto const& [sensor_calibration_data, calibration] : std::ranges::views::zip(calibration_data, _calibrations)) {
			auto const& [x, y, z] = sensor_calibration_data;
			calibration = ellipsoid_fit(x, y, z);
		}

		_calibrated = true;
	}

	void reset_calibration() { _calibrated = false; }

	bool calibrated() const { return _calibrated; }
};