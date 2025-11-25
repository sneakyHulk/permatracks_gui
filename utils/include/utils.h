#pragma once

#include <common_output.h>

#include <chrono>
#include <cmath>
#include <filesystem>

inline std::filesystem::path generate_filepath(std::string const& stem) {
	auto const current = std::chrono::system_clock::now();
	auto const [ymd, hms] = common::get_year_month_day_hh_mm_ss(current);
	return std::filesystem::path(CMAKE_SOURCE_DIR) / "result" /
	       common::stringprint(
	           stem, "_", std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(current.time_since_epoch()).count()), '_', ymd.year(), ymd.month(), ymd.day(), '_', hms.hours(), hms.minutes(), hms.seconds(), ".csv");
}

inline double angle_between_direction_vectors(double x0, double y0, double z0, double x1, double y1, double z1) {
	double const dot = x0 * x1 + y0 * y1 + z0 * z1;
	double const norm0 = std::sqrt(x0 * x0 + y0 * y0 + z0 * z0);
	double const norm1 = std::sqrt(x1 * x1 + y1 * y1 + z1 * z1);

	double cos_theta = dot / (norm0 * norm1);

	// clamp for numerical safety
	if (cos_theta > 1.0) cos_theta = 1.0;
	if (cos_theta < -1.0) cos_theta = -1.0;

	return std::acos(cos_theta);  // in radians
}