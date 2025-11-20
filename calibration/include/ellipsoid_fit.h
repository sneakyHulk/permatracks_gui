#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cstddef>
#include <expected>
#include <ranges>
#include <tuple>
#include <vector>

struct EllipsoidFitResult {
	Eigen::Matrix<double, 3, 3> transformation;
	Eigen::Vector<double, 3> center;
};

std::tuple<Eigen::Matrix<double, 3, 3>, Eigen::Vector<double, 3>> ellipsoid_fit2(std::vector<double> const& x, std::vector<double> const& y, std::vector<double> const& z);
EllipsoidFitResult ellipsoid_fit(std::vector<double> const& x, std::vector<double> const& y, std::vector<double> const& z);