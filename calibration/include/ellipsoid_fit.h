#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cstddef>
#include <expected>
#include <ranges>
#include <tuple>
#include <vector>

std::tuple<Eigen::Matrix<double, 3, 3>, Eigen::Vector<double, 3>> ellipsoid_fit(std::vector<double> const& x, std::vector<double> const& y, std::vector<double> const& z);