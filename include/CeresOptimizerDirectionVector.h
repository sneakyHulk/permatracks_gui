#pragma once

#include <ceres/ceres.h>
#include <dipol_model_with_direction_vector.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <ranges>
#include <string>
#include <common.h>

#include "Array.h"
#include "Direction.h"
#include "DirectionVector.h"
#include "MagnetSelection.h"
#include "MagneticFluxDensityData.h"
#include "Message.h"
#include "Pack.h"
#include "Position.h"

template <std::size_t N>
class CeresFunctorPositionAccuracyCost : public ceres::SizedCostFunction<3 * N, 3, 3> {
	double const H1;
	double const R1;
	double const Br1;
	std::array<double, 3 * N> const &target;
	std::array<double, 3 * N> const &weights;

   public:
	CeresFunctorPositionAccuracyCost(double const H1, double const R1, double const Br1, std::array<double, 3 * N> const &target, std::array<double, 3 * N> const &weights) : H1(H1), R1(R1), Br1(Br1), target(target), weights(weights) {}

   private:
	bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
		double const *position = parameters[0];   // x, y, z
		double const *direction = parameters[1];  // mx, my, mz (unit vector)

		dipol_model_with_direction_vector(residuals, H1, R1, Br1, position[0], position[1], position[2], direction[0], direction[1], direction[2]);

		for (auto i = 0; i < 3 * N; ++i) {
			residuals[i] = (residuals[i] - target[i]) * weights[i];
		}

		if (jacobians != nullptr && jacobians[0] != nullptr) {
			dipol_model_with_direction_vector_jacobian_position(jacobians[0], H1, R1, Br1, position[0], position[1], position[2], direction[0], direction[1], direction[2]);
			dipol_model_with_direction_vector_jacobian_direction(jacobians[1], H1, R1, Br1, position[0], position[1], position[2]);

			for (auto i = 0; i < 3 * N; ++i) {
				jacobians[0][i * 3] *= weights[i];
				jacobians[0][i * 3 + 1] *= weights[i];
				jacobians[0][i * 3 + 2] *= weights[i];
				jacobians[1][i * 3] *= weights[i];
				jacobians[1][i * 3 + 1] *= weights[i];
				jacobians[1][i * 3 + 2] *= weights[i];
			}
		}

		return true;
	}
};

template <std::size_t N>
class CeresFunctorDirectionConsistencyCost : public ceres::SizedCostFunction<3 * N, 3, 3> {
	double const H1;
	double const R1;
	double const Br1;
	std::array<double, 3 * N> const &target;
	std::array<double, 3 * N> const &weights;

   public:
	CeresFunctorDirectionConsistencyCost(double const H1, double const R1, double const Br1, std::array<double, 3 * N> const &target, std::array<double, 3 * N> const &weights) : H1(H1), R1(R1), Br1(Br1), target(target), weights(weights) {}

   private:
	bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
		double const *position = parameters[0];   // x, y, z
		double const *direction = parameters[1];  // mx, my, mz (unit vector)

		dipol_model_with_direction_vector(residuals, H1, R1, Br1, position[0], position[1], position[2], direction[0], direction[1], direction[2]);

		for (auto i = 0; i < 3 * N; ++i) {
			residuals[i] = (residuals[i] - target[i]) * weights[i];
		}

		if (jacobians != nullptr && jacobians[0] != nullptr) {
			dipol_model_with_direction_vector_jacobian_position(jacobians[0], H1, R1, Br1, position[0], position[1], position[2], direction[0], direction[1], direction[2]);
			dipol_model_with_direction_vector_jacobian_direction(jacobians[1], H1, R1, Br1, position[0], position[1], position[2]);
		}

		return true;
	}
};

template <std::size_t N>
class CeresOptimizerDirectionVector : virtual protected MagnetSelection {
	std::array<double, 3> init_position;
	std::array<double, 3> init_direction;
	std::array<double, 3> init_offset = {0., 0., 0.};

#if BOARD_VERSION == 1
	std::array<double, 3 * N> noises = {
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    0.04 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.32 * 1e-6,
	    // 0.41 * 1e-6,

	};
#elif BOARD_VERSION == 2
	std::array<double, 3 * N> noises = common::filled_array<3 * N>(0.04 * 1e-6);
#endif



   public:
	explicit CeresOptimizerDirectionVector(std::array<double, 3> &&init_position = {75e-3, 75e-3, 75e-3}, std::array<double, 3> &&init_direction = {1., 0., 0.}) : init_position(init_position), init_direction(init_direction) {
		assert(init_direction[0] * init_direction[0] + init_direction[1] * init_direction[1] + init_direction[2] * init_direction[2] == 1.);  // direction must be a unit vector
	}

	Message<Pack<Position, DirectionVector>> process(Message<Array<MagneticFluxDensityData, N>> const &data) {
		std::array<double, 3 * N> target;

		for (auto i = 0; i < N; ++i) {
			auto const &[inx, iny, inz] = data[i];

			target[i * 3] = inx;
			target[i * 3 + 1] = iny;
			target[i * 3 + 2] = inz;
		}

		std::array<double, 3 * N> noise_weights;
		for (auto const &[w, t, n] : std::ranges::views::zip(noise_weights, target, noises)) {
			if (std::isnan(t)) {
				w = t = 0.;
			} else {
				w = 1.0 / n;
			}
		}

		std::array<double, 3> position = init_position;
		std::array<double, 3> direction = init_direction;

		do_ceres(position, direction, target, noise_weights);

		return {position[0], position[1], position[2], direction[0], direction[1], direction[2], data.timestamp, data.src};
	}

   private:
	void do_top_k_filter(std::array<double, 3 * N> &weights, int k = 15) {
		std::array<int, 3 * N> indices;
		std::iota(indices.begin(), indices.end(), 0);
		std::ranges::partial_sort(indices, indices.begin() + 25, {}, [&](int i) { return weights[i]; });

		std::fill(weights.begin(), weights.end(), 0.);
		for (auto const &i : indices | std::ranges::views::take(k)) {
			weights[i] = 1.0;
		}
	}

	void do_reciprocal_scaling(std::array<double, 3 * N> &weights, double epsilon = 1e-4) {  // epsilon for smoothing
		for (auto &w : weights) w = 1 / (std::abs(w) + epsilon);

		double sum = std::accumulate(weights.begin(), weights.end(), 0.0) / weights.size();
		for (auto &w : weights) w /= sum;
	}

	void do_max_filter(std::array<double, 3 * N> &weights, double max = 0.002) {  // max in mm
		for (auto &w : weights) {
			if (w < max)
				w = 1.0;
			else
				w = 0.;
		}
	}

	void do_ceres(std::array<double, 3> &position, std::array<double, 3> &direction, std::array<double, 3 * N> &target, std::array<double, 3 * N> &weights) {
		ceres::Problem problem;

		const auto &[H, R, Br] = MagnetSelection::_magnets[0];

		problem.AddParameterBlock(position.data(), 3);
		problem.AddParameterBlock(direction.data(), 3, new ceres::SphereManifold<3>());

		problem.SetParameterLowerBound(position.data(), 0, -50e-3);
		problem.SetParameterUpperBound(position.data(), 0, 200e-3);
		problem.SetParameterLowerBound(position.data(), 1, -50e-3);
		problem.SetParameterUpperBound(position.data(), 1, 200e-3);
		problem.SetParameterLowerBound(position.data(), 2, 0);
		problem.SetParameterUpperBound(position.data(), 2, 300e-3);

		ceres::CostFunction *cost_function = new CeresFunctorPositionAccuracyCost<N>(H, R, Br, target, weights);
		ceres::CostFunction *cost_function2 = new CeresFunctorDirectionConsistencyCost<N>(H, R, Br, target, weights);

		problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.345), position.data(), direction.data());
		problem.AddResidualBlock(cost_function2, new ceres::SoftLOneLoss(0.1), position.data(), direction.data());

		ceres::Solver::Options const options{.minimizer_progress_to_stdout = false};

		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
	}
};