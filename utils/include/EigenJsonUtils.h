#pragma once

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>

namespace Eigen {
	template <typename scalar, int... args>
	void to_json(nlohmann::json& j, Matrix<scalar, args...> const& matrix) {
		for (int row = 0; row < matrix.rows(); ++row) {
			nlohmann::json column = nlohmann::json::array();
			for (int col = 0; col < matrix.cols(); ++col) {
				column.push_back(matrix(row, col));
			}
			j.push_back(column);
		}
	}

	template <typename scalar, int... args>
	void from_json(nlohmann::json const& j, Matrix<scalar, args...>& matrix) {
		assert(j.size() == matrix.rows());
		for (std::size_t row = 0; row < j.size(); ++row) {
			const auto& jrow = j.at(row);
			assert(jrow.size() == matrix.cols());
			for (std::size_t col = 0; col < jrow.size(); ++col) {
				const auto& value = jrow.at(col);
				if (value.is_null())
					matrix(row, col) = std::numeric_limits<double>::quiet_NaN();
				else
					matrix(row, col) = value.get<scalar>();
			}
		}
	}

	template <typename scalar, int args>
	void from_json(nlohmann::json const& j, Vector<scalar, args>& vector) {
		assert(j.size() == vector.size());
		for (std::size_t i = 0; i < j.size(); ++i) {
			const auto& value = j.at(i);
			if (value.is_null())
				vector(i) = std::numeric_limits<double>::quiet_NaN();
			else
				vector(i) = value.get<scalar>();
		}
	}
}  // namespace Eigen