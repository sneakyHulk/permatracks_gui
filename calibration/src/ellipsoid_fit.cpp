#include "ellipsoid_fit.h"
std::tuple<Eigen::Matrix<double, 3, 3>, Eigen::Vector<double, 3>> ellipsoid_fit(std::vector<double> const& x, std::vector<double> const& y, std::vector<double> const& z) {
	auto points = std::ranges::views::zip(x, y, z);

	Eigen::Matrix<double, 10, Eigen::Dynamic> D;  // = Eigen::Matrix<double, 10, Eigen::Dynamic>::Zero(10, points.size());

	D.resize(10, points.size());

	for (auto cols = 0; auto const& [x, y, z] : points) {
		D(0, cols) = x * x;
		D(1, cols) = y * y;
		D(2, cols) = z * z;
		D(3, cols) = 2.0 * y * z;
		D(4, cols) = 2.0 * x * z;
		D(5, cols) = 2.0 * x * y;
		D(6, cols) = 2.0 * x;
		D(7, cols) = 2.0 * y;
		D(8, cols) = 2.0 * z;
		D(9, cols) = 1.0;

		++cols;
	}

	Eigen::Matrix<double, 10, 10> S = D * D.transpose();
	Eigen::Matrix<double, 6, 6> const S11 = S.block<6, 6>(0, 0);
	Eigen::Matrix<double, 6, 4> const S12 = S.block<6, 4>(0, 6);
	Eigen::Matrix<double, 4, 6> const S21 = S.block<4, 6>(6, 0);
	Eigen::Matrix<double, 4, 4> const S22 = S.block<4, 4>(6, 6);

	Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
	C.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Ones();
	C.block<3, 3>(0, 0).diagonal() = -Eigen::Vector<double, 3>::Ones();
	C.block<3, 3>(3, 3).diagonal() = -4.0 * Eigen::Vector<double, 3>::Ones();

	Eigen::Matrix<double, 6, 6> const A = S11 - S12 * S22.inverse() * S21;  // if S22 is almost singular S22^-1 can be replaced with generalized inverse (.completeOrthogonalDecomposition().pseudoInverse())
	Eigen::Matrix<double, 6, 6> CA = C.inverse() * A;

	Eigen::EigenSolver<decltype(CA)> const solver(CA);

	Eigen::Index index_largest_eigenvalue;
	solver.eigenvalues().real().maxCoeff(&index_largest_eigenvalue);

	Eigen::Vector<double, 6> const v1 = solver.eigenvectors().col(index_largest_eigenvalue).real();
	Eigen::Vector<double, 4> const v2 = S22.inverse() * S21 * v1;

	Eigen::Vector<double, 10> v;
	v << v1, -v2;

	Eigen::Matrix<double, 3, 3> Q;
	Q << v(0), v(5), v(4), v(5), v(1), v(3), v(4), v(3), v(2);
	Eigen::Vector<double, 3> U;
	U << v(6), v(7), v(8);

	Eigen::Vector<double, 3> const B = -(Q.inverse() * U);

	Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> const solver2(Q);

	double const abc = B.transpose() * Q * B - v(9);
	Eigen::DiagonalMatrix<double, 3> const D2 = (solver2.eigenvalues().real() * (1 / abc)).cwiseSqrt().asDiagonal();

	Eigen::Matrix<double, 3, 3> const SQ = solver2.eigenvectors().real() * D2 * solver2.eigenvectors().real().transpose();

	return {SQ * 47.9751 * 1e-6, B};
}