#include "ellipsoid_fit.h"
std::tuple<Eigen::Matrix<double, 3, 3>, Eigen::Vector<double, 3>> ellipsoid_fit2(std::vector<double> const& x, std::vector<double> const& y, std::vector<double> const& z) {
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
EllipsoidFitResult ellipsoid_fit(std::vector<double> const& x, std::vector<double> const& y, std::vector<double> const& z) {
	auto points = std::ranges::views::zip(x, y, z);

	Eigen::Matrix<double, 9, Eigen::Dynamic> D{9, points.size()};
	Eigen::Vector<double, Eigen::Dynamic> d2(points.size());

	for (auto i = 0; auto const& [xi, yi, zi] : points) {
		D(0, i) = xi * xi + yi * yi - 2 * zi * zi;
		D(1, i) = xi * xi + zi * zi - 2 * yi * yi;
		D(2, i) = 2 * xi * yi;
		D(3, i) = 2 * xi * zi;
		D(4, i) = 2 * yi * zi;
		D(5, i) = 2 * xi;
		D(6, i) = 2 * yi;
		D(7, i) = 2 * zi;
		D(8, i) = 1;

		d2(i) = xi * xi + yi * yi + zi * zi;

		++i;
	}

	Eigen::Matrix<double, 9, 9> const DDt = D * D.transpose();
	Eigen::Vector<double, 9> const u = DDt.ldlt().solve(D * d2);

	double const a = u(0) + u(1) - 1;
	double const b = u(0) - 2 * u(1) - 1;
	double const c = u(1) - 2 * u(0) - 1;

	Eigen::Vector<double, 10> v;
	v << a, b, c, u.segment<7>(2);

	Eigen::Matrix4d A;
	A << v(0), v(3), v(4), v(6), v(3), v(1), v(5), v(7), v(4), v(5), v(2), v(8), v(6), v(7), v(8), v(9);

	Eigen::Matrix<double, 3, 3> const A3 = A.block<3, 3>(0, 0);
	Eigen::Vector<double, 3> const bv = v.segment<3>(6);
	Eigen::Vector<double, 3> const center = (-A3).ldlt().solve(bv);

	Eigen::Matrix<double, 4, 4> T = Eigen::Matrix4d::Identity();
	T.block<1, 3>(3, 0) = center.transpose();

	Eigen::Matrix<double, 4, 4> R = T * A * T.transpose();

	Eigen::Matrix<double, 3, 3> R3 = R.block<3, 3>(0, 0) / (-R(3, 3));

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(R3);
	Eigen::Vector<double, 3> evals = solver.eigenvalues();
	Eigen::Matrix<double, 3, 3> evecs = solver.eigenvectors().transpose();

	Eigen::Vector<double, 3> radii;
	for (int i = 0; i < 3; ++i) {
		radii(i) = std::sqrt(1.0 / std::abs(evals(i)));
		radii(i) *= (evals(i) > 0 ? 1.0 : -1.0);
	}

	double r = std::cbrt(radii(0) * radii(1) * radii(2));

	Eigen::Matrix<double, 3, 3> S = Eigen::Matrix<double, 3, 3>::Zero();
	S.diagonal() << r / radii(0), r / radii(1), r / radii(2);

	Eigen::Matrix<double, 3, 3> transformation = evecs * S * evecs.transpose();

	return {transformation, center};
}