#include "kinematics.h"
#include <cmath>
#include <iostream>

const double PI = 3.141592653589793;
const double d1 = 0.1;
const double a2 = 0.3;
const double a3 = 0.2;

Matrix4x4 computeDH(double theta, double d, double a, double alpha) {
	double cost = cos(theta), sint = sin(theta);
	double cosa = cos(alpha), sina = sin(alpha);

	return { {
		{cost, -sint * cosa, sint * sina, a * cost},
		{sint, cost * cosa, -cost * sina, a * sint},
		{0,		sina,		cosa,		d},
		{0,		0,			0,			1}
	} };
}

Matrix4x4 multiplyMatrix(const Matrix4x4& A, const Matrix4x4& B) {
	Matrix4x4 result = {};
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			for (int k = 0; k < 4; ++k)
				result[i][j] += A[i][k] * B[k][j];
	return result;
}

Matrix4x4 forwardKinematics(double theta1_deg, double theta2_deg, double theta3_deg) {
	double theta1 = theta1_deg * PI / 180.0;
	double theta2 = theta2_deg * PI / 180.0;
	double theta3 = theta3_deg * PI / 180.0;

	auto T01 = computeDH(theta1, d1, 0.0, PI / 2);
	auto T12 = computeDH(theta2, 0.0, a2, 0.0);
	auto T23 = computeDH(theta3, 0.0, a3, 0.0);

	auto T02 = multiplyMatrix(T01, T12);
	auto T03 = multiplyMatrix(T02, T23);

	return T03;
}

void printPositionOrientation(const Matrix4x4& T) {
	std::cout << "Position: ("
		<< T[0][3] << ", "
		<< T[1][3] << ", "
		<< T[2][3] << ")\n";

	std::cout << "Orientation matrix:\n";
	for (int i = 0; i < 3; ++i) {
		std::cout << T[i][0] << "\t"
			<< T[i][1] << "\t"
			<< T[i][2] << "\n\n";
	}
}

std::vector<IKResult> inverseKinematics(double x, double y, double z) {
	std::vector<IKResult> sols;

	double r = std::hypot(x, y);
	double s = z - d1;
	double theta1 = std::atan2(y, x) * 180.0 / PI;

	double L = r * r + s * s;
	double D = (L - a2 * a2 - a3 * a3) / (2 * a2 * a3);
	if (std::abs(D) > 1.0) {
		sols.push_back({ theta1, 0.0, 0.0, false });
		return sols;
	}

	double root = std::sqrt(1.0 - D * D);
	double theta3a = std::atan2(+root, D) * 180.0 / PI;
	double theta3b = std::atan2(-root, D) * 180.0 / PI;

	auto solveTheta2 = [&](double theta3_deg) {
		double theta3 = theta3_deg * PI / 180.0;
		double num = a3 * std::sin(theta3);
		double den = a2 + a3 * std::cos(theta3);
		double angle = std::atan2(s, r) - std::atan2(num, den);
		return angle * 180.0 / PI;
		};
	
	double theta2a = solveTheta2(theta3a);
	double theta2b = solveTheta2(theta3b);

	if (theta2a >= -90 && theta2a <= 90 && theta3a >= -50 && theta3a <= 50)
		sols.push_back({theta1, theta2a, theta3a, true});
	if (theta2b >= -90 && theta2b <= 90 && theta3b >= -50 && theta3b <= 50)
		sols.push_back({ theta1, theta2b, theta3b, true });

	return sols;
}