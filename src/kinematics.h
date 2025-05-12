#pragma once
#include <array>
#include <vector>

using Matrix4x4 = std::array<std::array<double, 4>, 4>;

struct IKResult {
	double t1, t2, t3;
	bool reachable;
};

Matrix4x4 computeDH(double theta, double d, double a, double alpha);
Matrix4x4 multiplyMatrix(const Matrix4x4& A, const Matrix4x4& B);
Matrix4x4 forwardKinematics(double theta1, double theta2, double theta3);
void printPositionOrientation(const Matrix4x4& T);

std::vector<IKResult> inverseKinematics(double x, double y, double z);