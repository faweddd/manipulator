#pragma once
#include <cmath>

struct Quaternion {
	double w, x, y, z;

	Quaternion operator*(const Quaternion o) const;
	void normalize();
};