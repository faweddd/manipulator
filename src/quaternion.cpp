#include "quaternion.h"

Quaternion Quaternion::operator*(const Quaternion o) const {
	return {
	w * o.w - x * o.x - y * o.y - z * o.z,
	w * o.x + x * o.w + y * o.z - z * o.y,
	w * o.y - x * o.z + y * o.w + z * o.x,
	w * o.z + x * o.y - y * o.x + z * o.w
	};
}

void Quaternion::normalize() {
	double n = std::sqrt(w * w + x * x + y * y + z * z);
	if (n > 0.0) {
		w /= n;
		x /= n;
		y /= n;
		z /= n;
	}
}