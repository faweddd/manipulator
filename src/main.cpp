#include "kinematics.h"
#include "quaternion.h"
#include <iostream>

int main() {
	
	for (int t1 = -180; t1 <= 180; t1 += 5)
		for (int t2 = -90; t2 <= 90; t2 += 5)
			for (int t3 = -50; t3 <= 50; t3 += 5) {
				auto T = forwardKinematics(t1, t2, t3);
				std::cout << "Angles: 01=" << t1
					<< " 02=" << t2
					<< " 03=" << t3 << " \n";
				printPositionOrientation(T);
			}

	std::cout << "\n--- Inverse Kinematics ---\n";

	for (double X = -0.6; X <= 0.6; X += 0.1) {
		for (double Y = -0.6; Y <= 0.6; Y += 0.1) {
			for (double Z = 0.0; Z <= 0.6; Z += 0.1) {
				if (X * X + Y * Y + Z * Z > 0.6 * 0.6)
					continue;

				auto sols = inverseKinematics(X, Y, Z);
				for (auto& sol: sols) {
					if (sol.reachable) {
						std::cout
							<< "Target (" << X << ", " << Y << ", " << Z << ") -> "
							<< "theta1 =" << sol.t1 << " "
							<< "theta2 =" << sol.t2 << " "
							<< "theta3 =" << sol.t3 << "\n";
					}
				}
			}
		}
	}

	std::cout << "\n--- Quaternion ---\n";

	double ax_deg = 30.0;
	double ay_deg = 40.0;
	double az_deg = 50.0;

	constexpr double PI = 3.14159265358979323846;

	double ax = ax_deg * PI / 180.0;
	double ay = ay_deg * PI / 180.0;
	double az = az_deg * PI / 180.0;

	Quaternion qx { std::cos(ax / 2), std::sin(ax / 2), 0, 0 };
	Quaternion qy { std::cos(ay / 2), 0, std::sin(ay / 2), 0 };
	Quaternion qz { std::cos(az / 2), 0, 0, std::sin(az / 2) };

	Quaternion q = qz * qy * qx;
	q.normalize();

	std::cout << "Detail quaternion:\n";
	std::cout << " w = " << q.w << "\n";
	std::cout << " x = " << q.x << "\n";
	std::cout << " y = " << q.y << "\n";
	std::cout << " z = " << q.z << "\n";

	return 0;
}

	
