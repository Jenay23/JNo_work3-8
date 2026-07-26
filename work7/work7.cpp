#include <cmath>
#include <iostream>
#include <iomanip>

int main() {
    const double v0 = 17.0;
    const double k1 = 0.0089;
    const double g = 9.8;
    const double x = 3.0;
    const double y = 4.0;
    const double s = sqrt(x * x + y * y);
    const double z0 = 0.25;
    const int n = 15;

    double z_temp = z0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "迭代补偿:\n";

    for (int i = 1; i <= n; ++i) {
        double theta = atan2(z_temp, s);
        double vx0 = v0 * cos(theta);
        double t = (exp(k1 * s) - 1) / (k1 * vx0);
        double z_real = v0 * sin(theta) * t - 0.5 * g * t * t;
        double dz = z0 - z_real;
        z_temp += dz;

        std::cout << "iter " << std::setw(2) << i
                  << "  theta =" << std::setw(10) << theta * 180.0 / M_PI
                  << "°  z_real =" << std::setw(10) << z_real
                  << "  dz =" << std::setw(10) << dz << std::endl;
    }

    double theta_final = atan2(z_temp, s);
    std::cout << "最终出射角: " << theta_final * 180.0 / M_PI << "°" << std::endl;

    return 0;
}
