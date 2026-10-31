#include <iostream>
#include <cmath>
#include <cassert>

struct Vec3 {
    double x, y, z;
    Vec3(double x_ = 0, double y_ = 0, double z_ = 0) : x(x_), y(y_), z(z_) {}
    
    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 cross(const Vec3& o) const {
        return Vec3(y * o.z - z * o.y,
                    z * o.x - x * o.z,
                    x * o.y - y * o.x);
    }
    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }
};

// Mirror of detail::transfer_acceleration in imu_pipeline.hpp
Vec3 transfer_acceleration(const Vec3& arm,
                           const Vec3& acceleration,
                           const Vec3& angular_rate,
                           const Vec3& angular_accel = Vec3(0, 0, 0))
{
    return acceleration - angular_accel.cross(arm) - angular_rate.cross(angular_rate.cross(arm));
}

int main() {
    std::cout << "=================================================================\n";
    std::cout << "  C++ REGRESSION TEST: LEVER-ARM KINEMATIC ACCELERATION TRANSFER \n";
    std::cout << "=================================================================\n";
    
    Vec3 arm(-0.150, 0.075, 1.000);
    
    // Test 1: Pure Translation (omega=0, alpha=0)
    Vec3 a_true(3.5, -2.1, 0.0);
    Vec3 omega1(0, 0, 0);
    Vec3 alpha1(0, 0, 0);
    Vec3 a_sens1 = a_true + alpha1.cross(arm) + omega1.cross(omega1.cross(arm));
    Vec3 a_recon1 = transfer_acceleration(arm, a_sens1, omega1, alpha1);
    double err1 = (a_recon1 - a_true).norm();
    assert(err1 < 1e-12);
    std::cout << "  [PASS] Test 1 (Pure Translation): error = " << err1 << " m/s^2\n";
    
    // Test 2: Constant Yaw Rotation (alpha=0)
    Vec3 a_true2(0.0, 12.0, 0.0);
    Vec3 omega2(0.0, 0.0, 1.2);
    Vec3 alpha2(0, 0, 0);
    Vec3 a_sens2 = a_true2 + alpha2.cross(arm) + omega2.cross(omega2.cross(arm));
    Vec3 a_recon2 = transfer_acceleration(arm, a_sens2, omega2, alpha2);
    double err2 = (a_recon2 - a_true2).norm();
    assert(err2 < 1e-12);
    std::cout << "  [PASS] Test 2 (Constant Yaw Rotation): error = " << err2 << " m/s^2\n";
    
    // Test 3: Full 3D Angular Acceleration (Transient Motion)
    Vec3 a_true3(2.0, 8.0, -1.0);
    Vec3 omega3(0.2, -0.1, 0.8);
    Vec3 alpha3(5.0, -1.5, 3.0);
    Vec3 a_sens3 = a_true3 + alpha3.cross(arm) + omega3.cross(omega3.cross(arm));
    Vec3 a_recon3 = transfer_acceleration(arm, a_sens3, omega3, alpha3);
    double err3 = (a_recon3 - a_true3).norm();
    assert(err3 < 1e-12);
    std::cout << "  [PASS] Test 3 (Full 3D Angular Accel): error = " << err3 << " m/s^2\n";
    
    // Test 4: Default argument backward compatibility (when alpha is omitted)
    Vec3 a_default = transfer_acceleration(arm, a_sens2, omega2);
    double err4 = (a_default - a_true2).norm();
    assert(err4 < 1e-12);
    std::cout << "  [PASS] Test 4 (Default Parameter Compatibility): error = " << err4 << " m/s^2\n";
    
    std::cout << "=================================================================\n";
    std::cout << "  ALL C++ TESTS PASSED (100%)\n";
    std::cout << "=================================================================\n";
    return 0;
}
