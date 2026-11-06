/**
 * @file run_telemetry_replay.cpp
 * @brief Open-loop telemetry replay driver for experimental correlation of the
 *        14-DOF C++20 TeR26 Multibody Vehicle Dynamics Engine against real TeR26 logs.
 */

#include "ter26/multibody/vehicle_dynamics_engine.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace ter26;

struct ControlSample {
    double t{0.0};
    double throttle{0.0};
    double brake{0.0};
    double steer_rad{0.0};
    double vx_ref{0.0};
    double vy_ref{0.0};
    double r_ref{0.0};
    int reset_flag{0};
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_controls.csv> <output_sim.csv>\n";
        return 1;
    }

    const std::string in_path  = argv[1];
    const std::string out_path = argv[2];

    std::ifstream fin(in_path);
    if (!fin.good()) {
        std::cerr << "Failed to open input CSV: " << in_path << "\n";
        return 1;
    }

    std::vector<ControlSample> samples;
    std::string line;
    // Skip header
    std::getline(fin, line);
    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string tok;
        ControlSample s{};
        if (std::getline(ss, tok, ',')) s.t = std::stod(tok);
        if (std::getline(ss, tok, ',')) s.throttle = std::stod(tok);
        if (std::getline(ss, tok, ',')) s.brake = std::stod(tok);
        if (std::getline(ss, tok, ',')) s.steer_rad = std::stod(tok);
        if (std::getline(ss, tok, ',')) s.vx_ref = std::stod(tok);
        if (std::getline(ss, tok, ',')) s.vy_ref = std::stod(tok);
        if (std::getline(ss, tok, ',')) s.r_ref = std::stod(tok);
        if (std::getline(ss, tok, ',')) s.reset_flag = std::stoi(tok);
        samples.push_back(s);
    }
    fin.close();

    if (samples.empty()) {
        std::cerr << "No samples found in " << in_path << "\n";
        return 1;
    }

    VehicleDynamicsEngine engine(VehicleDynamicsEngine::default_ter26_parameters());
    std::array<GroundContactQuery, 4> flat_ground{};
    for (int i = 0; i < 4; ++i) {
        flat_ground[i].point_w  = Vec3::Zero();
        flat_ground[i].normal_w = Vec3::UnitZ();
        flat_ground[i].friction_multiplier = 1.0;
    }

    const VehicleState s_eq = engine.compute_static_equilibrium(Vec3(0.0, 0.0, 0.305), Vec3::Zero(), &flat_ground);
    VehicleState curr = s_eq;

    std::ofstream fout(out_path);
    fout << "t,vx,vy,yaw_rate,ax_imu,ay_imu,az_imu,rl_rpm,rr_rpm,fl_fz,fr_fz,rl_fz,rr_fz,power_w\n";
    fout << std::fixed << std::setprecision(6);

    for (size_t k = 0; k < samples.size(); ++k) {
        const auto& smp = samples[k];
        const double dt = (k > 0) ? std::clamp(smp.t - samples[k - 1].t, 0.0005, 0.02) : 0.0025;

        if (k == 0 || smp.reset_flag == 1) {
            curr = s_eq;
            curr.v_b = Vec3(smp.vx_ref, smp.vy_ref, 0.0);
            curr.omega_b = Vec3(0.0, 0.0, smp.r_ref);
            curr.steering_rack_angle_rad = smp.steer_rad;
            curr.brake_pressure_norm = smp.brake;
            const double w0 = smp.vx_ref / 0.197;
            curr.omega_wheel = {w0, w0, w0, w0};
        }

        ControlInputs u{};
        u.throttle         = smp.throttle;
        u.brake            = smp.brake;
        u.steering_cmd_rad = smp.steer_rad;
        u.ebs_active       = false;

        // Keep ground query beneath current vehicle position
        for (int i = 0; i < 4; ++i) {
            flat_ground[i].point_w = Vec3(curr.p_w.x(), curr.p_w.y(), 0.0);
        }

        const VehicleStepResult res = engine.step_vehicle_dynamics(curr, u, flat_ground, ExternalInteraction{}, dt);
        curr = res.next_state;

        const double rl_rpm = curr.omega_wheel[RL] * (60.0 / (2.0 * M_PI));
        const double rr_rpm = curr.omega_wheel[RR] * (60.0 / (2.0 * M_PI));

        fout << smp.t << ","
             << curr.v_b.x() << ","
             << curr.v_b.y() << ","
             << curr.omega_b.z() << ","
             << res.specific_force_imu_b.x() << ","
             << res.specific_force_imu_b.y() << ","
             << res.specific_force_imu_b.z() << ","
             << rl_rpm << ","
             << rr_rpm << ","
             << res.corners[FL].Fz_N << ","
             << res.corners[FR].Fz_N << ","
             << res.corners[RL].Fz_N << ","
             << res.corners[RR].Fz_N << ","
             << res.electrical_power_W << "\n";
    }

    fout.close();
    return 0;
}
