/**
 * @file vehicle_model_binding.cpp
 * @brief Implementation of the Godot 4.7 GDExtension binding `Ter26VehicleBridge`
 *        and GDExtension entry point `ter26_vehicle_library_init`.
 */

#include "vehicle_model_binding.hpp"
#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

namespace godot {

namespace {

inline ter26::Vec3 to_iso_vec(const Vector3& v_g) noexcept {
    return ter26::math::vec_godot_to_iso(ter26::Vec3(v_g.x, v_g.y, v_g.z));
}

inline Vector3 to_godot_vec(const ter26::Vec3& v_iso) noexcept {
    const ter26::Vec3 vg = ter26::math::vec_iso_to_godot(v_iso);
    return Vector3(vg.x(), vg.y(), vg.z());
}

inline ter26::Mat3 to_iso_rot(const Basis& b_g) noexcept {
    ter26::Mat3 R_g;
    // Godot Basis stores columns in elements: b_g.get_column(j)
    const Vector3 c0 = b_g.get_column(0);
    const Vector3 c1 = b_g.get_column(1);
    const Vector3 c2 = b_g.get_column(2);
    R_g << c0.x, c1.x, c2.x,
           c0.y, c1.y, c2.y,
           c0.z, c1.z, c2.z;
    return ter26::math::project_to_so3(ter26::math::rot_godot_to_iso(R_g));
}

inline Basis to_godot_basis(const ter26::Mat3& R_iso) noexcept {
    const ter26::Mat3 R_g = ter26::math::rot_iso_to_godot(R_iso);
    Basis b;
    b.set_column(0, Vector3(R_g(0, 0), R_g(1, 0), R_g(2, 0)));
    b.set_column(1, Vector3(R_g(0, 1), R_g(1, 1), R_g(2, 1)));
    b.set_column(2, Vector3(R_g(0, 2), R_g(1, 2), R_g(2, 2)));
    return b;
}

} // anonymous namespace

void Ter26VehicleBridge::_bind_methods() {
    ClassDB::bind_method(D_METHOD("reset_to_transform_godot", "transform_godot"), &Ter26VehicleBridge::reset_to_transform_godot);
    ClassDB::bind_method(D_METHOD("step_godot", "dt_s", "throttle", "brake", "steering_rad", "ebs_active", "ground_points_godot", "ground_normals_godot"), &Ter26VehicleBridge::step_godot);
    ClassDB::bind_method(D_METHOD("apply_collision_impulse_godot", "contact_point_godot", "contact_normal_godot", "other_vel_godot", "other_mass_kg", "restitution"), &Ter26VehicleBridge::apply_collision_impulse_godot);
    ClassDB::bind_method(D_METHOD("get_chassis_transform_godot"), &Ter26VehicleBridge::get_chassis_transform_godot);
    ClassDB::bind_method(D_METHOD("get_chassis_linear_velocity_godot"), &Ter26VehicleBridge::get_chassis_linear_velocity_godot);
    ClassDB::bind_method(D_METHOD("get_chassis_angular_velocity_godot"), &Ter26VehicleBridge::get_chassis_angular_velocity_godot);
    ClassDB::bind_method(D_METHOD("get_wheel_local_transform_godot", "corner_idx"), &Ter26VehicleBridge::get_wheel_local_transform_godot);
    ClassDB::bind_method(D_METHOD("get_wheel_rpm", "corner_idx"), &Ter26VehicleBridge::get_wheel_rpm);
    ClassDB::bind_method(D_METHOD("get_diagnostics_dictionary"), &Ter26VehicleBridge::get_diagnostics_dictionary);
    ClassDB::bind_method(D_METHOD("set_powertrain_mode", "mode_idx"), &Ter26VehicleBridge::set_powertrain_mode);
}

Ter26VehicleBridge::Ter26VehicleBridge()
    : engine_(ter26::VehicleDynamicsEngine::default_ter26_parameters())
{
    state_ = engine_.compute_static_equilibrium();
    std::array<ter26::GroundContactQuery, 4> g{};
    last_result_ = engine_.evaluate_derivatives(state_, ter26::ControlInputs{}, g, ter26::ExternalInteraction{});
}

void Ter26VehicleBridge::reset_to_transform_godot(const Transform3D& transform_godot) {
    const ter26::Vec3 p_node_iso = to_iso_vec(transform_godot.origin);
    const ter26::Mat3 R_iso      = to_iso_rot(transform_godot.basis);
    const ter26::Vec3 rpy        = ter26::math::rot_to_rpy_iso(R_iso);
    const double h_cg            = engine_.parameters().h_cg_m;
    const ter26::Vec3 p_cg_iso   = p_node_iso + R_iso * ter26::Vec3(0.0, 0.0, h_cg);

    std::array<ter26::GroundContactQuery, 4> g{};
    for (int i = 0; i < 4; ++i) {
        g[i].point_w  = ter26::Vec3(p_cg_iso.x(), p_cg_iso.y(), p_node_iso.z());
        g[i].normal_w = ter26::Vec3::UnitZ();
        g[i].friction_multiplier = 1.0;
    }
    state_ = engine_.compute_static_equilibrium(p_cg_iso, rpy, &g);
    last_result_ = engine_.evaluate_derivatives(state_, ter26::ControlInputs{}, g, ter26::ExternalInteraction{});
}

Dictionary Ter26VehicleBridge::step_godot(
    double dt_s,
    double throttle,
    double brake,
    double steering_rad,
    bool ebs_active,
    const PackedFloat64Array& ground_points_godot,
    const PackedFloat64Array& ground_normals_godot)
{
    ter26::ControlInputs u{};
    u.throttle         = throttle;
    u.brake            = brake;
    u.steering_cmd_rad = steering_rad;
    u.ebs_active       = ebs_active;

    std::array<ter26::GroundContactQuery, 4> g{};
    for (int i = 0; i < 4; ++i) {
        if (ground_points_godot.size() >= 12) {
            const Vector3 pg(ground_points_godot[3 * i + 0],
                             ground_points_godot[3 * i + 1],
                             ground_points_godot[3 * i + 2]);
            g[i].point_w = to_iso_vec(pg);
        } else {
            g[i].point_w = ter26::Vec3::Zero();
        }
        if (ground_normals_godot.size() >= 12) {
            const Vector3 ng(ground_normals_godot[3 * i + 0],
                             ground_normals_godot[3 * i + 1],
                             ground_normals_godot[3 * i + 2]);
            g[i].normal_w = to_iso_vec(ng).normalized();
        } else {
            g[i].normal_w = ter26::Vec3::UnitZ();
        }
        g[i].friction_multiplier = 1.0;
    }

    last_result_ = engine_.step_vehicle_dynamics(state_, u, g, ter26::ExternalInteraction{}, dt_s);
    state_ = last_result_.next_state;

    return get_diagnostics_dictionary();
}

Vector3 Ter26VehicleBridge::apply_collision_impulse_godot(
    const Vector3& contact_point_godot,
    const Vector3& contact_normal_godot,
    const Vector3& other_vel_godot,
    double other_mass_kg,
    double restitution)
{
    ter26::ExternalInteraction ext{};
    ext.has_collision_impulse   = true;
    ext.impulse_contact_point_w = to_iso_vec(contact_point_godot);
    ext.impulse_normal_w        = to_iso_vec(contact_normal_godot).normalized();
    ext.other_body_vel_w        = to_iso_vec(other_vel_godot);
    ext.other_body_mass_kg      = other_mass_kg;
    ext.restitution             = restitution;

    std::array<ter26::GroundContactQuery, 4> g{};
    for (int i = 0; i < 4; ++i) {
        g[i].point_w  = ter26::Vec3(state_.p_w.x(), state_.p_w.y(), 0.0);
        g[i].normal_w = ter26::Vec3::UnitZ();
    }

    const auto res = engine_.step_vehicle_dynamics(state_, ter26::ControlInputs{}, g, ext, 1e-5);
    state_.v_b         = res.next_state.v_b;
    state_.omega_b     = res.next_state.omega_b;
    state_.v_susp      = res.next_state.v_susp;
    state_.omega_wheel = res.next_state.omega_wheel;

    return to_godot_vec(res.external_impulse_applied_w);
}

Transform3D Ter26VehicleBridge::get_chassis_transform_godot() const {
    const double h_cg = engine_.parameters().h_cg_m;
    const ter26::Vec3 p_node_iso = state_.p_w - state_.R_wb * ter26::Vec3(0.0, 0.0, h_cg);
    return Transform3D(to_godot_basis(state_.R_wb), to_godot_vec(p_node_iso));
}

Vector3 Ter26VehicleBridge::get_chassis_linear_velocity_godot() const {
    const ter26::Vec3 v_w = state_.R_wb * state_.v_b;
    return to_godot_vec(v_w);
}

Vector3 Ter26VehicleBridge::get_chassis_angular_velocity_godot() const {
    const ter26::Vec3 omega_w = state_.R_wb * state_.omega_b;
    return to_godot_vec(omega_w);
}

Transform3D Ter26VehicleBridge::get_wheel_local_transform_godot(int corner_idx) const {
    const int idx = std::clamp(corner_idx, 0, 3);
    const auto& c = last_result_.corners[idx];
    // Include wheel spin angle phi_wheel[idx] about local +Y spindle axis
    const ter26::Mat3 R_spin = Eigen::AngleAxisd(state_.phi_wheel[idx], ter26::Vec3::UnitY()).toRotationMatrix();
    const ter26::Mat3 R_w_b  = c.wheel_orientation_b * R_spin;
    const Basis b_scene_offset(Vector3(-1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, -1.0));
    return Transform3D(to_godot_basis(R_w_b) * b_scene_offset, to_godot_vec(c.wheel_center_b));
}

double Ter26VehicleBridge::get_wheel_rpm(int corner_idx) const {
    const int idx = std::clamp(corner_idx, 0, 3);
    return state_.omega_wheel[idx] * (60.0 / (2.0 * M_PI));
}

Dictionary Ter26VehicleBridge::get_diagnostics_dictionary() const {
    Dictionary d;
    d["vx"]        = state_.v_b.x();
    d["vy"]        = state_.v_b.y();
    d["vz"]        = state_.v_b.z();
    d["yaw_rate"]  = state_.omega_b.z();
    d["roll_rate"] = state_.omega_b.x();
    d["pitch_rate"]= state_.omega_b.y();
    d["ax_imu"]    = last_result_.specific_force_imu_b.x();
    d["ay_imu"]    = last_result_.specific_force_imu_b.y();
    d["az_imu"]    = last_result_.specific_force_imu_b.z();
    d["steer_angle_actual"] = state_.steering_rack_angle_rad;
    d["electrical_power_W"] = last_result_.electrical_power_W;
    d["energy_J"]  = last_result_.total_mechanical_energy_J;

    const char* names[4] = {"FL", "FR", "RL", "RR"};
    for (int i = 0; i < 4; ++i) {
        const auto& c = last_result_.corners[i];
        Dictionary cd;
        cd["Fz"]     = c.Fz_N;
        cd["Fx"]     = c.Fx_N;
        cd["Fy"]     = c.Fy_N;
        cd["Mx"]     = c.Mx_Nm;
        cd["My"]     = c.My_Nm;
        cd["Mz"]     = c.Mz_Nm;
        cd["kappa"]  = c.kappa_dynamic;
        cd["alpha"]  = c.alpha_dynamic_rad;
        cd["jounce"] = c.wheel_jounce_m;
        cd["camber_deg"] = c.camber_rad * (180.0 / M_PI);
        cd["toe_deg"]    = c.toe_rad * (180.0 / M_PI);
        cd["mr"]         = c.motion_ratio;
        cd["rpm"]        = state_.omega_wheel[i] * (60.0 / (2.0 * M_PI));
        cd["f_pushrod"]  = c.joint_reactions.f_pushrod_N;
        cd["f_damper"]   = c.joint_reactions.f_damper_coilover_N;
        cd["f_tie_rod"]  = c.joint_reactions.f_tie_rod_N;
        d[names[i]] = cd;
    }
    return d;
}

void Ter26VehicleBridge::set_powertrain_mode(int mode_idx) {
    auto p = engine_.parameters();
    if (mode_idx == 1) {
        p.powertrain.mode = ter26::PowertrainParams::Mode::PLANETARY_IN_HUB_4WD;
        p.powertrain.mounting = ter26::PowertrainParams::MountingLocation::UPRIGHT_IN_HUB_PLANETARY;
    } else {
        p.powertrain.mode = ter26::PowertrainParams::Mode::TER26_RWD_DUAL_MOTOR;
        p.powertrain.mounting = ter26::PowertrainParams::MountingLocation::CHASSIS_INBOARD_CV;
    }
    engine_.set_parameters(p);
}

void initialize_ter26_vehicle_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    ClassDB::register_class<Ter26VehicleBridge>();
}

void uninitialize_ter26_vehicle_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {
GDExtensionBool GDE_EXPORT ter26_vehicle_library_init(
    GDExtensionInterfaceGetProcAddress p_get_proc_address,
    const GDExtensionClassLibraryPtr p_library,
    GDExtensionInitialization* r_initialization)
{
    godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_ter26_vehicle_module);
    init_obj.register_terminator(uninitialize_ter26_vehicle_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}
}

} // namespace godot
