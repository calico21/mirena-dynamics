#pragma once
/**
 * @file vehicle_model_binding.hpp
 * @brief Godot 4.7 GDExtension Class `Ter26VehicleBridge` exposing the C++20
 *        14-DOF Multibody Vehicle Dynamics Engine to GDScript (`vehicleMotion.gd`).
 */

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>

#include "ter26/multibody/vehicle_dynamics_engine.hpp"

namespace godot {

class Ter26VehicleBridge : public RefCounted {
    GDCLASS(Ter26VehicleBridge, RefCounted)

protected:
    static void _bind_methods();

public:
    Ter26VehicleBridge();
    ~Ter26VehicleBridge() override = default;

    /// Reset vehicle to static equilibrium at the specified Godot world Transform3D
    void reset_to_transform_godot(const Transform3D& transform_godot);

    /// Step the 14-DOF multibody model by dt_s using Godot-frame ground queries and control inputs
    Dictionary step_godot(
        double dt_s,
        double throttle,
        double brake,
        double steering_rad,
        bool ebs_active,
        const PackedFloat64Array& ground_points_godot,
        const PackedFloat64Array& ground_normals_godot);

    /// Apply a discrete external collision impulse (e.g. cone or barrier hit) in Godot world coordinates
    Vector3 apply_collision_impulse_godot(
        const Vector3& contact_point_godot,
        const Vector3& contact_normal_godot,
        const Vector3& other_vel_godot,
        double other_mass_kg,
        double restitution);

    /// Get chassis Transform3D in Godot world coordinates
    Transform3D get_chassis_transform_godot() const;

    /// Get chassis linear velocity in Godot world coordinates [m/s]
    Vector3 get_chassis_linear_velocity_godot() const;

    /// Get chassis angular velocity in Godot world coordinates [rad/s]
    Vector3 get_chassis_angular_velocity_godot() const;

    /// Get local Transform3D of wheel corner_idx (0=FL, 1=FR, 2=RL, 3=RR) relative to chassis
    Transform3D get_wheel_local_transform_godot(int corner_idx) const;

    /// Get wheel rotational speed in RPM for ROS 2 /sensors/wheel_speeds
    double get_wheel_rpm(int corner_idx) const;

    /// Get detailed telemetry dictionary for ROS 2 /debug/state/car and UI
    Dictionary get_diagnostics_dictionary() const;

    /// Switch between TeR26 RWD Dual-Motor (0) and 4WD In-Hub Planetary Gearbox (1)
    void set_powertrain_mode(int mode_idx);

private:
    ter26::VehicleDynamicsEngine engine_;
    ter26::VehicleState state_;
    ter26::VehicleStepResult last_result_;
};

} // namespace godot
