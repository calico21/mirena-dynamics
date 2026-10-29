"""
inspection.launch.py -- the INSPECTION mission, start to finish.

The car sits on stands, the steering is swept and the wheels are spun to a
target speed, to demonstrate that both actuators respond. There is no track,
nothing to perceive, nothing to map and nothing to plan, so this mission
launches exactly one node: the inspection controller. That is the whole point of
a launch file per mission -- an inspection run costs one process, not a pipeline
sitting idle behind it.

The drivetrain runs closed-loop on the CAN bridge's wheel speeds, so the one
thing this mission does depend on is /sensors/wheel_speeds arriving. Without it
the controller brakes rather than spinning the wheels open-loop, reports
NOT_WORKING, and still completes its steering sweep and ends the mission.

Every parameter the node accepts is stated below at its default value, rather
than left implicit -- see the other mission files for why. The numbers here are
the ones the scrutineers ask to see changed, so a profile that has to be
reconstructed from the binary is one nobody can change on the day.

The sweep is timed from AS DRIVING, not from launch: the node subscribes to
/as_status and starts its clock there, because the seconds the rules ask for are
seconds of the wheel actually moving.

  ros2 launch mirena_supervisor inspection.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # -- Control, and nothing else ---------------------------------------
        Node(
            package='mirena_control',
            executable='mirena_control_inspection',
            name='mirena_control',
            parameters=[{
                'use_sim_time': use_sim_time,
                # -- The steering sweep
                'inspection_start_delay': 3.0,                 # s braked after DRIVING, before the sweep starts
                'inspection_duration': 25.0,                   # s of sweep before the run is done
                'inspection_frequency': 0.5,                   # Hz
                'inspection_amplitude': 0.3,                   # rad, ~17 deg at the wheel
                # -- The drivetrain, held at a target wheel speed by a PID closing on the
                #    mean of the two rear wheel speeds off the CAN bus
                'inspection_target_rpm': 200.0,                # RPM, the number on the scrutineering sheet
                'inspection_rpm_kp': 0.0015,
                'inspection_rpm_ki': 0.002,
                'inspection_rpm_kd': 0.0001,
                'inspection_rpm_tau_d': 0.08,                  # s, low-pass on the derivative
                # -- The loop authority. gas_max is what limits the reachable speed:
                #    raise it if the target cannot be met
                'inspection_gas_max': 0.5,
                'inspection_gas_min': -0.5,
                'inspection_gas_rate': 2.0,                    # per second, so a far target is not a step on the pedal
                'inspection_brake_gas': -1.0,                  # held once the window closes and whenever the loop cannot run
                'inspection_brake_duration': 5.0,              # s of braking after the sweep, before mission_finished
                # -- Feedback freshness. Older than this and the loop brakes rather than
                #    driving the drivetrain open-loop on a stale measurement
                'wheel_speed_timeout': 0.2,                    # s
                # -- Loop rates
                'command_period': 0.01,                        # s, 100 Hz
                'heartbeat_period': 0.1,                       # s, 10 Hz supervisor report
            }],
        ),
    ])
