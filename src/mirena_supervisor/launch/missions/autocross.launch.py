"""
autocross.launch.py -- the AUTOCROSS mission, start to finish.

This file *is* the mission definition: which nodes run and with what parameters.
mirena_supervisor starts it through the launch API when the AS selects this
mission, and shuts it down when the run ends, so the whole pipeline lives
exactly as long as the mission does.

Every parameter each node accepts is stated below at its default value, rather
than left implicit. The point is that this file, and nothing else, is what the
mission ran with: a default that changes in a node's source changes what the car
does without any file here being touched, and a run whose parameters have to be
reconstructed from three packages is one nobody can reproduce a week later. The
cost is that these files and the nodes' declared defaults must be kept in step.

Nothing here is a lifecycle node: a node's lifetime is the mission's, so
"configured but inactive" has nowhere left to live. The supervisor starts this
file on AS READY and holds the mission in CONFIGURING until every node below is
publishing a heartbeat. Commands go out from the moment there is a solution --
the car is what decides whether to act on them, and it does not until its own
state machine reaches DRIVING.

use_sim_time is the only argument, forwarded by the supervisor from whichever
top-level launch file started it.

  ros2 launch mirena_supervisor autocross.launch.py use_sim_time:=true
"""

from math import radians

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # The lateral PID gains are launch arguments so a tuning sweep can override
    # them from the command line. Defaults are the values this mission has
    # always run with. Launch arguments are strings; launch_ros YAML-parses
    # them back ('0.3' -> 0.3), exactly as it already does for use_sim_time.
    cross_track_kp = LaunchConfiguration('cross_track_kp')
    cross_track_ki = LaunchConfiguration('cross_track_ki')
    cross_track_kd = LaunchConfiguration('cross_track_kd')
    cross_track_tau_d = LaunchConfiguration('cross_track_tau_d')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cross_track_kp', default_value='0.4'),
        DeclareLaunchArgument('cross_track_ki', default_value='0.02'),
        DeclareLaunchArgument('cross_track_kd', default_value='0.1'),
        DeclareLaunchArgument('cross_track_tau_d', default_value='0.06'),

        # -- State estimation ------------------------------------------------
        # Owns /state/car and the odom->car/cog TF.
        Node(
            package='mirena_state',
            executable='mirena_state_ekf',
            name='mirena_state',
            parameters=[{
                'use_sim_time': use_sim_time,
                # -- Timeouts and prediction
                'odom_timeout_sec': 0.5,                       # s without INS odometry before it is not trusted
                'imu_timeout_sec': 0.2,                        # s without IMU before it is not trusted
                'max_predict_dt_sec': 0.1,                     # s, longest gap the EKF will predict across
                # -- Noise model. Continuous-time process noise over [x, y, psi, u, v, omega]
                'process_noise_diag': [0.01, 0.01, 0.005, 0.5, 0.5, 0.5],
                # -- Floor on INS measurement variances, which guards against zero covariances
                'odom_meas_floor_diag': [1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2],
                # Seeds P at init; the odom frame's own pose has zero
                # reported variance by construction.
                'init_cov_floor_diag': [0.01, 0.01, 0.001, 0.01, 0.01, 0.001],
                'gyro_meas_var': 1.0e-5,                       # rad^2/s^2
                # -- INS disagreement gate. An INS solution this far from the estimate -- the
                # innovation the update is about to apply -- is absorbed by re-anchoring
                # T_odom <- ins rather than applied to the pose, so /state/car never
                # teleports when the solution returns from a dropout. <= 0 disables it.
                'ins_reanchor_m': 12.0,                      # m of disagreement that triggers a re-anchor
            }],
        ),

        # -- Perception ------------------------------------------------------
        Node(
            package='mirena_perception',
            executable='mirena_perception',
            name='mirena_perception',
            parameters=[{
                'use_sim_time': use_sim_time,
                'enable_debug': True,                          # publish the clustered/standing point clouds
                # -- Finding candidates
                'coarse_cell': 0.4,                            # m, cell of the throwaway first ground guess
                'coarse_ring': 1.2,                            # m, how far that pass looks for a lower point
                'candidate_height': 0.08,                      # m above coarse ground to be worth considering
                'peak_cell': 0.25,                             # m, grid the candidates are thinned on
                'peak_height': 0.15,                           # m a candidate must reach to be one at all
                # -- The car looking at itself
                'ego_half_length': 2.0,                        # m
                'ego_half_width': 1.2,                         # m
                # -- The ground plane fitted under one candidate
                'ring_inner': 0.55,                            # m, clears the cone itself
                'ring_outer': 2.6,                             # m, far enough to pin a slope
                'ring_cell': 0.4,                              # m, the ring is reduced to one low point per cell
                # -- What a cone looks like. A small cone is 0.325 m, a big one 0.505 m
                'min_top': 0.18,                               # m above the fitted plane
                'max_top': 0.58,                               # m above the fitted plane
                'core_radius': 0.32,                           # m the blob is measured in
                'stand_height': 0.1,                           # m that counts as standing rather than ground
                'max_footprint': 0.55,                         # m, widest the standing part may be
                'min_points': 2,                               # fewest points a detection may be built from
                'max_roughness': 0.045,                        # m, sd of the plane fit residual; separates cones from vegetation
                'max_ring_ratio': 1.2,                         # how much of the ring may also be standing up
                'max_range': 30.0,                             # m, furthest a cone is claimed
                'big_cone_min_height': 0.4,                    # m, tall enough to be a start-line cone
                # -- How well a detection is located. Measured, not assumed
                'range_sigma': 0.08,                           # m along the beam
                'bearing_sigma': 0.01,                         # rad across the beam
                'cluster_spread': 0.3,                         # m
            }],
        ),

        # -- SLAM ------------------------------------------------------------
        Node(
            package='mirena_slam',
            executable='mirena_slam',
            name='mirena_slam',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_debug': True,                         # publish the map and association markers
                # -- Association. One gate answers every "is this the thing I already have" question
                'mahalanobis_threshold': 5.99,                 # chi-square gate on squared Mahalanobis distance; 95th pct, 2 DoF
                'max_association_distance': 5.5,               # m, Euclidean backstop applied before the chi-square test
                'local_map_radius': 20.0,                      # m the car looks; also defines "should have been seen"
                'view_half_angle': 1.05,                       # rad, half-angle of the sensor field of view
                # -- Measurements
                'default_measurement_sigma': 0.10,             # m, used when perception reports no covariance
                'min_measurement_sigma': 0.03,                 # m, floor; also what keeps the association gate open
                # -- Existence, in log-odds. Both measurable from a labelled run
                'p_detect': 0.55,                              # how often perception reports a cone genuinely in view
                'p_false': 0.05,                               # how often it reports one where there is none
                'promote_log_odds': 9.0,                       # evidence before a candidate is written into the graph (~4 looks)
                'publish_log_odds': 1.0,                       # evidence to stay in the published map; below promote, for hysteresis
                # -- Promotion
                'separability_k': 2.0,                         # sigmas of separation before two estimates are two objects
                'max_candidate_age': 20,                       # keyframes a candidate may stay unresolved before promoting anyway
                'max_promotion_factors': 10,                   # most measurements one promotion may bring into the graph
                # -- When a pose is worth its keep
                'keyframe_odom_sigma': 0.30,                   # m of odometry drift before the pose needs pinning
                'keyframe_max_frames': 4,                      # frames between keyframes, at most, whatever else is true
                # -- Node timing
                'map_publish_period': 0.1,                     # s, 10 Hz map republish
                'tf_publish_period': 0.01,                     # s, 100 Hz map->odom; must keep up with odom->car/cog
                'heartbeat_period': 0.1,                       # s, 10 Hz supervisor report
                'max_sync_delta_ms': 10.0,                     # ms; pose/detection pairs further apart are dropped
                'report_stale_after_s': 1.0,                   # s without a keyframe before the report reads PARTIAL
                'keyframe_max_interval_s': 0.5,                # s, longest interval the odometry noise is integrated over
                'correction_time_constant_s': 0.5,             # s, easing filter on the published map->odom
                # -- Odometry motion model: the floor under every keyframe factor, and what
                #    governs normal driving; velocity-derived drift overrides it only on INS dropouts
                'odom_model_xy_per_m': 0.05,                   # sigma m per m travelled
                'odom_model_xy_per_rad': 0.10,                 # sigma m per rad turned
                'odom_model_th_per_m': 0.02,                   # sigma rad per m travelled
                'odom_model_th_per_rad': 0.05,                 # sigma rad per rad turned
                'odom_sigma_floor_xy': 0.02,                   # m
                'odom_sigma_floor_th': 0.005,                  # rad
            }],
        ),

        # -- Planning --------------------------------------------------------
        Node(
            package='mirena_planning',
            executable='mirena_planning',
            name='mirena_planning',
            parameters=[{
                'use_sim_time': use_sim_time,
                'mission': 'autocross',
                'fallback_frame': 'map',                       # used when the map frame is not yet available
                'plan_period': 0.1,                            # s, 10 Hz replan
                'heartbeat_period': 0.1,                       # s, 10 Hz supervisor report
                'publish_debug': True,                         # publish the Delaunay/track debug markers
                'republish_tolerance': 0.25,                   # m a gate must move before the track is republished
                # -- Track geometry the search is allowed to believe in
                'min_track_width': 2.0,                        # m; a rail against unmerged duplicate landmarks
                'max_track_width': 5.0,                        # m; top of the range that works, not the middle
                'cone_spacing': 5.0,                           # m, ceiling set by the rules
                'max_gate_gap': 8.5,                           # m between consecutive gates
                # -- The search itself
                'curvature_weight': 1.0,                       # what bending the centreline costs against the gate it earns
                'lookahead': 5,                                # gates ahead before committing; odd on purpose
                'max_path_points': 400,
                # -- Closing the loop
                'start_gate_radius': 10.0,                     # m around the map origin searched for the start line
                'closure_hysteresis': 5,                       # consecutive agreements before the loop is called closed
            }],
        ),
        # -- Control ---------------------------------------------------------
        # One lap of the unknown circuit, then stop past the line.
        Node(
            package='mirena_control',
            executable='mirena_control_pid',
            name='mirena_control',
            parameters=[{
                'use_sim_time': use_sim_time,
                'laps': 1,
                # -- Mission end. Laps are counted from arc-length progress along the centreline
                'stop_distance': 5.0,                         # m past the final crossing; closed tracks only
                'min_path_length': 5.0,                        # m, floor on the reference window
                'stop_speed_eps': 0.3,                         # m/s below which the car counts as stationary
                'stop_hold_time': 0.5,                         # s stationary before the run is reported done
                # -- Reference window
                'lookahead_base': 15.0,                        # m
                'lookahead_gain': 1.2,                         # m per (m/s)
                # -- Frames
                'robot_frame': 'car/cog',
                'work_frame': 'odom',
                'map_frame': 'map',
                # -- Loop rates. solver_period paces guidance; the PID closes at command_period
                'solver_period': 0.05,                         # s, 20 Hz guidance
                'command_period': 0.01,                        # s, 100 Hz command
                'heartbeat_period': 0.1,                       # s, 10 Hz supervisor report
                'tf_timeout': 0.005,                           # s
                # -- Centreline projection search
                'projection_window_back': 5.0,                 # m
                'projection_window_forward': 20.0,             # m
                'projection_coarse_step': 0.25,                # m
                # -- Safety envelope. Every reason to stop producing plans ends in braking
                'state_timeout': 0.15,                         # s; /state/car arrives at 100 Hz
                'plan_timeout': 0.25,                          # s without a new plan before full braking
                'mission_timeout': 600.0,                      # s of driving; backstop for a loop that never closes
                # -- Guards on the progress estimate, which a SLAM jump would otherwise move
                'finish_confirm_count': 3,                     # consecutive solves agreeing the car is past the line
                'progress_rate_margin': 1.5,                   # multiple of |u| * dt progress may advance by
                'progress_rate_slack': 0.5,                    # m added to that allowance
                'publish_markers': True,                       # reference and predicted-path markers
                # -- Longitudinal: speed error -> gas
                'speed_kp': 0.35,
                'speed_ki': 0.15,
                'speed_kd': 0.02,
                'speed_tau_d': 0.05,                           # s, low-pass on the derivative
                # -- Lateral: cross-track error -> steering angle, rad per m
                'cross_track_kp': cross_track_kp,
                'cross_track_ki': cross_track_ki,
                'cross_track_kd': cross_track_kd,
                'cross_track_tau_d': cross_track_tau_d,  # s
                'heading_gain': 0.8,                           # rad of steer per rad of heading error; this is the damping
                'curvature_feedforward': 1.0,                  # fraction of atan(wheelbase * kappa) supplied open-loop
                'wheelbase': 1.7,                             # m, L_F + L_R of the model in codegen/dbm_model.py
                'cross_track_limit': 3.0,                      # m; error saturates here before it reaches the PID
                # -- Actuator envelope, matching the bounds the OCP puts on delta and g
                'steer_limit': radians(30.0),                  # rad
                'steer_rate': radians(5.0 * 30.0),             # rad/s
                'gas_rate': 5.0,                               # per second
                # -- Speed profile, which the MPC gets from its cost function instead
                'max_speed': 8.0,                             # m/s ceiling regardless of geometry
                'min_speed': 0.2,                              # m/s floor away from a stop, so the car keeps rolling
                'max_lateral_accel': 2.0,                      # m/s^2 the tyres are trusted with in a corner
                'max_decel': 2.0,                              # m/s^2 planned for a corner or the finish
                'track_end_margin': 3.0,                       # m short of the last mapped gate the braking curve aims
            }],
        ),
        #Node(
        #        package='mirena_control',
        #        executable='mirena_control_mpc',
        #        name='mirena_control',
        #        parameters=[{
        #            'use_sim_time': use_sim_time,
        #            'laps': 1,
        #            # -- Mission end. Laps are counted from arc-length progress along the centreline
        #            'stop_distance': 20.0,                         # m past the final crossing; closed tracks only
        #            'min_path_length': 5.0,                        # m, floor on the reference window
        #            'stop_speed_eps': 0.3,                         # m/s below which the car counts as stationary
        #            'stop_hold_time': 0.5,                         # s stationary before the run is reported done
        #            # -- Reference window
        #            'lookahead_base': 15.0,                        # m
        #            'lookahead_gain': 1.2,                         # m per (m/s)
        #            # -- Frames
        #            'robot_frame': 'car/cog',
        #            'work_frame': 'odom',
        #            'map_frame': 'map',
        #            # -- Loop rates
        #            'solver_period': 0.05,                         # s, 20 Hz NLP solve
        #            'command_period': 0.01,                        # s, 100 Hz command
        #            'heartbeat_period': 0.1,                       # s, 10 Hz supervisor report
        #            'tf_timeout': 0.005,                           # s
        #            # -- Centreline projection search
        #            'projection_window_back': 5.0,                 # m
        #            'projection_window_forward': 20.0,             # m
        #            'projection_coarse_step': 0.25,                # m
        #            # -- Safety envelope. Every reason to stop producing plans ends in braking
        #            'state_timeout': 0.15,                         # s; /state/car arrives at 100 Hz
        #            'plan_timeout': 0.25,                          # s without a new plan before full braking
        #            'mission_timeout': 600.0,                      # s of driving; backstop for a loop that never closes
        #            # -- Guards on the progress estimate, which a SLAM jump would otherwise move
        #            'finish_confirm_count': 3,                     # consecutive solves agreeing the car is past the line
        #            'progress_rate_margin': 1.5,                   # multiple of |u| * dt progress may advance by
        #            'progress_rate_slack': 0.5,                    # m added to that allowance
        #            # -- Accepting a solve that did not fully converge
        #            'max_accepted_residual': 0.1,                  # 0 accepts only fully converged solves
        #            'publish_markers': True,                       # horizon and reference markers
        #        }],
        #     ),
    ])
