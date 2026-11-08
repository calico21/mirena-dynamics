"""
sim.launch.py -- the whole system, against the simulator.

  ros2 launch mirena_supervisor sim.launch.py

Starts two things and stops there: the simulator, and the supervisor. The
autonomy pipeline is deliberately *not* here. The supervisor brings it up when
the AS selects a mission -- launching launch/missions/<mission>.launch.py on
AS READY and holding the mission in CONFIGURING until every node in it is
beating -- and tears it down when the run ends. So a skidpad run gets the skidpad
pipeline, an inspection run gets one node, and the system is ready for the next
mission without being restarted.

Its counterpart is hardware.launch.py, which is the same file with the drivers
where the simulator is. The only thing that differs downstream is use_sim_time,
which is forwarded to every mission from here.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # -- Data source ------------------------------------------------------
        Node(
            package='mirena_sim',
            executable='mirena_sim',
            parameters=[{'use_sim_time': True, 'publish_sim_time': True}],
        ),

        # -- Supervisor -------------------------------------------------------
        # Runs for the life of the session; the pipeline comes and goes beneath
        # it, one launch file per mission.
        Node(
            package='mirena_supervisor',
            executable='mirena_supervisor',
            name='mirena_supervisor',
            parameters=[{
                'use_sim_time': True,
                'mission_launch_package': 'mirena_supervisor',
                'mission_launch_directory': 'launch/missions',
                'mission_launch_arguments': ['use_sim_time:=true'],
            }],
        ),
    ])
