"""Mission supervisor for the Mirena driverless car.

Watches the AS state machine and runs one mission's worth of autonomy per run.
A mission is a launch file: selecting SKIDPAD starts missions/skidpad.launch.py,
selecting INSPECTION starts a launch file with one node in it, and ending a run
shuts that launch file down again.

    AS READY, mission selected  ->  launch it                       CONFIGURING
    every node is beating       ->                                  READY
    AS DRIVING                  ->                                  RUNNING
    any node reports finished   ->                                  FINISHED, held
    AS EMERGENCY / node death   ->                                  FAILED, held

The states report, they do not gate. CONFIGURING means the pipeline is still
coming up; READY means it is up and the car may be released; RUNNING is the
confirmation that the car has taken us up on it. Nothing here holds the
controller back -- the car's own state machine ignores our commands until it
reaches DRIVING, and that is the interlock that matters.

FINISHED and FAILED are terminal and latched: the launch file is torn down but
the status keeps publishing, because the verdict on a run is what someone at the
car needs to read afterwards. Selecting the next mission is what clears it.

The end of a run is whichever node says so. Any node of the mission may put
`mission_finished: true` in its heartbeat report; the controller does that today,
but that is the controller's business and is not encoded here. Nor is the
mission's node list: the pipeline is whoever publishes on /health, discovered
from the graph, so adding a node to a mission is one edit in its launch file.
"""

import json
import os
import threading
from typing import Any, Dict, List, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.time import Time
from rclpy.timer import Timer
from ament_index_python.packages import get_package_share_directory

from mirena_common.msg import Heartbeat, AsInfo, MissionInfo

from mirena_supervisor.mission_launcher import MissionLauncher

# The topic every pipeline node reports on. Publishing here is what makes a node
# part of the mission as far as this supervisor is concerned.
HEALTH_TOPIC: str = '/health'

# Any node of the mission setting this in its report ends the run.
MISSION_FINISHED_KEY: str = 'mission_finished'

# The run's telemetry, forwarded to the car on DV_info and read off the RES
# display. Like MISSION_FINISHED_KEY these are keys, not nodes: whoever is best
# placed to know a number publishes it, and the supervisor copies it across
# without knowing which node that was. Today perception reports the cones it can
# see, SLAM the size of the map it has built, and the controller the laps it has
# closed, but that is their business and is not encoded here.
TELEMETRY_KEYS: Dict[str, str] = {
    'lap_counter': 'laps',
    'cones_count_actual': 'cones_count_actual',
    'cones_count_total': 'cones_count_total',
}

# A mission is in flight in these and nowhere else. INACTIVE before the first
# run, FINISHED and FAILED after one: all three mean no pipeline is running and
# the next mission may be started.
ACTIVE_STATUSES = (
    MissionInfo.MISSION_STATUS_CONFIGURING,
    MissionInfo.MISSION_STATUS_READY,
    MissionInfo.MISSION_STATUS_RUNNING,
)

# AsInfo mission enum -> launch file basename under launch/missions/.
MISSION_LAUNCH_FILES: Dict[int, str] = {
    AsInfo.AS_MISSION_ACCELERATION: 'acceleration',
    AsInfo.AS_MISSION_SKIDPAD: 'skidpad',
    AsInfo.AS_MISSION_TRACKDRIVE: 'trackdrive',
    AsInfo.AS_MISSION_INSPECTION: 'inspection',
    AsInfo.AS_MISSION_AUTOCROSS: 'autocross',
    AsInfo.AS_MISSION_BRAKE_TEST: 'brake_test',
    AsInfo.AS_MISSION_NONE: 'none',
}


class MirenaSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('mirena_supervisor')

        # Where the mission launch files live, and what to forward to them. The
        # top-level launch file sets these, which is how a simulated run reaches
        # the missions as use_sim_time:=true without the supervisor knowing
        # anything about simulation.
        self.declare_parameter('mission_launch_package', 'mirena_supervisor')
        self.declare_parameter('mission_launch_directory', 'launch/missions')
        self.declare_parameter('mission_launch_arguments', [''])
        # How long the pipeline gets to come up before its silence is a failure.
        # Covers the slowest node's startup: loading a model, building a solver.
        self.declare_parameter('startup_timeout_sec', 60.0)
        # How long the set of heartbeat publishers must hold still before it is
        # taken to be the whole pipeline.
        self.declare_parameter('discovery_settle_sec', 2.0)
        # A running mission with any node quiet for this long is dead.
        self.declare_parameter('heartbeat_timeout_sec', 3.0)

        self._launch_dir = os.path.join(
            get_package_share_directory(
                self.get_parameter('mission_launch_package').value),
            self.get_parameter('mission_launch_directory').value)
        # Passed through to every mission launch file as-is. A string array
        # rather than a dict parameter because ROS 2 has no map parameter type,
        # and this way the top-level launch file writes the same `name:=value`
        # a human would type.
        self._launch_arguments = [
            a for a in self.get_parameter('mission_launch_arguments').value if a]
        self._startup_timeout = self.get_parameter('startup_timeout_sec').value
        self._discovery_settle = self.get_parameter('discovery_settle_sec').value
        self._heartbeat_timeout = self.get_parameter('heartbeat_timeout_sec').value

        self._as_status: int = AsInfo.AS_STATUS_OFF
        self._mission_info: MissionInfo = MissionInfo()
        self._mission_info.status = MissionInfo.MISSION_STATUS_INACTIVE

        self._launcher = MissionLauncher(self.get_logger())
        self._launched_at: Optional[Time] = None
        # Who has ever reported this run, and when they last did.
        self._last_report: Dict[str, Time] = {}
        # The heartbeat publishers seen so far, and when that set last changed:
        # CONFIGURING ends when it has held still long enough to believe it is
        # the whole pipeline. Not _publishers -- rclpy's Node owns that name.
        self._pipeline: Set[str] = set()
        self._pipeline_changed_at: Optional[Time] = None

        self._as_sub: Subscription = self.create_subscription(
            AsInfo, '/as_status', self.as_info_callback, 10)
        self._health_sub: Subscription = self.create_subscription(
            Heartbeat, HEALTH_TOPIC, self.health_callback, 10)
        self._mission_info_pub: Publisher = self.create_publisher(
            MissionInfo, '/system/mission_status', 10)

        self._monitor_timer: Timer = self.create_timer(0.1, self.monitor)

        self.get_logger().info(f"Mirena Supervisor ready. Missions: {self._launch_dir}")

    # ------------------------------------------------------------- transitions

    def as_info_callback(self, msg: AsInfo) -> None:
        previous, self._as_status = self._as_status, msg.status

        if msg.status == AsInfo.AS_STATUS_EMERGENCY:
            # A run that already ended is not turned into a failure by a later
            # emergency: FINISHED is the truth about what happened.
            if self._mission_info.status in ACTIVE_STATUSES:
                self.get_logger().error('Emergency signalled by the AS.')
                self.fail_mission()
            return

        if msg.status == previous:
            return

        if msg.status == AsInfo.AS_STATUS_READY:
            if self._mission_info.status not in ACTIVE_STATUSES:
                self.start_mission(msg.mission)

        elif msg.status == AsInfo.AS_STATUS_DRIVING:
            # Purely a report: the car has released itself and is acting on what
            # we publish. Nothing on our side changes behaviour here.
            if self._mission_info.status == MissionInfo.MISSION_STATUS_READY:
                self.get_logger().info('AS is DRIVING. Mission RUNNING.')
                self._mission_info.status = MissionInfo.MISSION_STATUS_RUNNING
            elif self._mission_info.status == MissionInfo.MISSION_STATUS_CONFIGURING:
                # The car went before the pipeline was up. Nothing useful can
                # come of the run, and it is already moving.
                self.get_logger().error('AS reached DRIVING while still configuring.')
                self.fail_mission()

    def start_mission(self, mission_id: int) -> None:
        # A terminal status is held so it can be read at the car, and the run's
        # counters are held with it. Selecting the next mission is what clears
        # both -- before the launch file is looked for, so that a mission that
        # never starts cannot leave the last run's laps standing under its
        # verdict, which would read as laps this car has not driven.
        for field in TELEMETRY_KEYS:
            setattr(self._mission_info, field, 0)

        name = MISSION_LAUNCH_FILES.get(mission_id)
        launch_file = os.path.join(self._launch_dir, f'{name}.launch.py') if name else None
        if launch_file is None or not os.path.exists(launch_file):
            self.get_logger().error(f"No mission launch file for mission id {mission_id}.")
            self.fail_mission()
            return

        self._last_report = {}
        self._pipeline = set()
        self._pipeline_changed_at = None
        self._mission_info.status = MissionInfo.MISSION_STATUS_CONFIGURING

        if not self._launcher.start(name, launch_file, self._launch_arguments):
            self.fail_mission()
            return

        self._launched_at = self.get_clock().now()

    # ---------------------------------------------------------------- monitor

    def health_callback(self, msg: Heartbeat) -> None:
        """Every heartbeat is liveness; any of them may end the run."""
        if self._mission_info.status not in ACTIVE_STATUSES:
            return

        self._last_report[msg.node_name] = self.get_clock().now()

        if self._mission_info.status != MissionInfo.MISSION_STATUS_RUNNING:
            return
        try:
            report: Dict[str, Any] = json.loads(msg.report)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Malformed JSON report from '{msg.node_name}'.")
            return
        self._collect_telemetry(msg.node_name, report)

        if report.get(MISSION_FINISHED_KEY) is True:
            self.get_logger().info(f"'{msg.node_name}' reported the run complete.")
            self.finish_mission()

    def _collect_telemetry(self, node_name: str, report: Dict[str, Any]) -> None:
        """Copy whatever of the run's telemetry this report carries.

        A node that does not know a number leaves the key out and the last value
        stands: these are counters on a display, and blanking one back to zero
        because the node that owns it happened to beat without it would read as
        the car having lost its map. The fields are uint32 on the wire, so a
        value that is not a whole non-negative number is dropped with a warning
        rather than thrown at the message and taken down with it.
        """
        for field, key in TELEMETRY_KEYS.items():
            if key not in report:
                continue
            value = report[key]
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                # Throttled: a node wrong about this is wrong on every beat, and
                # ten lines a second of it would bury the log the run is read from.
                self.get_logger().warning(
                    f"'{node_name}' reported {key}={value!r}, "
                    f"which is not a count; ignoring it.",
                    throttle_duration_sec=5.0)
                continue
            setattr(self._mission_info, field, value)

    def monitor(self) -> None:
        self._mission_info_pub.publish(self._mission_info)

        if self._mission_info.status not in ACTIVE_STATUSES:
            return

        # The launch file exiting on its own means a node died or refused to
        # start -- a mission cannot end that way, only be torn down by us.
        if not self._launcher.is_running():
            self.get_logger().error('The mission launch exited on its own.')
            self.fail_mission()
            return

        now = self.get_clock().now()

        if self._mission_info.status == MissionInfo.MISSION_STATUS_CONFIGURING:
            self._check_configuring(now)
        else:
            self._check_alive(now)

    def _check_configuring(self, now: Time) -> None:
        """CONFIGURING is the wait for the pipeline to start beating.

        There is no list of nodes to check against -- the pipeline is whoever
        publishes on /health -- so the end of the wait is the publisher set
        holding still. Nodes come up at wildly different speeds (perception
        loads a TensorRT engine, control builds an acados solver), and a node
        only starts beating once it is genuinely up, so a set that has stopped
        growing is the whole mission.
        """
        found = self._health_publishers()
        if found != self._pipeline:
            self._pipeline = found
            self._pipeline_changed_at = now
            if found:
                self.get_logger().info(f"Pipeline so far: {', '.join(sorted(found))}")

        waited = self._seconds_since(self._launched_at, now)
        if waited > self._startup_timeout:
            missing = 'nothing has' if not found else f"only {', '.join(sorted(found))} have"
            self.get_logger().error(
                f"Pipeline did not come up within {self._startup_timeout:.0f}s: {missing} reported.")
            self.fail_mission()
            return

        # Every publisher must also have actually been heard from: a node can be
        # advertising /health a moment before its first message lands.
        if not found or not found.issubset(self._last_report.keys()):
            return
        if self._seconds_since(self._pipeline_changed_at, now) < self._discovery_settle:
            return

        self.get_logger().info(
            f"All {len(found)} pipeline nodes beating. Mission READY.")
        self._mission_info.status = MissionInfo.MISSION_STATUS_READY

    def _check_alive(self, now: Time) -> None:
        """Past CONFIGURING, a node that stops beating kills the mission."""
        for node_name, last in self._last_report.items():
            silent = self._seconds_since(last, now)
            if silent > self._heartbeat_timeout:
                self.get_logger().error(
                    f"'{node_name}' silent for {silent:.1f}s; the mission is dead.")
                self.fail_mission()
                return

    def _health_publishers(self) -> Set[str]:
        """Node names of everything publishing on /health.

        get_publishers_info_by_topic is the supported way to ask the graph who
        is on a topic, which is what makes the pipeline self-describing.

        An endpoint is visible before the participant info that names it has
        arrived, and shows up as a placeholder until then. Those are dropped:
        one can never be matched against a heartbeat, so counting it would hold
        the mission in CONFIGURING for as long as it lingered.
        """
        try:
            info = self.get_publishers_info_by_topic(HEALTH_TOPIC)
        except Exception as e:
            self.get_logger().warning(f"Could not read {HEALTH_TOPIC} publishers: {e}")
            return set()
        return {i.node_name for i in info if not i.node_name.startswith('_NODE_NAME_UNKNOWN')}

    @staticmethod
    def _seconds_since(stamp: Optional[Time], now: Time) -> float:
        return 0.0 if stamp is None else (now - stamp).nanoseconds / 1e9

    # ------------------------------------------------------------------- end

    def finish_mission(self) -> None:
        if self._mission_info.status == MissionInfo.MISSION_STATUS_FINISHED:
            return
        self.get_logger().info('Mission FINISHED.')
        self._mission_info.status = MissionInfo.MISSION_STATUS_FINISHED
        self._mission_info_pub.publish(self._mission_info)
        self._teardown()

    def fail_mission(self) -> None:
        if self._mission_info.status == MissionInfo.MISSION_STATUS_FAILED:
            return
        self.get_logger().fatal('ABORTING MISSION: system entered FAILED state.')
        self._mission_info.status = MissionInfo.MISSION_STATUS_FAILED
        self._mission_info_pub.publish(self._mission_info)
        self._teardown()

    def _teardown(self) -> None:
        """Stop the launch file, leaving the terminal status where it is.

        The stop runs on its own thread. Killing a mission's process group takes
        as long as its slowest node takes to die, and this node has a status to
        keep publishing at 10 Hz meanwhile -- going quiet at the exact moment a
        run ends is indistinguishable, from the car, from having crashed.

        What makes the supervisor ready for the next mission is the launcher
        being empty, not the status -- which is held until one is next selected.
        """
        self._launched_at = None
        self._last_report = {}
        self._pipeline = set()
        self._pipeline_changed_at = None
        threading.Thread(target=self._launcher.stop, daemon=True).start()

    def destroy_node(self) -> bool:
        self._launcher.stop()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MirenaSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt; shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
