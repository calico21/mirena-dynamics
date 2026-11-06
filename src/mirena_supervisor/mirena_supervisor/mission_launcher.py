"""Starting and stopping a mission's launch file from inside a running node.

The supervisor does not shell out to `ros2 launch`; it drives the same launch
API that command does. The catch is that a LaunchService owns an asyncio event
loop and is not designed to be started, shut down and started again inside one
process, which is exactly what a supervisor that runs several missions back to
back needs. So each mission gets its own child process with its own
LaunchService, and "clean up the mission" is "let that process exit".

That also means a mission that wedges cannot wedge the supervisor: the child is
signalled, and killed outright if it will not go.
"""

import multiprocessing as mp
import os
import signal
import threading
from typing import List, Optional

from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 'spawn' rather than the Linux default 'fork': the supervisor is a live rclpy
# node with threads and DDS sockets, and forking that into a child which then
# builds its own launch service is a reliable way to inherit a broken middleware
# state. A spawned child starts clean.
_ctx = mp.get_context('spawn')

# How long a mission is given to tear itself down before it is killed.
SHUTDOWN_GRACE_SEC = 10.0


def _run_launch_service(launch_file: str, launch_arguments: List[str], stop) -> None:
    """Child-process entry point: run one launch file until asked to stop."""
    # A mission's nodes should die with the mission, not outlive it as orphans
    # if this process is killed hard. Its own process group makes that a single
    # signal to a single group.
    os.setpgrp()

    # Every mission file is a .launch.py, so the Python source is the exact
    # loader for them -- no format sniffing needed.
    description = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments=[tuple(a.split(':=', 1)) for a in launch_arguments],
        )
    ])

    service = LaunchService(noninteractive=True)
    service.include_launch_description(description)

    def _wait_for_stop():
        stop.wait()
        service.shutdown()

    threading.Thread(target=_wait_for_stop, daemon=True).start()
    service.run(shutdown_when_idle=True)


class MissionLauncher:
    """One mission at a time. start() is a no-op if one is already running."""

    def __init__(self, logger):
        self._log = logger
        self._process: Optional[mp.Process] = None
        self._stop: Optional[mp.Event] = None
        self._mission: Optional[str] = None

    @property
    def mission(self) -> Optional[str]:
        """Name of the running mission, or None."""
        return self._mission if self.is_running() else None

    def is_running(self) -> bool:
        return self._process is not None and self._process.is_alive()

    def start(self, mission: str, launch_file: str, launch_arguments: List[str]) -> bool:
        if self.is_running():
            self._log.error(
                f"Refusing to start '{mission}': '{self._mission}' is still running.")
            return False

        # A previous mission that has already exited still needs reaping before
        # its slot can be reused.
        self._reap()

        self._log.info(
            f"Launching mission '{mission}': {launch_file} {' '.join(launch_arguments)}")

        self._stop = _ctx.Event()
        self._process = _ctx.Process(
            target=_run_launch_service,
            args=(launch_file, launch_arguments, self._stop),
            name=f'mission-{mission}',
            daemon=False,
        )
        self._process.start()
        self._mission = mission
        return True

    def stop(self) -> None:
        """Shut the mission down and leave the slot ready for the next one.

        Escalates: ask the launch service to shut down, then SIGTERM the child's
        process group, then SIGKILL it. Returns only once nothing is left.
        """
        if self._process is None:
            return

        if self._process.is_alive():
            self._log.info(f"Stopping mission '{self._mission}'...")
            if self._stop is not None:
                self._stop.set()
            self._process.join(SHUTDOWN_GRACE_SEC)

        if self._process.is_alive():
            self._log.warning(
                f"Mission '{self._mission}' did not stop in {SHUTDOWN_GRACE_SEC:.0f}s; "
                "terminating its process group.")
            self._signal_group(signal.SIGTERM)
            self._process.join(5.0)

        if self._process.is_alive():
            self._log.error(f"Mission '{self._mission}' would not terminate; killing.")
            self._signal_group(signal.SIGKILL)
            self._process.join(5.0)

        self._reap()
        self._log.info("Mission stopped; ready for the next one.")

    def _signal_group(self, sig: int) -> None:
        if self._process is None or self._process.pid is None:
            return
        try:
            os.killpg(os.getpgid(self._process.pid), sig)
        except (ProcessLookupError, PermissionError) as e:
            self._log.warning(f"Could not signal the mission process group: {e}")

    def _reap(self) -> None:
        proc = self._process
        if proc is not None and not proc.is_alive():
            try:
                proc.close()
            except ValueError:
                pass  # never started, or already closed
        self._process = None
        self._stop = None
        self._mission = None
