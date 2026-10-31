# stages.py — start / stop groups of mission nodes phase by phase
#
# The Pi 5 browned out (switched off) with every node, the OAK-D and the
# downward camera powering up at once. mission.launch.py now starts only the
# light nodes; mission_node starts each group here when its phase needs it,
# and stops it when the phase no longer does:
#
#   ros2 launch mission_bringup mission_stage.launch.py stage:=<name> <launch args>
#
#   oak            preflight -> landing   (exit corridor depth, live view, FC proximity)
#   down_cam       preflight -> landing   (phases 1, 3, 4 + live view)
#   arena          phase 3 -> exit corridor start point (red_zone_node)
#   exit_corridor  phase 5 only            (path_planner + corridor_perception)
#
# Each stage is its own `ros2 launch` process; its console output goes to
# <log_dir>/stage_<name>.out. Stopping it = SIGINT to that launch, which
# shuts its nodes down. A stage launch also gets SIGINT if mission_node dies
# (PR_SET_PDEATHSIG), so nothing is left running.
#
# staged_launch: false (default) = every node was started by the launch file
# (SITL, the phase test launches): start/stop do nothing.

import ctypes
import os
import shlex
import signal
import subprocess
import time

PR_SET_PDEATHSIG = 1
STOP_TIMEOUT_S = 10.0

# Node names that show a stage is up (mission_stage.launch.py)
STAGE_NODES = {
    "oak": ["sector_builder_node", "coverage_guard_node", "oak_container"],
    "down_cam": [],                       # mission_node waits for /down_frame itself
    "arena": ["red_zone_node"],
    "exit_corridor": ["corridor_perception_node", "path_planner_node"],
}


def _die_with_parent():
    """In the child before exec: SIGINT it when mission_node dies."""
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)
    except OSError:
        pass


class Stages:
    def __init__(self, node, enabled: bool, launch_args: str, log_dir: str = ""):
        self.node = node
        self.enabled = enabled
        self.launch_args = shlex.split(launch_args)
        self.log_dir = os.path.expanduser(log_dir) if log_dir else ""
        self.procs = {}                   # stage -> Popen
        self._stopping = []               # SIGINT sent, not yet reaped

    def start(self, stage: str, timeout: float = 30.0) -> bool:
        """Start a stage and wait (polling node._tick via node._sleep) until its
        nodes are up. True = up (or staging off); False = not up in time."""
        if not self.enabled:
            return True
        log = self.node.get_logger()
        proc = self.procs.get(stage)
        if proc is None or proc.poll() is not None:
            cmd = ["ros2", "launch", "mission_bringup", "mission_stage.launch.py",
                   f"stage:={stage}", *self.launch_args]
            log.info(f"[mission_node] STAGE {stage}: starting")
            out = None
            if self.log_dir:
                os.makedirs(self.log_dir, exist_ok=True)
                out = open(os.path.join(self.log_dir, f"stage_{stage}.out"), "ab")
            self.procs[stage] = subprocess.Popen(
                cmd, preexec_fn=_die_with_parent, stdout=out,
                stderr=subprocess.STDOUT if out else None)
            if out is not None:
                out.close()             # the child keeps its own copy
        want = STAGE_NODES.get(stage, [])
        if "sim:=true" in self.launch_args:            # SITL: no depthai driver
            want = [n for n in want if n != "oak_container"]
        end = time.time() + timeout
        while True:
            missing = [n for n in want if n not in self.node.get_node_names()]
            if not missing:
                log.info(f"[mission_node] STAGE {stage}: up")
                return True
            if self.procs[stage].poll() is not None:
                log.error(f"[mission_node] STAGE {stage}: launch exited "
                          f"(code {self.procs[stage].returncode})")
                return False
            if time.time() > end:
                log.error(f"[mission_node] STAGE {stage}: not up in {timeout:.0f} s "
                          f"(missing {', '.join(missing)})")
                return False
            self.node._sleep(0.5)

    def running(self, stage: str) -> bool:
        """Is the stage's launch running? (staging off: always True)"""
        if not self.enabled:
            return True
        proc = self.procs.get(stage)
        return proc is not None and proc.poll() is None

    def stop(self, stage: str):
        """SIGINT the stage's launch; it shuts its nodes down. Does not wait
        (called in flight): stop_all reaps it."""
        proc = self.procs.pop(stage, None)
        if proc is None or proc.poll() is not None:
            return
        self.node.get_logger().info(f"[mission_node] STAGE {stage}: stopping")
        proc.send_signal(signal.SIGINT)
        self._stopping.append(proc)

    def stop_all(self):
        """Stop every stage and wait for them (shutdown only)."""
        for stage in list(self.procs):
            self.stop(stage)
        for proc in self._stopping:
            try:
                proc.wait(timeout=STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                proc.kill()
        self._stopping = []
