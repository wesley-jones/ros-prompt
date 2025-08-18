# -*- coding: utf-8 -*-
"""
ros_prompt/adapters_py/builtins/builtin_nav_modes.py

Two vendor-agnostic **built-in** behaviours for ROS Prompt:

  • `SwitchToMappingMode` – cleanly stop Nav2, stop SLAM (if running),
    launch SLAM Toolbox (online_async), launch Nav2 in SLAM mode
    (`slam:=True autostart:=False`), then STARTUP the stack.

  • `StartNavWithMap` – cleanly stop Nav2 & SLAM, then launch Nav2 for
    localisation with a stored map (`slam:=False map:=... autostart:=False`),
    then STARTUP the stack.

Design notes
------------
• Each top-level builtin is a **py_trees Sequence** that wires together
  small, re-usable child behaviours.  Nothing heavy runs in `initialise()`;
  all actions are triggered from `update()` so the behaviours stay
  non-blocking.

• Launching uses the standard **`ros2 launch` CLI** via `subprocess.Popen` (non‑blocking),
  because `LaunchService.run()` must be in the main thread. We keep handles to
  those child processes so we can terminate them cleanly later with a dedicated
  behaviour (`TerminateLaunch`). This preserves responsiveness and avoids
  stray wrapper processes.

• Lifecycle orchestration uses standard services:
    - `nav2_msgs/srv/ManageLifecycleNodes` for Nav2 STARTUP/SHUTDOWN
    - `lifecycle_msgs/srv/ChangeState` & `.../GetState` for SLAM Toolbox

• Vendor differences are isolated behind small helpers (manager name
  discovery, topic waits, arg parsing, etc.).

Tested ROS APIs: Humble→Jazzy.  Older Foxy builds may use `/nav_lifecycle_manager`.
"""

from __future__ import annotations
import threading
import time
from typing import Dict, Iterable, List, Optional, Sequence as SeqType, Tuple

import py_trees

# ROS imports are runtime-only (no ROS in unit tests)
try:  # rclpy / launch may not be available at type-check time
    import rclpy
    from rclpy.node import Node
    from rclpy.task import Future
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

    from lifecycle_msgs.srv import GetState as LCGetState, ChangeState
    from lifecycle_msgs.msg import Transition, State as LCState

    from nav2_msgs.srv import ManageLifecycleNodes

    # We no longer import LaunchService due to main-thread constraint.
    # from launch import LaunchService, LaunchDescription
    # from launch.actions import IncludeLaunchDescription
    # from launch.launch_description_sources import PythonLaunchDescriptionSource
    # from launch_ros.actions import Node as LaunchNode
except Exception:  # pragma: no cover - allow import in non-ROS envs
    Node = object  # type: ignore

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

NAV2_MANAGER_CANDIDATES = (
    "/nav2_lifecycle_manager",
    "/nav_lifecycle_manager",
    "/lifecycle_manager_navigation",  # older images
)

NAV2_CORE_NODES = (
    "/controller_server",
    "/planner_server",
    "/bt_navigator",
    "/smoother_server",
    "/waypoint_follower",
    "/amcl",  # present when in localisation mode
    "/map_server",  # present when using a static map
)

SLAM_NODE_NAME = "/slam_toolbox"


def _now() -> float:
    return time.time()


class _NonBlockingBehaviour(py_trees.behaviour.Behaviour):
    """Base that stores the ROS node and common utilities.

    Each child behaviour **must** be non-blocking: trigger work on the first
    `update()` call and then poll for readiness on subsequent ticks.
    """

    def __init__(self, name: str, node: Node, timeout_sec: float = 30.0):
        super().__init__(name=name)
        self.node = node
        self.log = node.get_logger()
        self.timeout_sec = timeout_sec
        self._deadline: float = 0.0
        self._started = False

    # py_trees hooks -----------------------------------------------------
    def initialise(self) -> None:
        # Do nothing heavy here; just reset flags.
        self._started = False
        self._deadline = _now() + self.timeout_sec

    # helper -------------------------------------------------------------
    def _timed_out(self) -> bool:
        return _now() > self._deadline


# ---- Graph queries ------------------------------------------------------

def list_topics(node: Node) -> List[str]:
    return [name for name, _ in node.get_topic_names_and_types()]


def node_exists(node: Node, name: str) -> bool:
    for n, ns in node.get_node_names_and_namespaces():
        fq = f"{ns}/{n}" if ns and ns != "/" else f"/{n}"
        if fq == name:
            return True
    return False


def any_node_exists(node: Node, names: Iterable[str]) -> bool:
    return any(node_exists(node, n) for n in names)


def service_exists(node: Node, srv_name: str) -> bool:
    """True if a service with the given fully-qualified name exists."""
    for name, _ in node.get_service_names_and_types():
        if name == srv_name:
            return True
    return False


def list_topics(node: Node) -> List[str]:
    return [name for name, _ in node.get_topic_names_and_types()]


def node_exists(node: Node, name: str) -> bool:
    for n, ns in node.get_node_names_and_namespaces():
        fq = f"{ns}/{n}" if ns and ns != "/" else f"/{n}"
        if fq == name:
            return True
    return False


def any_node_exists(node: Node, names: Iterable[str]) -> bool:
    return any(node_exists(node, n) for n in names)


# ---- Lifecycle helpers (non-blocking) -----------------------------------

class RequestNav2Shutdown(_NonBlockingBehaviour):
    """Send SHUTDOWN to Nav2 lifecycle manager and wait until core nodes
    are gone.  If no manager is present, succeed immediately.
    """

    def __init__(self, node: Node, name: str = "RequestNav2Shutdown", timeout_sec: float = 20.0):
        super().__init__(name=name, node=node, timeout_sec=timeout_sec)
        self._mgr_ns: Optional[str] = None
        self._client = None
        self._future: Optional[Future] = None

    def update(self) -> py_trees.common.Status:
        if self._timed_out():
            self.log.error("[Nav2Shutdown] timeout")
            return py_trees.common.Status.FAILURE

        # discover manager once
        if not self._mgr_ns:
            for cand in NAV2_MANAGER_CANDIDATES:
                if node_exists(self.node, cand):
                    self._mgr_ns = cand
                    break
            if not self._mgr_ns:
                # nothing to do
                return py_trees.common.Status.SUCCESS

        # send request once
        if not self._started:
            self._client = self.node.create_client(ManageLifecycleNodes, f"{self._mgr_ns}/manage_nodes")
            if not self._client.wait_for_service(timeout_sec=0.0):
                # service not up yet – keep ticking
                return py_trees.common.Status.RUNNING
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.SHUTDOWN
            self._future = self._client.call_async(req)
            self._started = True
            return py_trees.common.Status.RUNNING

        # wait for service reply (non-blocking)
        assert self._future is not None
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        # after reply, wait until core nodes are gone
        if any_node_exists(self.node, NAV2_CORE_NODES):
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS


class RequestSlamShutdown(_NonBlockingBehaviour):
    """Send SHUTDOWN to SLAM Toolbox if present; otherwise succeed."""

    def __init__(self, node: Node, name: str = "RequestSlamShutdown", timeout_sec: float = 15.0):
        super().__init__(name=name, node=node, timeout_sec=timeout_sec)
        self._client_cs = None
        self._future: Optional[Future] = None

    def update(self) -> py_trees.common.Status:
        if self._timed_out():
            self.log.error("[SLAM Shutdown] timeout")
            return py_trees.common.Status.FAILURE

        # Fast path: if neither node nor its lifecycle services exist, succeed.
        cs = f"{SLAM_NODE_NAME}/change_state"
        gs = f"{SLAM_NODE_NAME}/get_state"
        slam_present = node_exists(self.node, SLAM_NODE_NAME)
        cs_up = service_exists(self.node, cs)
        gs_up = service_exists(self.node, gs)
        if not slam_present and not cs_up and not gs_up:
            return py_trees.common.Status.SUCCESS

        # If node is not present but a stale service listing remains, keep polling
        # briefly; ROS graph should converge quickly.
        if not slam_present and not self._started:
            return py_trees.common.Status.RUNNING

        if not self._started:
            # change_state → SHUTDOWN (only if service is visible)
            self._client_cs = self.node.create_client(ChangeState, cs)
            if not self._client_cs.wait_for_service(timeout_sec=0.0):
                # service not up yet; re-check on next tick (but won't time out
                # spuriously because we early-success when both node+services are gone)
                return py_trees.common.Status.RUNNING
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_SHUTDOWN
            self._future = self._client_cs.call_async(req)
            self._started = True
            return py_trees.common.Status.RUNNING

        assert self._future is not None
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        # Confirm node gone or FINALIZED; if /get_state is not available, consider it down.
        if not service_exists(self.node, gs):
            return py_trees.common.Status.SUCCESS

        client_gs = self.node.create_client(LCGetState, gs)
        if client_gs.wait_for_service(timeout_sec=0.0):
            fut = client_gs.call_async(LCGetState.Request())
            if not fut.done():
                return py_trees.common.Status.RUNNING
            if fut.result().current_state.id == LCState.PRIMARY_STATE_FINALIZED:
                return py_trees.common.Status.SUCCESS
        # In all other cases, if the node is still present keep spinning until timeout.
        if not node_exists(self.node, SLAM_NODE_NAME):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# ---- Launch helpers ------------------------------------------------------
import subprocess, os, signal
from typing import Dict

# Registry of ros2 launch subprocesses we started ourselves.
_LAUNCH_REGISTRY: Dict[str, subprocess.Popen] = {}

class LaunchIncludeBehaviour(_NonBlockingBehaviour):
    """Start a launch file using the **ros2 launch** CLI (non-blocking).

    We avoid `launch.LaunchService` here because it often expects to run in the
    main thread to install signal handlers. Using the CLI keeps the BT tick loop
    responsive and is robust across vendor images.

    Optionally pass a `label` so the process handle is tracked in a registry
    for later teardown by `TerminateLaunch`.
    """

    def __init__(self, node: Node,
                 launch_file: SeqType[str],
                 launch_args: SeqType[str],
                 label: Optional[str] = None,
                 name: str = "LaunchIncludeBehaviour",
                 timeout_sec: float = 5.0):
        super().__init__(name=name, node=node, timeout_sec=timeout_sec)
        if len(launch_file) != 2:
            raise RuntimeError(f"launch_file must be '<pkg> <file.py>', got: {launch_file}")
        self._pkg, self._file = launch_file
        self._args = list(launch_args)
        self._proc: Optional[subprocess.Popen] = None
        self._label = label

    def update(self) -> py_trees.common.Status:
        if self._timed_out():
            return py_trees.common.Status.FAILURE

        if not self._started:
            cmd = ["ros2", "launch", self._pkg, self._file] + self._args
            self.log.info("[Launch] " + " ".join(cmd))
            self._proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            if self._label:
                _LAUNCH_REGISTRY[self._label] = self._proc
            self._started = True
            return py_trees.common.Status.SUCCESS  # launch is in-flight

        # If process died early, report failure.
        if self._proc and (self._proc.poll() is not None) and self._proc.returncode != 0:
            self.log.error(f"[Launch] process exited with code {self._proc.returncode}")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        # Do not kill on SUCCESS; teardown is handled explicitly by TerminateLaunch.
        pass


class TerminateLaunch(_NonBlockingBehaviour):
    """Gracefully stop a tracked `ros2 launch` subprocess by label.

    Non-blocking: sends SIGINT on first tick, then waits up to `grace_sec`.
    If still running, sends SIGTERM and finally SIGKILL after `kill_sec`.
    Succeeds when the process exits or there is nothing to terminate.
    """

    def __init__(self, node: Node, label: str, grace_sec: float = 3.0, kill_sec: float = 2.0,
                 name: str = "TerminateLaunch", timeout_sec: float = 10.0):
        super().__init__(name=name, node=node, timeout_sec=timeout_sec)
        self._label = label
        self._phase = 0
        self._t0 = 0.0

    def update(self) -> py_trees.common.Status:
        if self._timed_out():
            self.log.warn(f"[TerminateLaunch:{self._label}] timeout; giving up")
            _LAUNCH_REGISTRY.pop(self._label, None)
            return py_trees.common.Status.SUCCESS

        proc = _LAUNCH_REGISTRY.get(self._label)
        if not proc:
            return py_trees.common.Status.SUCCESS

        # If process already exited, unregister and succeed
        if proc.poll() is not None:
            _LAUNCH_REGISTRY.pop(self._label, None)
            return py_trees.common.Status.SUCCESS

        # Phase 0: send SIGINT
        if self._phase == 0:
            try:
                os.kill(proc.pid, signal.SIGINT)
                self.log.info(f"[TerminateLaunch:{self._label}] SIGINT sent")
            except Exception as e:
                self.log.warn(f"[TerminateLaunch:{self._label}] SIGINT failed: {e}")
            self._phase = 1
            self._t0 = _now()
            return py_trees.common.Status.RUNNING

        # Phase 1: wait grace, then SIGTERM
        if self._phase == 1:
            if proc.poll() is None and (_now() - self._t0) < 3.0:
                return py_trees.common.Status.RUNNING
            if proc.poll() is None:
                try:
                    os.kill(proc.pid, signal.SIGTERM)
                    self.log.warn(f"[TerminateLaunch:{self._label}] SIGTERM sent")
                except Exception as e:
                    self.log.warn(f"[TerminateLaunch:{self._label}] SIGTERM failed: {e}")
                self._phase = 2
                self._t0 = _now()
                return py_trees.common.Status.RUNNING
            _LAUNCH_REGISTRY.pop(self._label, None)
            return py_trees.common.Status.SUCCESS

        # Phase 2: wait then SIGKILL
        if self._phase == 2:
            if proc.poll() is None and (_now() - self._t0) < 2.0:
                return py_trees.common.Status.RUNNING
            if proc.poll() is None:
                try:
                    os.kill(proc.pid, signal.SIGKILL)
                    self.log.error(f"[TerminateLaunch:{self._label}] SIGKILL sent")
                except Exception as e:
                    self.log.warn(f"[TerminateLaunch:{self._label}] SIGKILL failed: {e}")
            _LAUNCH_REGISTRY.pop(self._label, None)
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


class Nav2LifecycleStartup(_NonBlockingBehaviour):
    """Call ManageLifecycleNodes.STARTUP and wait for ACTIVE."""

    def __init__(self, node: Node, name: str = "Nav2LifecycleStartup", timeout_sec: float = 25.0):
        super().__init__(name=name, node=node, timeout_sec=timeout_sec)
        self._mgr_ns: Optional[str] = None
        self._client = None
        self._future: Optional[Future] = None
        self._client_gs = None

    def update(self) -> py_trees.common.Status:
        if self._timed_out():
            self.log.error("[Nav2 STARTUP] timeout")
            return py_trees.common.Status.FAILURE

        if not self._mgr_ns:
            for cand in NAV2_MANAGER_CANDIDATES:
                if node_exists(self.node, cand):
                    self._mgr_ns = cand
                    break
            if not self._mgr_ns:
                self.log.warn("[Nav2 STARTUP] lifecycle manager not found")
                return py_trees.common.Status.FAILURE

        if not self._started:
            self._client = self.node.create_client(ManageLifecycleNodes, f"{self._mgr_ns}/manage_nodes")
            if not self._client.wait_for_service(timeout_sec=0.0):
                return py_trees.common.Status.RUNNING
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.STARTUP
            self._future = self._client.call_async(req)
            self._client_gs = self.node.create_client(LCGetState, f"{self._mgr_ns}/get_state")
            self._started = True
            return py_trees.common.Status.RUNNING

        assert self._future is not None
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        # Optionally verify ACTIVE
        if self._client_gs and self._client_gs.wait_for_service(timeout_sec=0.0):
            fut = self._client_gs.call_async(LCGetState.Request())
            if not fut.done():
                return py_trees.common.Status.RUNNING
            if fut.result().current_state.id == LCState.PRIMARY_STATE_ACTIVE:
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

        # If we can't query state, consider STARTUP done.
        return py_trees.common.Status.SUCCESS

# ---- Wait for topics ----------------------------------------------------- -----------------------------------------------------

class WaitForTopics(_NonBlockingBehaviour):
    """Return SUCCESS once *all* topics are present in the graph."""

    def __init__(self, node: Node, topics: SeqType[str], name: str = "WaitForTopics", timeout_sec: float = 30.0):
        super().__init__(name=name, node=node, timeout_sec=timeout_sec)
        self._topics = list(topics)

    def update(self) -> py_trees.common.Status:
        if self._timed_out():
            missing = [t for t in self._topics if t not in list_topics(self.node)]
            self.log.error(f"[WaitForTopics] timeout. Missing: {missing}")
            return py_trees.common.Status.FAILURE

        current = set(list_topics(self.node))
        if all(t in current for t in self._topics):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# ---------------------------------------------------------------------------
# Top-level builtins: SwitchToMappingMode & StartNavWithMap
# ---------------------------------------------------------------------------

class SwitchToMappingMode(py_trees.composites.Sequence):
    """Composite builtin used by robot_caps.yaml as `SwitchToMappingMode`.

    Steps (all non-blocking children):
      1) RequestNav2Shutdown
      2) TerminateLaunch(label="nav2")            # ensure wrapper process exits
      3) RequestSlamShutdown
      4) TerminateLaunch(label="slam")            # ensure old SLAM wrapper exits
      5) LaunchIncludeBehaviour (SLAM online_async, label="slam")
      6) WaitForTopics(["/map", "/slam_toolbox/pose"])   # SLAM publishing
      7) LaunchIncludeBehaviour (Nav2 navigation_launch.py with slam:=True, autostart:=False, label="nav2")
      8) Nav2LifecycleStartup
      9) WaitForTopics(["/map"])                          # map topic stable
    """

    def __init__(self,
                 node: Node,
                 slam_launch: str,
                 nav_launch: str,
                 common_args: str,
                 nav_args: str,
                 timeout_sec: int = 60,
                 name: str = "SwitchToMappingMode"):
        super().__init__(name=name, memory=True)
        # Tokenise incoming strings from robot_caps.yaml
        slam_launch_tokens = slam_launch.split()  # [pkg, file.py]
        nav_launch_tokens  = nav_launch.split()
        common_arg_tokens  = common_args.split() if common_args else []
        nav_arg_tokens     = nav_args.split() if nav_args else []

        # 1) stop Nav2 & SLAM
        self.add_child(RequestNav2Shutdown(node=node, timeout_sec=20))
        self.add_child(TerminateLaunch(node=node, label="nav2", name="TerminateNav2Launch"))
        self.add_child(RequestSlamShutdown(node=node, timeout_sec=15))
        self.add_child(TerminateLaunch(node=node, label="slam", name="TerminateSlamLaunch"))

        # 2) start SLAM
        self.add_child(LaunchIncludeBehaviour(node=node,
                                             launch_file=slam_launch_tokens,
                                             launch_args=common_arg_tokens,
                                             label="slam",
                                             name="LaunchSLAM"))
        self.add_child(WaitForTopics(node=node,
                                     topics=("/map", "/slam_toolbox/pose"),
                                     name="WaitSLAMTopics",
                                     timeout_sec=45))

        # 3) start Nav2 in slam mode and activate
        self.add_child(LaunchIncludeBehaviour(node=node,
                                             launch_file=nav_launch_tokens,
                                             launch_args=common_arg_tokens + nav_arg_tokens,
                                             label="nav2",
                                             name="LaunchNav2Slam"))
        self.add_child(Nav2LifecycleStartup(node=node, timeout_sec=30))
        self.add_child(WaitForTopics(node=node,
                                     topics=("/map",),
                                     name="WaitMapTopic",
                                     timeout_sec=30))


class StartNavWithMap(py_trees.composites.Sequence):
    """Composite builtin used by robot_caps.yaml as `StartNavWithMap`.

    Steps:
      1) RequestNav2Shutdown
      2) TerminateLaunch(label="nav2")
      3) RequestSlamShutdown
      4) TerminateLaunch(label="slam")
      5) LaunchIncludeBehaviour (Nav2 navigation_launch.py with slam:=False, autostart:=False, map:=<yaml>, label="nav2")
      6) Nav2LifecycleStartup
      7) WaitForTopics(["/map"])  # static map available
    """

    def __init__(self,
                 node: Node,
                 nav_launch: str,
                 common_args: str,
                 nav_args: str,
                 map_yaml: Optional[str] = None,
                 timeout_sec: int = 60,
                 name: str = "StartNavWithMap"):
        super().__init__(name=name, memory=True)

        nav_launch_tokens  = nav_launch.split()
        common_arg_tokens  = common_args.split() if common_args else []
        nav_arg_tokens     = nav_args.split() if nav_args else []
        if map_yaml:
            nav_arg_tokens.append(f"map:={map_yaml}")

        # 1) stop anything running
        self.add_child(RequestNav2Shutdown(node=node, timeout_sec=20))
        self.add_child(TerminateLaunch(node=node, label="nav2", name="TerminateNav2Launch"))
        self.add_child(RequestSlamShutdown(node=node, timeout_sec=15))
        self.add_child(TerminateLaunch(node=node, label="slam", name="TerminateSlamLaunch"))

        # 2) start Nav2 for localisation
        self.add_child(LaunchIncludeBehaviour(node=node,
                                             launch_file=nav_launch_tokens,
                                             launch_args=common_arg_tokens + nav_arg_tokens,
                                             label="nav2",
                                             name="LaunchNav2Localisation"))
        self.add_child(Nav2LifecycleStartup(node=node, timeout_sec=30))
        self.add_child(WaitForTopics(node=node,
                                     topics=("/map",),
                                     name="WaitStaticMap",
                                     timeout_sec=30))


# End of file
