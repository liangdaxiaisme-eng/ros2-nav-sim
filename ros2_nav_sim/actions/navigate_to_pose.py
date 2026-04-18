"""ROS 2 action types — NavigateToPose + lightweight ActionServer / ActionClient."""

from __future__ import annotations

import copy
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from ..ros_compat import Node, _is_running, _topic_registry, _topic_registry_lock

from ..msgs.std_msgs import Header
from ..msgs.geometry_msgs import PoseStamped


# ---------------------------------------------------------------------------
# NavigateToPose action definition
# ---------------------------------------------------------------------------
class NavigateToPose:
    @dataclass
    class Goal:
        pose: PoseStamped = field(default_factory=PoseStamped)
        behavior_tree: str = ""

    @dataclass
    class Result:
        total_elapsed_time: float = 0.0
        number_of_recoveries: int = 0
        distance_remaining: float = 0.0
        error_code: int = 0
        error_msg: str = ""

    @dataclass
    class Feedback:
        current_pose: PoseStamped = field(default_factory=PoseStamped)
        navigation_time: float = 0.0
        estimated_time_remaining: float = 0.0
        number_of_recoveries: int = 0
        distance_remaining: float = 0.0


# ---------------------------------------------------------------------------
# GoalHandle (returned to client for status / cancel)
# ---------------------------------------------------------------------------
class GoalHandle:
    """Represents a submitted goal on the action server side."""

    def __init__(self, goal_id: str, goal: Any, server: "ActionServer" = None) -> None:
        self.goal_id = goal_id
        self.goal = goal
        self.feedback: Optional[Any] = None
        self.result: Optional[Any] = None
        self.accepted = False
        self.cancel_requested = False
        self.completed = False
        self._server = server
        self._start_time = time.time()

    def publish_feedback(self, feedback: Any) -> None:
        self.feedback = feedback
        if self._server is not None:
            _action_feedback_notify(self._server._action_name, self.goal_id, feedback)

    def mark_completed(self, result: Any) -> None:
        self.result = result
        self.completed = True

    @property
    def is_cancel_requested(self) -> bool:
        return self.cancel_requested

    def request_cancel(self) -> None:
        self.cancel_requested = True


# ---------------------------------------------------------------------------
# ActionFuture (returned to client when sending a goal)
# ---------------------------------------------------------------------------
class ActionFuture:
    """Simple future-like object for async action results."""

    def __init__(self, goal_id: str) -> None:
        self.goal_id = goal_id
        self._feedback_callbacks: List[Callable] = []
        self._result_callbacks: List[Callable] = []
        self._result: Optional[Any] = None
        self._accepted: bool = False
        self._completed: bool = False
        self._queued_feedback: List[Any] = []
        self._lock = threading.Lock()

    def add_feedback_callback(self, cb: Callable) -> None:
        with self._lock:
            self._feedback_callbacks.append(cb)
            # deliver any queued feedback
            queued = list(self._queued_feedback)
            self._queued_feedback.clear()
        for fb in queued:
            try:
                cb(fb)
            except Exception:
                import traceback
                traceback.print_exc()

    def add_result_callback(self, cb: Callable) -> None:
        with self._lock:
            self._result_callbacks.append(cb)

    def _set_feedback(self, feedback: Any) -> None:
        with self._lock:
            cbs = list(self._feedback_callbacks)
            if not cbs:
                self._queued_feedback.append(feedback)
                return
        for cb in cbs:
            try:
                cb(feedback)
            except Exception:
                import traceback
                traceback.print_exc()

    def _set_result(self, result: Any) -> None:
        with self._lock:
            self._result = result
            self._completed = True
            cbs = list(self._result_callbacks)
        for cb in cbs:
            try:
                cb(result)
            except Exception:
                import traceback
                traceback.print_exc()

    def _set_accepted(self, accepted: bool) -> None:
        with self._lock:
            self._accepted = accepted

    def get_result(self) -> Optional[Any]:
        return self._result

    def is_completed(self) -> bool:
        return self._completed

    def is_accepted(self) -> bool:
        return self._accepted


# ---------------------------------------------------------------------------
# Global action registry
# ---------------------------------------------------------------------------
_action_servers: Dict[str, "ActionServer"] = {}
_action_servers_lock = threading.Lock()


# ---------------------------------------------------------------------------
# ActionServer
# ---------------------------------------------------------------------------
class ActionServer:
    """Lightweight action server — runs execute_callback in a thread per goal."""

    def __init__(
        self,
        node: Node,
        action_type: Any,
        action_name: str,
        execute_callback: Callable,
        goal_callback: Optional[Callable] = None,
        cancel_callback: Optional[Callable] = None,
    ) -> None:
        self._node = node
        self._action_type = action_type
        self._action_name = action_name
        self._execute_callback = execute_callback
        self._goal_callback = goal_callback
        self._cancel_callback = cancel_callback

        self._current_handle: Optional[GoalHandle] = None
        self._handles: Dict[str, GoalHandle] = {}
        self._lock = threading.Lock()
        self._destroyed = False

        # Register in global registry so clients can find us
        with _action_servers_lock:
            _action_servers[action_name] = self

    def _submit_goal(self, goal_id: str, goal: Any, future: ActionFuture) -> None:
        """Called internally by ActionClient to submit a goal."""
        with self._lock:
            if self._destroyed:
                return
            handle = GoalHandle(goal_id, goal, server=self)

            # Run goal callback (accept/reject)
            if self._goal_callback is not None:
                try:
                    accepted = self._goal_callback(goal)
                except Exception:
                    accepted = True
            else:
                accepted = True

            if not accepted:
                future._set_accepted(False)
                return

            handle.accepted = True
            self._handles[goal_id] = handle
            self._current_handle = handle

        future._set_accepted(True)

        # Run execute in a thread
        def _run() -> None:
            try:
                result = self._execute_callback(handle)
                handle.mark_completed(result)
                future._set_result(result)
            except Exception as e:
                import traceback
                traceback.print_exc()
                error_result = self._action_type.Result(error_code=1, error_msg=str(e))
                handle.mark_completed(error_result)
                future._set_result(error_result)
            finally:
                with self._lock:
                    self._handles.pop(goal_id, None)
                    if self._current_handle is handle:
                        self._current_handle = None

        t = threading.Thread(target=_run, daemon=True)
        t.start()

    def publish_feedback(self, feedback: Any) -> None:
        """Publish feedback for the currently active goal (server-side)."""
        with self._lock:
            handle = self._current_handle
        if handle is not None:
            handle.publish_feedback(feedback)

    def is_active(self) -> bool:
        with self._lock:
            return self._current_handle is not None and not self._current_handle.completed

    def destroy(self) -> None:
        self._destroyed = True
        with _action_servers_lock:
            _action_servers.pop(self._action_name, None)


# ---------------------------------------------------------------------------
# ActionClient
# ---------------------------------------------------------------------------
class ActionClient:
    """Lightweight action client."""

    def __init__(self, node: Node, action_type: Any, action_name: str) -> None:
        self._node = node
        self._action_type = action_type
        self._action_name = action_name
        self._futures: Dict[str, ActionFuture] = {}

    def send_goal_async(self, goal: Any) -> ActionFuture:
        """Send a goal asynchronously. Returns an ActionFuture."""
        goal_id = str(uuid.uuid4())
        future = ActionFuture(goal_id)
        self._futures[goal_id] = future

        # Register feedback forwarder
        _register_feedback_forwarder(self._action_name, goal_id, future)

        # Find the server and submit
        with _action_servers_lock:
            server = _action_servers.get(self._action_name)

        if server is None:
            future._set_accepted(False)
            future._set_result(
                self._action_type.Result(error_code=1, error_msg="No action server available")
            )
        else:
            server._submit_goal(goal_id, goal, future)

        return future

    def cancel_goal_async(self) -> None:
        """Request cancellation of the current goal."""
        # Find the latest active future and request cancel
        for goal_id, future in reversed(list(self._futures.items())):
            if not future.is_completed():
                with _action_servers_lock:
                    server = _action_servers.get(self._action_name)
                if server is not None:
                    with server._lock:
                        handle = server._handles.get(goal_id)
                    if handle is not None:
                        handle.cancel_requested = True
                        if server._cancel_callback is not None:
                            try:
                                server._cancel_callback(handle)
                            except Exception:
                                pass
                break


# ---------------------------------------------------------------------------
# Feedback notification (in-process)
# ---------------------------------------------------------------------------
_feedback_forwarders: Dict[str, Dict[str, List[Callable]]] = {}
_feedback_lock = threading.Lock()


def _register_feedback_forwarder(action_name: str, goal_id: str, future: ActionFuture) -> None:
    with _feedback_lock:
        _feedback_forwarders.setdefault(action_name, {}).setdefault(goal_id, []).append(
            future._set_feedback
        )


def _action_feedback_notify(action_name: str, goal_id: str, feedback: Any) -> None:
    with _feedback_lock:
        cbs = list(_feedback_forwarders.get(action_name, {}).get(goal_id, []))
    for cb in cbs:
        try:
            cb(copy.deepcopy(feedback))
        except Exception:
            pass
