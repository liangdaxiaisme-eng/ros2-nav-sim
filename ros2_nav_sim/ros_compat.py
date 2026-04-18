"""ROS 2 compatibility layer — lightweight rclpy-like framework."""

from __future__ import annotations

import copy
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
_topic_registry: Dict[str, List[Tuple[Callable, Any]]] = {}  # topic -> [(callback, node), ...]
_topic_registry_lock = threading.Lock()

_nodes: List["Node"] = []
_nodes_lock = threading.Lock()

_running = False
_running_lock = threading.Lock()


def _is_running() -> bool:
    with _running_lock:
        return _running


def _set_running(v: bool) -> None:
    global _running
    with _running_lock:
        _running = v


# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------
RELIABILITY_RELIABLE = 1
RELIABILITY_BEST_EFFORT = 2

DURABILITY_TRANSIENT_LOCAL = 1
DURABILITY_VOLATILE = 2

HISTORY_KEEP_LAST = 1
HISTORY_KEEP_ALL = 2


@dataclass
class QoSProfile:
    reliability: int = RELIABILITY_RELIABLE
    durability: int = DURABILITY_VOLATILE
    history: int = HISTORY_KEEP_LAST
    depth: int = 10

    # Class-level constants (rclpy-compatible)
    RELIABILITY_RELIABLE = RELIABILITY_RELIABLE
    RELIABILITY_BEST_EFFORT = RELIABILITY_BEST_EFFORT
    DURABILITY_TRANSIENT_LOCAL = DURABILITY_TRANSIENT_LOCAL
    DURABILITY_VOLATILE = DURABILITY_VOLATILE
    HISTORY_KEEP_LAST = HISTORY_KEEP_LAST
    HISTORY_KEEP_ALL = HISTORY_KEEP_ALL


qos_profile_default = QoSProfile(
    reliability=RELIABILITY_RELIABLE,
    durability=DURABILITY_VOLATILE,
    history=HISTORY_KEEP_LAST,
    depth=10,
)
qos_profile_sensor_data = QoSProfile(
    reliability=RELIABILITY_BEST_EFFORT,
    durability=DURABILITY_VOLATILE,
    history=HISTORY_KEEP_LAST,
    depth=5,
)
qos_profile_parameters = QoSProfile(
    reliability=RELIABILITY_RELIABLE,
    durability=DURABILITY_TRANSIENT_LOCAL,
    history=HISTORY_KEEP_LAST,
    depth=1,
)


# ---------------------------------------------------------------------------
# Time
# ---------------------------------------------------------------------------
class Time:
    __slots__ = ("_nanoseconds",)

    def __init__(self, seconds: float = 0, nanoseconds: int = 0) -> None:
        self._nanoseconds = int(seconds * 1_000_000_000) + int(nanoseconds)

    def nanoseconds(self) -> int:
        return self._nanoseconds

    def seconds_nanoseconds(self) -> Tuple[int, int]:
        s = self._nanoseconds // 1_000_000_000
        ns = self._nanoseconds % 1_000_000_000
        return s, ns

    def __add__(self, other: Duration) -> Time:
        return Time(nanoseconds=self._nanoseconds + other._nanoseconds)

    def __sub__(self, other: Time) -> Duration:
        return Duration(nanoseconds=self._nanoseconds - other._nanoseconds)

    def __repr__(self) -> str:
        s, ns = self.seconds_nanoseconds()
        return f"Time(seconds={s}, nanoseconds={ns})"


class Duration:
    __slots__ = ("_nanoseconds",)

    def __init__(self, seconds: float = 0, nanoseconds: int = 0) -> None:
        self._nanoseconds = int(seconds * 1_000_000_000) + int(nanoseconds)

    def nanoseconds(self) -> int:
        return self._nanoseconds

    def __repr__(self) -> str:
        s = self._nanoseconds // 1_000_000_000
        ns = self._nanoseconds % 1_000_000_000
        return f"Duration(seconds={s}, nanoseconds={ns})"


class Clock:
    def __init__(self) -> None:
        pass

    def now(self) -> Time:
        return Time(nanoseconds=int(time.time() * 1_000_000_000))


# ---------------------------------------------------------------------------
# Logger
# ---------------------------------------------------------------------------
class Logger:
    def __init__(self, node_name: str) -> None:
        self._node_name = node_name

    def _ts(self) -> str:
        return time.strftime("%Y-%m-%d %H:%M:%S")

    def info(self, msg: str) -> None:
        print(f"[{self._ts()}] [INFO] [{self._node_name}]: {msg}")

    def warn(self, msg: str) -> None:
        print(f"[{self._ts()}] [WARN] [{self._node_name}]: {msg}")

    def error(self, msg: str) -> None:
        print(f"[{self._ts()}] [ERROR] [{self._node_name}]: {msg}")

    def debug(self, msg: str) -> None:
        print(f"[{self._ts()}] [DEBUG] [{self._node_name}]: {msg}")


# ---------------------------------------------------------------------------
# Publisher / Subscription / Timer
# ---------------------------------------------------------------------------
class Publisher:
    def __init__(self, node: "Node", msg_type: Any, topic: str, qos: QoSProfile) -> None:
        self._node = node
        self._msg_type = msg_type
        self._topic = topic
        self._qos = qos
        self._last_message: Optional[Any] = None  # for transient local

        with _topic_registry_lock:
            if topic not in _topic_registry:
                _topic_registry[topic] = []

    def publish(self, msg: Any) -> None:
        self._last_message = copy.deepcopy(msg)
        with _topic_registry_lock:
            subs = list(_topic_registry.get(self._topic, []))
        for callback, sub_node in subs:
            if sub_node._destroyed:
                continue
            try:
                callback(copy.deepcopy(msg))
            except Exception as e:
                import traceback
                traceback.print_exc()

    def get_subscription_count(self) -> int:
        with _topic_registry_lock:
            return len(_topic_registry.get(self._topic, []))

    def destroy(self) -> None:
        pass  # nothing special to clean up


class Subscription:
    def __init__(self, node: "Node", msg_type: Any, topic: str,
                 callback: Callable, qos: QoSProfile) -> None:
        self._node = node
        self._msg_type = msg_type
        self._topic = topic
        self._callback = callback
        self._qos = qos

        with _topic_registry_lock:
            _topic_registry.setdefault(topic, []).append((callback, node))

        # transient-local: replay last message
        if qos.durability == DURABILITY_TRANSIENT_LOCAL:
            with _topic_registry_lock:
                # find publishers on this topic
                for n in list(_nodes):
                    for pub in n._publishers:
                        if pub._topic == topic and pub._last_message is not None:
                            try:
                                callback(copy.deepcopy(pub._last_message))
                            except Exception:
                                pass

    def destroy(self) -> None:
        with _topic_registry_lock:
            entries = _topic_registry.get(self._topic, [])
            entries[:] = [(cb, n) for cb, n in entries if cb is not self._callback]


class Timer:
    def __init__(self, period: float, callback: Callable) -> None:
        self._period = period
        self._callback = callback
        self._cancelled = False
        self._last_fire = time.time()
        self._lock = threading.Lock()

    def _tick(self) -> None:
        with self._lock:
            if self._cancelled:
                return
            now = time.time()
            if now - self._last_fire >= self._period:
                self._last_fire = now
                try:
                    self._callback()
                except Exception:
                    import traceback
                    traceback.print_exc()

    def cancel(self) -> None:
        with self._lock:
            self._cancelled = True

    def is_ready(self) -> bool:
        with self._lock:
            if self._cancelled:
                return False
            return (time.time() - self._last_fire) >= self._period


# ---------------------------------------------------------------------------
# Parameter helper
# ---------------------------------------------------------------------------
@dataclass
class Parameter:
    name: str
    value: Any


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class Node:
    def __init__(self, node_name: str, namespace: str = "") -> None:
        self._node_name = node_name
        self._namespace = namespace
        self._publishers: List[Publisher] = []
        self._subscriptions: List[Subscription] = []
        self._timers: List[Timer] = []
        self._params: Dict[str, Any] = {}
        self._logger = Logger(node_name)
        self._clock = Clock()
        self._destroyed = False

        with _nodes_lock:
            _nodes.append(self)

    # --- factory methods ---

    def create_publisher(self, msg_type: Any, topic: str, qos: QoSProfile) -> Publisher:
        pub = Publisher(self, msg_type, topic, qos)
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type: Any, topic: str,
                            callback: Callable, qos: QoSProfile) -> Subscription:
        sub = Subscription(self, msg_type, topic, callback, qos)
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, period: float, callback: Callable) -> Timer:
        timer = Timer(period, callback)
        self._timers.append(timer)
        return timer

    # --- accessors ---

    def get_logger(self) -> Logger:
        return self._logger

    def get_clock(self) -> Clock:
        return self._clock

    def get_name(self) -> str:
        return self._node_name

    def declare_parameter(self, name: str, default_value: Any) -> Parameter:
        if name not in self._params:
            self._params[name] = default_value
        return Parameter(name, self._params[name])

    def get_parameter(self, name: str) -> Parameter:
        if name not in self._params:
            self._logger.warn(f"Parameter '{name}' not declared, returning None")
            return Parameter(name, None)
        return Parameter(name, self._params[name])

    def set_parameters(self, params: List[Parameter]) -> None:
        for p in params:
            self._params[p.name] = p.value

    # --- cleanup ---

    def destroy_node(self) -> None:
        self._destroyed = True
        for t in self._timers:
            t.cancel()
        for s in self._subscriptions:
            s.destroy()
        with _nodes_lock:
            if self in _nodes:
                _nodes.remove(self)


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------
def init(args: Optional[List[str]] = None) -> None:
    _set_running(True)


def shutdown() -> None:
    _set_running(False)
    # Snapshot nodes first, then destroy outside the lock to avoid deadlock
    with _nodes_lock:
        nodes_snapshot = list(_nodes)
    for node in nodes_snapshot:
        node.destroy_node()
    with _nodes_lock:
        _nodes.clear()
    with _topic_registry_lock:
        _topic_registry.clear()


def ok() -> bool:
    return _is_running()


def spin(node: Node) -> None:
    """Block and process timers / keep-alive until shutdown()."""
    while ok():
        spin_once(node, timeout_sec=0.05)


def spin_once(node: Node, timeout_sec: float = 0.1) -> None:
    """Run one spin cycle — fire any ready timers."""
    deadline = time.time() + timeout_sec
    for timer in list(node._timers):
        if not _is_running():
            break
        timer._tick()
    # sleep remainder
    remaining = deadline - time.time()
    if remaining > 0:
        time.sleep(remaining)
