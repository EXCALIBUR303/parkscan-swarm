"""Robot autonomy: lane-graph routing, task state machine, navigation, battery, docking.

One robot's brain, and nothing else. This package consumes a
:class:`~parkscan_core.world.WorldBackend` -- *any* implementation -- and drives a single
robot through it. It performs no I/O: :meth:`~parkscan_agent.robot.RobotAgent.tick` returns
a :class:`~parkscan_agent.robot.TelemetrySample`, and the runner that owns the loop decides
what to publish.

It depends on :mod:`parkscan_core` alone. It does **not** import ``parkscan_sim``, and
``tests/test_no_simulator_dependency.py`` enforces that by parsing every module in this
package: the same autonomy has to run unchanged against a future ROS 2 backend, and an
adapter whose only consumer secretly knows which backend it is talking to has not proven
anything (ADR-0001).

Nothing here is a physics simulation, and nothing here models hardware. The controller
emits a desired body twist; the backend decides what a robot does with it.
"""

from parkscan_agent.battery_policy import BatteryParams, ReturnDecision, should_return
from parkscan_agent.docking import DockConfig, DockController, DockPhase, DockStatus
from parkscan_agent.lanegraph import (
    NoRouteError,
    RoutingError,
    UnknownNodeError,
    lane_distance_m,
    nearest_node,
    path_length_m,
    route,
)
from parkscan_agent.navigation import NavConfig, avoid, follow_route, waypoint_command
from parkscan_agent.robot import AgentConfig, RobotAgent, TelemetrySample, ZoneProgress
from parkscan_agent.statemachine import TRANSITIONS, Event, IllegalTransition, step

__all__ = [
    "TRANSITIONS",
    "AgentConfig",
    "BatteryParams",
    "DockConfig",
    "DockController",
    "DockPhase",
    "DockStatus",
    "Event",
    "IllegalTransition",
    "NavConfig",
    "NoRouteError",
    "ReturnDecision",
    "RobotAgent",
    "RoutingError",
    "TelemetrySample",
    "UnknownNodeError",
    "ZoneProgress",
    "avoid",
    "follow_route",
    "lane_distance_m",
    "nearest_node",
    "path_length_m",
    "route",
    "should_return",
    "step",
    "waypoint_command",
]
