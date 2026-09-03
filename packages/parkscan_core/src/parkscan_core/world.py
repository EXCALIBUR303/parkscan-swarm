"""The one interface every simulator backend implements.

Why this exists
---------------
Three verified constraints force a process boundary into this project (see
``docs/ARCHITECTURE.md`` section 0): ROS 2 has no macOS/arm64 support tier, containers on
macOS get no GPU so a containerised Gazebo renders cameras on the CPU, and the detector
must therefore run natively on the host. Since the boundary is unavoidable, the design
puts every piece of intellectual content *above* it and makes the simulator a swappable
backend. ``sim-lite`` (deterministic, ~100x real time, the default) and the optional
``ros2-gz`` backend both satisfy this Protocol; the task state machine, the coordinator,
the allocator, the lease manager, perception and telemetry import only
:mod:`parkscan_core.types` and never learn which one they are talking to.

Why it is this narrow
---------------------
The classic failure of "pluggable backends" is an abstraction so generic it carries no
meaning -- a bag of ``Any`` that every backend satisfies and no consumer can rely on.
This one is deliberately opinionated, and the narrowness *is* the design:

* **2D and metric.** Poses are :class:`~parkscan_core.types.Pose2D`; there is no z, no
  roll, no pitch in the state. A parking lot is planar and the robots are wheeled.
* **REP-103 axes.** ``x`` forward/east, ``y`` left/north, ``theta`` counter-clockwise.
  Stated once, honoured by both backends, never re-derived per call site.
* **A single flat ground plane at z = 0.** Geolocation back-projects onto it. That
  assumption is wrong for a pothole by definition -- a pothole is *below* the plane --
  and that error is measured in ``make eval`` rather than hidden.
* **Fixed dt, one tick at a time.** :meth:`WorldBackend.step` advances exactly one tick;
  there is no "run until". Determinism and replay depend on the caller owning the loop.
* **Sim clock, not wall clock.** :meth:`WorldBackend.now_ns` is the only time source
  above the adapter, so a run at 100x real time and a run at 0.3x produce identical logs.
* **Sense and actuate are per-robot and unbatched.** Three robots do not need a vectorised
  API, and a batched one would leak the backend's internal layout upwards.

Anything a backend cannot express within those bounds does not belong above the adapter.
A backend that needs more is a signal to change this Protocol deliberately -- and to
change both implementations -- not to smuggle state through a side channel.

The ROS 2 adapter, when it exists, is a thin node: subscribe ``/<ns>/odom``,
``/<ns>/scan``, ``/<ns>/camera/image_raw``; publish ``/<ns>/cmd_vel``. **No autonomy logic
lives inside ROS.** Until that backend actually runs, nothing in this repository may be
described as a ROS 2 system (ACCEPTANCE H12).
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol, runtime_checkable

from parkscan_core.lot import LotSpec
from parkscan_core.types import RobotId, SensorFrame, TwistCmd

__all__ = ["WorldBackend"]


@runtime_checkable
class WorldBackend(Protocol):
    """A steppable, sensable, actuatable 2D world containing the fleet.

    ``@runtime_checkable`` gives ``isinstance`` a method-presence check only -- it does
    not verify signatures. It is here so a mis-wired backend fails loudly at construction
    instead of at the first :meth:`sense` call; the real contract is enforced by mypy and
    by the shared backend conformance tests.
    """

    def reset(self, seed: int, lot: LotSpec) -> None:
        """Rebuild the world from ``lot`` and seed **all** randomness from ``seed``.

        After ``reset``, two backends of the same type given the same ``(seed, lot)`` must
        produce bit-identical trajectories. No global RNG: every stochastic component
        derives its stream from this seed.
        """
        ...

    def step(self, dt: float) -> None:
        """Advance the world by exactly one tick of ``dt`` seconds.

        ``dt`` is fixed for the whole run. Backends must not sub-step adaptively in a way
        that makes the result depend on wall-clock timing.
        """
        ...

    def now_ns(self) -> int:
        """Simulation time in nanoseconds since the run started. Never the wall clock."""
        ...

    def robots(self) -> Sequence[RobotId]:
        """The robots in this world, in a stable order that does not change after reset."""
        ...

    def sense(self, robot: RobotId) -> SensorFrame:
        """The current observation for ``robot``.

        ``SensorFrame.camera`` is ``None`` on ticks where the camera did not fire, which
        is the common case: the camera runs at a few Hz while the control loop runs at
        tens of Hz.
        """
        ...

    def actuate(self, robot: RobotId, cmd: TwistCmd) -> None:
        """Latch ``cmd`` as ``robot``'s commanded body velocity until the next call.

        The command takes effect on the next :meth:`step`; backends apply their own
        acceleration and saturation limits, so the *measured* twist reported by
        :meth:`sense` will generally differ from what was commanded.
        """
        ...
