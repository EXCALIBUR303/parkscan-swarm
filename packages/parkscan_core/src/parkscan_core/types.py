"""Core value types shared by every layer above the simulator adapter.

Everything here is deliberately small, frozen and backend-agnostic. The autonomy
stack, the coordinator, perception and the telemetry publisher import *only*
these types; none of them ever learns which :class:`~parkscan_core.world.WorldBackend`
produced the data.

Two conventions are load-bearing and are stated once, here:

* **REP-103 axes.** ``x`` forward / east, ``y`` left / north, ``theta`` counter-clockwise
  from ``+x``, all in metres and radians. There is a single flat ground plane at ``z = 0``.
* **Honesty is structural.** ``DataSource`` / ``DetSource`` are required fields on the
  observation types and NOT NULL columns in the database (see ``docs/DATA_MODEL.md``).
  Nothing in this package supplies a default for them, so a caller cannot forget to
  say where an observation came from.

The enums are :class:`enum.StrEnum` so ``str(value)`` and JSON serialisation both yield
the exact lowercase spelling used by the PostgreSQL enum types and the MQTT/WebSocket
envelopes in ``docs/API_CONTRACT.md``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

import numpy as np
import numpy.typing as npt

__all__ = [
    "BBox",
    "BayState",
    "CoordView",
    "DataSource",
    "DetKind",
    "DetSource",
    "Detection",
    "Frame",
    "Pose2D",
    "RobotId",
    "RobotState",
    "SensorFrame",
    "Severity",
    "TwistCmd",
    "ZoneState",
]

type RobotId = str
"""Stable robot identifier, e.g. ``"robot_1"``. Matches ``robot.id`` in the schema."""


# --------------------------------------------------------------------------- enums --
class DataSource(StrEnum):
    """Provenance of a telemetry-bearing row. NOT NULL in every observation table."""

    SIMULATED = "simulated"
    REPLAY = "replay"
    LIVE = "live"


class DetSource(StrEnum):
    """Provenance of a detection. ``SIMULATED`` is what drives the SIMULATED badge."""

    SIMULATED = "simulated"
    MODEL = "model"
    GROUND_TRUTH = "ground_truth"


class DetKind(StrEnum):
    """What a detection is about."""

    POTHOLE = "pothole"
    SPACE_OCCUPANCY = "space_occupancy"


class Severity(StrEnum):
    """Pothole severity. ``None`` for kinds that do not carry one."""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class RobotState(StrEnum):
    """The robot's own view of itself, reported in telemetry."""

    IDLE = "idle"
    ASSIGNED = "assigned"
    TRANSIT = "transit"
    INSPECTING = "inspecting"
    WAITING_ON_LEASE = "waiting_on_lease"
    RETURNING = "returning"
    CHARGING = "charging"
    FAILED = "failed"
    OFFLINE = "offline"


class CoordView(StrEnum):
    """The coordinator's *independent* judgement of a robot.

    Deliberately not merged with :class:`RobotState`: a robot that believes it is
    ``INSPECTING`` while the coordinator has declared it ``OFFLINE`` is precisely the
    case the recovery logic exists to handle.
    """

    ONLINE = "online"
    SUSPECT = "suspect"
    OFFLINE = "offline"
    STUCK = "stuck"


class ZoneState(StrEnum):
    """Lifecycle of one inspection zone."""

    UNASSIGNED = "unassigned"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETE = "complete"
    UNRECOVERABLE = "unrecoverable"


class BayState(StrEnum):
    """Occupancy of a parking space. ``UNKNOWN`` until it has actually been observed."""

    VACANT = "vacant"
    OCCUPIED = "occupied"
    UNKNOWN = "unknown"


# ----------------------------------------------------------------------- geometry --
@dataclass(frozen=True, slots=True)
class Pose2D:
    """Planar pose in the lot's metric frame (REP-103)."""

    x_m: float
    y_m: float
    theta_rad: float


@dataclass(frozen=True, slots=True)
class TwistCmd:
    """Body-frame velocity: forward speed and yaw rate (REP-103)."""

    v_mps: float
    omega_radps: float


@dataclass(frozen=True, slots=True, eq=False)
class Frame:
    """One camera frame.

    ``image`` is an ``(height, width, 3)`` uint8 array, or ``None`` when the backend
    emits geometry-only frames (``sim-lite`` without synthetic rendering). Structural
    equality is disabled (``eq=False``) because comparing two frames would compare two
    numpy arrays, which returns an array rather than a bool; identity comparison is the
    only meaningful default here.

    The frame deliberately carries **no robot pose**. Geolocation is a separate stage;
    a detector that could see the map would be able to cheat.
    """

    frame_id: int
    stamp_ns: int
    camera_id: str
    width: int
    height: int
    image: npt.NDArray[np.uint8] | None

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            msg = f"frame dimensions must be positive, got {self.width}x{self.height}"
            raise ValueError(msg)
        if self.image is not None and self.image.shape[:2] != (self.height, self.width):
            msg = (
                f"image shape {self.image.shape!r} does not match declared "
                f"{self.height}x{self.width}"
            )
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class BBox:
    """Axis-aligned box in image pixels, origin top-left, ``y`` increasing downward."""

    x1: float
    y1: float
    x2: float
    y2: float

    def __post_init__(self) -> None:
        if self.x1 > self.x2 or self.y1 > self.y2:
            msg = f"degenerate bbox: ({self.x1}, {self.y1}) .. ({self.x2}, {self.y2})"
            raise ValueError(msg)

    @property
    def bottom_centre(self) -> tuple[float, float]:
        """The ground-contact point: horizontal centre of the **bottom** edge.

        This exists as a property, and not as an inline expression at the call site,
        so that the geolocation stage cannot accidentally back-project the box *centre*.
        Back-projecting the centre of a pothole box puts the pothole roughly half a box
        height too far away, and the error is invisible in the output because the result
        still looks like a plausible point on the ground.
        """
        return ((self.x1 + self.x2) / 2.0, self.y2)

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1


# -------------------------------------------------------------------- observations --
@dataclass(frozen=True, slots=True)
class Detection:
    """One detection in **image space**.

    There is no world position here on purpose: a :class:`Detection` is what a
    :class:`~parkscan_core.world.WorldBackend`-independent detector returns, and
    geolocation happens afterwards in its own testable stage.

    ``det_source`` has no default. Persisting a detection without saying whether it came
    from a simulator, a model or ground truth must be impossible, not merely discouraged.
    """

    kind: DetKind
    det_source: DetSource
    detector_name: str
    confidence: float
    severity: Severity | None
    bbox: BBox
    stamp_ns: int

    def __post_init__(self) -> None:
        if not 0.0 <= self.confidence <= 1.0:
            msg = f"confidence must be in [0, 1], got {self.confidence}"
            raise ValueError(msg)
        if not self.detector_name:
            msg = "detector_name must be non-empty"
            raise ValueError(msg)


@dataclass(frozen=True, slots=True)
class SensorFrame:
    """Everything one robot observes on one tick.

    ``twist`` is the *measured* body velocity, not the commanded one. ``ranges`` is a
    fixed-count planar range scan in metres with ``inf`` meaning "no return"; the count
    and angular layout are fixed by the backend for the whole run.
    """

    stamp_ns: int
    pose: Pose2D
    twist: TwistCmd
    battery_soc: float
    ranges: tuple[float, ...]
    camera: Frame | None

    def __post_init__(self) -> None:
        if not 0.0 <= self.battery_soc <= 1.0:
            msg = f"battery_soc must be in [0, 1], got {self.battery_soc}"
            raise ValueError(msg)
