"""Pydantic v2 wire models: MQTT ingest payloads and REST/WebSocket responses.

Two rules hold throughout:

* Every response carries ``data_source``. The dashboard's SIMULATED badge is
  rendered from that field, never from a hardcoded string (ACCEPTANCE H1/H2).
* ``data_source`` and ``det_source`` have **no defaults** on the ingest models.
  A producer that does not say where its data came from gets a 4xx-shaped
  validation failure and the message is dropped, rather than being silently
  recorded as simulated.
"""

from __future__ import annotations

import datetime as dt
from typing import Annotated, Any, Literal, Self

from pydantic import AwareDatetime, BaseModel, ConfigDict, Field, model_validator

from parkscan_core.types import (
    BayState,
    CoordView,
    DataSource,
    DetKind,
    DetSource,
    RobotState,
    Severity,
    ZoneState,
)

Unit = Annotated[float, Field(ge=0.0, le=1.0)]

_NS_PER_S = 1_000_000_000


def _stamp_from(stamp: dt.datetime | None, stamp_ns: int | None) -> dt.datetime:
    """Resolve a message timestamp.

    Precedence: an explicit aware ``stamp`` wins; otherwise ``stamp_ns`` is read
    as nanoseconds on the *simulator* clock and mapped through the UTC epoch (so
    a sim that starts at t=0 produces obviously synthetic 1970 timestamps rather
    than a plausible-looking wall time); otherwise ingest wall time.
    """
    if stamp is not None:
        return stamp.astimezone(dt.UTC)
    if stamp_ns is not None:
        return dt.datetime.fromtimestamp(stamp_ns / _NS_PER_S, tz=dt.UTC)
    return dt.datetime.now(tz=dt.UTC)


class _Wire(BaseModel):
    """Base for anything read off the wire: unknown fields are an error."""

    model_config = ConfigDict(extra="forbid")


# --------------------------------------------------------------------- ingest --
class RunMeta(_Wire):
    """Optional provenance a producer may attach to its first message.

    When present it upgrades the placeholder ``run`` row the ingest created, so a
    run stops being ``seed = -1 / unregistered`` and becomes replayable.
    """

    seed: int = Field(ge=0)
    strategy: str = Field(min_length=1)
    backend: str = Field(min_length=1)
    git_sha: str = Field(min_length=1)
    config_hash: str = Field(min_length=1)


class PoseIn(_Wire):
    """REP-103: +x forward/east, +y left/north, theta CCW."""

    x_m: float
    y_m: float
    theta_rad: float


class TwistIn(_Wire):
    v_mps: float
    omega_radps: float


class BBoxIn(_Wire):
    x1: float
    y1: float
    x2: float
    y2: float

    @model_validator(mode="after")
    def _ordered(self) -> Self:
        if self.x1 > self.x2 or self.y1 > self.y2:
            msg = "bbox corners are out of order (need x1 <= x2 and y1 <= y2)"
            raise ValueError(msg)
        return self


class TelemetryMessage(_Wire):
    """``parkscan/v1/{run_id}/robot/{id}/telemetry``."""

    pose: PoseIn
    battery_soc: Unit
    state: RobotState
    data_source: DataSource
    twist: TwistIn | None = None
    coordinator_view: CoordView | None = None
    zone_id: str | None = None
    epoch: int = Field(default=0, ge=0)
    stamp: AwareDatetime | None = None
    stamp_ns: int | None = Field(default=None, ge=0)
    run: RunMeta | None = None

    def resolved_stamp(self) -> dt.datetime:
        return _stamp_from(self.stamp, self.stamp_ns)


class ProgressMessage(_Wire):
    """``parkscan/v1/{run_id}/robot/{id}/progress``.

    ``cells_done_total`` is CUMULATIVE and is stored by assignment, never by addition:
    QoS 1 is at-least-once, and a redelivered increment would report coverage that never
    happened.
    """

    zone_id: str = Field(min_length=1)
    epoch: int = Field(ge=0)
    cells_done: list[str] = Field(default_factory=list)
    cells_done_total: int = Field(ge=0)
    cells_total: int = Field(ge=0)
    stamp: AwareDatetime | None = None
    stamp_ns: int | None = Field(default=None, ge=0)

    @model_validator(mode="after")
    def _totals_consistent(self) -> Self:
        if self.cells_done_total > self.cells_total:
            msg = f"cells_done_total={self.cells_done_total} exceeds cells_total={self.cells_total}"
            raise ValueError(msg)
        return self

    def resolved_stamp(self) -> dt.datetime:
        return _stamp_from(self.stamp, self.stamp_ns)


class CoverageOut(BaseModel):
    """One zone's coverage, for the dashboard choropleth."""

    zone_id: str
    state: ZoneState
    cells_done: int
    cells_total: int
    coverage: float
    assigned_robot_id: str | None
    epoch: int
    data_source: DataSource


class DetectionMessage(_Wire):
    """``parkscan/v1/{run_id}/robot/{id}/detection``."""

    kind: DetKind
    det_source: DetSource
    data_source: DataSource
    detector_name: str = Field(min_length=1)
    confidence: Unit
    bbox: BBoxIn
    range_m: float = Field(ge=0.0)
    geoloc_valid: bool
    severity: Severity | None = None
    local_x: float | None = None
    local_y: float | None = None
    evidence_path: str | None = None
    stamp: AwareDatetime | None = None
    stamp_ns: int | None = Field(default=None, ge=0)
    run: RunMeta | None = None

    @model_validator(mode="after")
    def _geoloc_consistent(self) -> Self:
        """A valid geolocation must carry a point; an invalid one must not.

        Rays at or above the horizon are rejected, never clamped (C11), so an
        invalid detection has no world position to record — storing one anyway is
        exactly the failure mode this check exists to prevent.
        """
        has_point = self.local_x is not None and self.local_y is not None
        if self.geoloc_valid and not has_point:
            msg = "geoloc_valid is true but local_x/local_y are missing"
            raise ValueError(msg)
        if not self.geoloc_valid and has_point:
            msg = "geoloc_valid is false but local_x/local_y were supplied"
            raise ValueError(msg)
        return self

    def resolved_stamp(self) -> dt.datetime:
        return _stamp_from(self.stamp, self.stamp_ns)


class SubscribeCommand(_Wire):
    """The only client to server message ``/ws/live`` accepts."""

    type: Literal["subscribe"]
    topics: list[str] = Field(default_factory=list)


# ------------------------------------------------------------------ responses --
class ComponentStatus(BaseModel):
    """Liveness of one dependency. ``detail`` is set only when it is not ``ok``."""

    status: Literal["ok", "error", "stopped", "connecting"]
    detail: str | None = None


class HealthResponse(BaseModel):
    """``GET /api/v1/health`` — 200 even when degraded, with per-dependency detail."""

    status: Literal["ok", "degraded"]
    db: ComponentStatus
    mqtt: ComponentStatus
    version: str
    git_sha: str
    run_id: str
    data_source: DataSource


class ProblemDetail(BaseModel):
    """RFC 9457 ``application/problem+json``."""

    type: str = "about:blank"
    title: str
    status: int
    detail: str | None = None
    instance: str | None = None
    errors: list[dict[str, Any]] | None = None


class RunOut(BaseModel):
    id: str
    seed: int
    strategy: str
    backend: str
    git_sha: str
    config_hash: str
    started_at: dt.datetime
    ended_at: dt.datetime | None
    data_source: DataSource
    registered: bool = Field(
        description="False for a placeholder run created by the ingest before any "
        "producer registered its seed and configuration."
    )


class RobotOut(BaseModel):
    id: str
    run_id: str
    state: RobotState | None
    coordinator_view: CoordView | None
    battery_soc: float | None
    local_x: float | None
    local_y: float | None
    heading_rad: float | None
    current_zone_id: str | None
    assignment_epoch: int
    last_heartbeat_at: dt.datetime | None
    data_source: DataSource


class ZoneOut(BaseModel):
    id: str
    run_id: str
    polygon: dict[str, Any] | None = Field(description="GeoJSON geometry, WGS84.")
    sweep_length_m: float | None
    state: ZoneState
    assigned_robot_id: str | None
    epoch: int
    cells_total: int
    cells_done: int
    reassignment_count: int
    data_source: DataSource


class DetectionOut(BaseModel):
    id: str
    run_id: str
    robot_id: str
    kind: str
    det_source: DetSource
    data_source: DataSource
    detector_name: str
    confidence: float
    severity: Severity | None
    bbox: BBoxIn
    local_x: float | None
    local_y: float | None
    range_m: float
    geoloc_valid: bool
    evidence_path: str | None
    stamp: dt.datetime


class ParkingSpaceOut(BaseModel):
    id: str
    run_id: str
    zone_id: str | None
    polygon: dict[str, Any] | None = Field(description="GeoJSON geometry, WGS84.")
    state: BayState
    last_observed_at: dt.datetime | None
    last_detection_id: str | None
    data_source: DataSource


class WsEnvelope(BaseModel):
    """The single server to client envelope on ``/ws/live``."""

    type: Literal["telemetry", "detection", "zone", "alert", "assignment", "run_state", "lag"]
    stamp: dt.datetime
    data_source: DataSource
    payload: dict[str, Any]
