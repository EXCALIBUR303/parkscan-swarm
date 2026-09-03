"""SQLAlchemy 2.0 models implementing ``docs/DATA_MODEL.md``.

Three schema decisions carry the honesty and correctness guarantees, and none of
them is incidental:

* ``data_source`` / ``det_source`` are **NOT NULL with no Python-side default**.
  A row cannot exist without stating where it came from (ACCEPTANCE H1), so the
  SIMULATED badge is rendered from data rather than from a hardcoded string.
* Positions are stored twice: ``local_x`` / ``local_y`` in the simulator's metric
  frame (the source of truth) *and* ``geog`` as ``geography(Point,4326)``, GIST
  indexed. ``geography``, never ``geometry`` — ``ST_Distance`` on
  ``geometry(4326)`` returns degrees, not metres.
* ``zone.epoch`` and ``charging_bay.lease_token`` are the fencing tokens
  (``BIGINT NOT NULL DEFAULT 0``), and ``robot.state`` (self-reported) is a
  different column from ``robot.coordinator_view`` (judged). The divergence
  between those two is exactly what the recovery logic exists to handle.

Deviation from the prose in ``DATA_MODEL.md``, recorded deliberately: ``robot``,
``zone``, ``parking_space`` and ``charging_bay`` are keyed on ``(run_id, id)``
rather than on ``id`` alone. Their ids (``robot_1``, ``zone_01``) repeat across
runs by design, and ``make bench`` writes 30 runs into one database — a bare text
primary key would collide on the second run. Every child table already carries
``run_id``, so the composite foreign keys add no columns.
"""

from __future__ import annotations

import datetime as dt
import enum
import uuid
from typing import Any

import sqlalchemy as sa
from geoalchemy2 import Geography
from geoalchemy2.elements import WKBElement
from sqlalchemy.dialects import postgresql
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column

from parkscan_core.types import (
    BayState,
    CoordView,
    DataSource,
    DetSource,
    RobotState,
    Severity,
    ZoneState,
)

#: Sentinel ``run.seed`` for a run first observed on the MQTT wire whose producer
#: never registered its provenance. Negative is impossible for a real seed, so a
#: placeholder can never be mistaken for a reproducible run. The ingest upgrades
#: the row in place as soon as a producer sends real run metadata.
UNREGISTERED_SEED = -1
UNREGISTERED = "unregistered"


def _pg_enum(enum_cls: type[enum.Enum], name: str) -> postgresql.ENUM:
    """A PostgreSQL ENUM over a ``StrEnum``, keyed on values not member names.

    ``create_type=False``: the Alembic migration owns type creation, so no model
    import can emit surprise DDL.
    """
    return postgresql.ENUM(
        enum_cls,
        name=name,
        values_callable=lambda members: [str(m.value) for m in members],
        create_type=False,
    )


DATA_SOURCE_ENUM = _pg_enum(DataSource, "data_source")
DET_SOURCE_ENUM = _pg_enum(DetSource, "det_source")
ROBOT_STATE_ENUM = _pg_enum(RobotState, "robot_state")
COORD_VIEW_ENUM = _pg_enum(CoordView, "coord_view")
ZONE_STATE_ENUM = _pg_enum(ZoneState, "zone_state")
SEVERITY_ENUM = _pg_enum(Severity, "severity")
BAY_STATE_ENUM = _pg_enum(BayState, "bay_state")

#: Every enum type the migration must create, in creation order.
PG_ENUM_TYPES: tuple[tuple[str, type[enum.Enum]], ...] = (
    ("data_source", DataSource),
    ("det_source", DetSource),
    ("robot_state", RobotState),
    ("coord_view", CoordView),
    ("zone_state", ZoneState),
    ("severity", Severity),
    ("bay_state", BayState),
)


def _point_col() -> Geography:
    return Geography(geometry_type="POINT", srid=4326, spatial_index=True)


def _polygon_col() -> Geography:
    return Geography(geometry_type="POLYGON", srid=4326, spatial_index=True)


def _uuid_pk() -> Any:
    return mapped_column(
        postgresql.UUID(as_uuid=True),
        primary_key=True,
        server_default=sa.text("gen_random_uuid()"),
    )


def _timestamptz(**kwargs: Any) -> Any:
    return mapped_column(sa.DateTime(timezone=True), **kwargs)


class Base(DeclarativeBase):
    """Declarative base. ``Base.metadata`` is what Alembic compares against."""


class Run(Base):
    """One simulation run. Everything else is scoped to it."""

    __tablename__ = "run"

    id: Mapped[uuid.UUID] = _uuid_pk()
    seed: Mapped[int] = mapped_column(sa.Integer, nullable=False)
    strategy: Mapped[str] = mapped_column(sa.Text, nullable=False)
    backend: Mapped[str] = mapped_column(sa.Text, nullable=False)
    data_source: Mapped[DataSource] = mapped_column(
        DATA_SOURCE_ENUM, nullable=False, server_default=sa.text("'simulated'::data_source")
    )
    git_sha: Mapped[str] = mapped_column(sa.Text, nullable=False)
    config_hash: Mapped[str] = mapped_column(sa.Text, nullable=False)
    started_at: Mapped[dt.datetime] = _timestamptz(nullable=False, server_default=sa.func.now())
    ended_at: Mapped[dt.datetime | None] = _timestamptz(nullable=True)


class Robot(Base):
    """Latest known state of one robot within one run.

    Almost every column is nullable on purpose: a robot id can first be seen in a
    detection message, before any telemetry has arrived. NULL then means "not yet
    observed", which is honest; a fabricated ``idle`` at ``(0, 0)`` would not be.
    """

    __tablename__ = "robot"

    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        primary_key=True,
    )
    id: Mapped[str] = mapped_column(sa.Text, primary_key=True)
    state: Mapped[RobotState | None] = mapped_column(ROBOT_STATE_ENUM, nullable=True)
    coordinator_view: Mapped[CoordView | None] = mapped_column(COORD_VIEW_ENUM, nullable=True)
    battery_soc: Mapped[float | None] = mapped_column(sa.REAL, nullable=True)
    local_x: Mapped[float | None] = mapped_column(sa.Double, nullable=True)
    local_y: Mapped[float | None] = mapped_column(sa.Double, nullable=True)
    geog: Mapped[WKBElement | None] = mapped_column(_point_col(), nullable=True)
    heading_rad: Mapped[float | None] = mapped_column(sa.REAL, nullable=True)
    current_zone_id: Mapped[str | None] = mapped_column(sa.Text, nullable=True)
    assignment_epoch: Mapped[int] = mapped_column(
        sa.BigInteger, nullable=False, server_default=sa.text("0")
    )
    last_heartbeat_at: Mapped[dt.datetime | None] = _timestamptz(nullable=True)
    data_source: Mapped[DataSource] = mapped_column(DATA_SOURCE_ENUM, nullable=False)

    __table_args__ = (
        sa.CheckConstraint(
            "battery_soc IS NULL OR (battery_soc >= 0.0 AND battery_soc <= 1.0)",
            name="ck_robot_battery_soc",
        ),
        sa.ForeignKeyConstraint(
            ["run_id", "current_zone_id"],
            ["zone.run_id", "zone.id"],
            name="fk_robot_current_zone",
            ondelete="SET NULL",
            use_alter=True,
        ),
    )


class Zone(Base):
    """One inspection zone. ``epoch`` is the assignment fencing token."""

    __tablename__ = "zone"

    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        primary_key=True,
    )
    id: Mapped[str] = mapped_column(sa.Text, primary_key=True)
    polygon: Mapped[WKBElement] = mapped_column(_polygon_col(), nullable=False)
    entry_waypoint: Mapped[WKBElement | None] = mapped_column(_point_col(), nullable=True)
    exit_waypoint: Mapped[WKBElement | None] = mapped_column(_point_col(), nullable=True)
    sweep_length_m: Mapped[float | None] = mapped_column(sa.REAL, nullable=True)
    state: Mapped[ZoneState] = mapped_column(
        ZONE_STATE_ENUM, nullable=False, server_default=sa.text("'unassigned'::zone_state")
    )
    assigned_robot_id: Mapped[str | None] = mapped_column(sa.Text, nullable=True)
    # Fencing token: a progress report is accepted only if report.epoch == zone.epoch.
    epoch: Mapped[int] = mapped_column(sa.BigInteger, nullable=False, server_default=sa.text("0"))
    cells_total: Mapped[int] = mapped_column(
        sa.Integer, nullable=False, server_default=sa.text("0")
    )
    # Why a requeued zone resumes instead of restarting.
    cells_done: Mapped[int] = mapped_column(sa.Integer, nullable=False, server_default=sa.text("0"))
    reassignment_count: Mapped[int] = mapped_column(
        sa.Integer, nullable=False, server_default=sa.text("0")
    )

    __table_args__ = (
        sa.CheckConstraint("cells_done >= 0 AND cells_done <= cells_total", name="ck_zone_cells"),
        sa.ForeignKeyConstraint(
            ["run_id", "assigned_robot_id"],
            ["robot.run_id", "robot.id"],
            name="fk_zone_assigned_robot",
            ondelete="SET NULL",
            use_alter=True,
        ),
    )


class Telemetry(Base):
    """Pose/battery time series. The high-volume table."""

    __tablename__ = "telemetry"

    id: Mapped[int] = mapped_column(sa.BigInteger, primary_key=True, autoincrement=True)
    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str] = mapped_column(sa.Text, nullable=False)
    stamp: Mapped[dt.datetime] = _timestamptz(nullable=False)
    local_x: Mapped[float | None] = mapped_column(sa.Double, nullable=True)
    local_y: Mapped[float | None] = mapped_column(sa.Double, nullable=True)
    geog: Mapped[WKBElement | None] = mapped_column(_point_col(), nullable=True)
    heading_rad: Mapped[float | None] = mapped_column(sa.REAL, nullable=True)
    battery_soc: Mapped[float | None] = mapped_column(sa.REAL, nullable=True)
    state: Mapped[RobotState | None] = mapped_column(ROBOT_STATE_ENUM, nullable=True)
    data_source: Mapped[DataSource] = mapped_column(DATA_SOURCE_ENUM, nullable=False)

    __table_args__ = (
        sa.ForeignKeyConstraint(
            ["run_id", "robot_id"],
            ["robot.run_id", "robot.id"],
            name="fk_telemetry_robot",
            ondelete="CASCADE",
        ),
        sa.CheckConstraint(
            "battery_soc IS NULL OR (battery_soc >= 0.0 AND battery_soc <= 1.0)",
            name="ck_telemetry_battery_soc",
        ),
        sa.Index("ix_telemetry_run_robot_stamp", "run_id", "robot_id", sa.text("stamp DESC")),
    )


class Detection(Base):
    """One pothole or space-occupancy observation.

    ``local_x`` / ``local_y`` / ``geog`` are NULL when ``geoloc_valid`` is false:
    a ray at or above the horizon is *rejected*, never clamped, so there is no
    world point to store (ACCEPTANCE C11).
    """

    __tablename__ = "detection"

    id: Mapped[uuid.UUID] = _uuid_pk()
    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str] = mapped_column(sa.Text, nullable=False)
    kind: Mapped[str] = mapped_column(sa.Text, nullable=False)
    # No default, ever: a detection that cannot say where it came from is a bug.
    det_source: Mapped[DetSource] = mapped_column(DET_SOURCE_ENUM, nullable=False)
    data_source: Mapped[DataSource] = mapped_column(DATA_SOURCE_ENUM, nullable=False)
    detector_name: Mapped[str] = mapped_column(sa.Text, nullable=False)
    confidence: Mapped[float] = mapped_column(sa.REAL, nullable=False)
    severity: Mapped[Severity | None] = mapped_column(SEVERITY_ENUM, nullable=True)
    bbox_x1: Mapped[float] = mapped_column(sa.REAL, nullable=False)
    bbox_y1: Mapped[float] = mapped_column(sa.REAL, nullable=False)
    bbox_x2: Mapped[float] = mapped_column(sa.REAL, nullable=False)
    bbox_y2: Mapped[float] = mapped_column(sa.REAL, nullable=False)
    local_x: Mapped[float | None] = mapped_column(sa.Double, nullable=True)
    local_y: Mapped[float | None] = mapped_column(sa.Double, nullable=True)
    geog: Mapped[WKBElement | None] = mapped_column(_point_col(), nullable=True)
    range_m: Mapped[float] = mapped_column(sa.REAL, nullable=False)
    geoloc_valid: Mapped[bool] = mapped_column(sa.Boolean, nullable=False)
    evidence_path: Mapped[str | None] = mapped_column(sa.Text, nullable=True)
    stamp: Mapped[dt.datetime] = _timestamptz(nullable=False)

    __table_args__ = (
        sa.ForeignKeyConstraint(
            ["run_id", "robot_id"],
            ["robot.run_id", "robot.id"],
            name="fk_detection_robot",
            ondelete="CASCADE",
        ),
        sa.CheckConstraint("confidence >= 0.0 AND confidence <= 1.0", name="ck_detection_conf"),
        sa.CheckConstraint("bbox_x1 <= bbox_x2 AND bbox_y1 <= bbox_y2", name="ck_detection_bbox"),
        sa.Index("ix_detection_run_stamp", "run_id", sa.text("stamp DESC")),
        sa.Index("ix_detection_kind", "run_id", "kind"),
    )


class ParkingSpace(Base):
    """A marked stall and its last observed occupancy."""

    __tablename__ = "parking_space"

    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        primary_key=True,
    )
    id: Mapped[str] = mapped_column(sa.Text, primary_key=True)
    zone_id: Mapped[str | None] = mapped_column(sa.Text, nullable=True)
    polygon: Mapped[WKBElement] = mapped_column(_polygon_col(), nullable=False)
    state: Mapped[BayState] = mapped_column(
        BAY_STATE_ENUM, nullable=False, server_default=sa.text("'unknown'::bay_state")
    )
    last_observed_at: Mapped[dt.datetime | None] = _timestamptz(nullable=True)
    last_detection_id: Mapped[uuid.UUID | None] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("detection.id", ondelete="SET NULL"),
        nullable=True,
    )
    data_source: Mapped[DataSource] = mapped_column(DATA_SOURCE_ENUM, nullable=False)

    __table_args__ = (
        sa.ForeignKeyConstraint(
            ["run_id", "zone_id"],
            ["zone.run_id", "zone.id"],
            name="fk_parking_space_zone",
            ondelete="SET NULL",
        ),
    )


class ChargingBay(Base):
    """A charging bay. ``lease_token`` is the fencing token a TTL cannot provide."""

    __tablename__ = "charging_bay"

    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        primary_key=True,
    )
    id: Mapped[str] = mapped_column(sa.Text, primary_key=True)
    local_x: Mapped[float | None] = mapped_column(sa.Double, nullable=True)
    local_y: Mapped[float | None] = mapped_column(sa.Double, nullable=True)
    geog: Mapped[WKBElement | None] = mapped_column(_point_col(), nullable=True)
    lease_holder_id: Mapped[str | None] = mapped_column(sa.Text, nullable=True)
    # Monotonic, only ever increases. enter_bay(robot, token) rejects token < lease_token.
    lease_token: Mapped[int] = mapped_column(
        sa.BigInteger, nullable=False, server_default=sa.text("0")
    )
    lease_expires_at: Mapped[dt.datetime | None] = _timestamptz(nullable=True)

    __table_args__ = (
        sa.ForeignKeyConstraint(
            ["run_id", "lease_holder_id"],
            ["robot.run_id", "robot.id"],
            name="fk_charging_bay_holder",
            ondelete="SET NULL",
        ),
        sa.CheckConstraint("lease_token >= 0", name="ck_charging_bay_token"),
    )


class ResourceLease(Base):
    """Audit row per lease grant on a lane segment or a bay."""

    __tablename__ = "resource_lease"

    id: Mapped[int] = mapped_column(sa.BigInteger, primary_key=True, autoincrement=True)
    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        nullable=False,
    )
    resource_id: Mapped[str] = mapped_column(sa.Text, nullable=False)
    robot_id: Mapped[str] = mapped_column(sa.Text, nullable=False)
    token: Mapped[int] = mapped_column(sa.BigInteger, nullable=False)
    granted_at: Mapped[dt.datetime] = _timestamptz(nullable=False, server_default=sa.func.now())
    expires_at: Mapped[dt.datetime] = _timestamptz(nullable=False)
    released_at: Mapped[dt.datetime | None] = _timestamptz(nullable=True)
    #: released | expired | fenced_out. NULL while the lease is still held.
    outcome: Mapped[str | None] = mapped_column(sa.Text, nullable=True)

    __table_args__ = (
        sa.ForeignKeyConstraint(
            ["run_id", "robot_id"],
            ["robot.run_id", "robot.id"],
            name="fk_resource_lease_robot",
            ondelete="CASCADE",
        ),
        sa.CheckConstraint(
            "outcome IS NULL OR outcome IN ('released','expired','fenced_out')",
            name="ck_resource_lease_outcome",
        ),
        sa.Index("ix_resource_lease_resource", "run_id", "resource_id", sa.text("granted_at DESC")),
    )


class Alert(Base):
    """Operator-visible event."""

    __tablename__ = "alert"

    id: Mapped[uuid.UUID] = _uuid_pk()
    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str | None] = mapped_column(sa.Text, nullable=True)
    zone_id: Mapped[str | None] = mapped_column(sa.Text, nullable=True)
    kind: Mapped[str] = mapped_column(sa.Text, nullable=False)
    message: Mapped[str] = mapped_column(sa.Text, nullable=False)
    stamp: Mapped[dt.datetime] = _timestamptz(nullable=False, server_default=sa.func.now())
    acknowledged: Mapped[bool] = mapped_column(
        sa.Boolean, nullable=False, server_default=sa.text("false")
    )

    __table_args__ = (
        sa.ForeignKeyConstraint(
            ["run_id", "robot_id"],
            ["robot.run_id", "robot.id"],
            name="fk_alert_robot",
            ondelete="CASCADE",
        ),
        sa.ForeignKeyConstraint(
            ["run_id", "zone_id"],
            ["zone.run_id", "zone.id"],
            name="fk_alert_zone",
            ondelete="CASCADE",
        ),
        sa.Index("ix_alert_run_stamp", "run_id", sa.text("stamp DESC")),
    )


class MetricSample(Base):
    """What ``make bench`` writes (Phase 4). Named metrics only — see DATA_MODEL."""

    __tablename__ = "metric_sample"

    id: Mapped[int] = mapped_column(sa.BigInteger, primary_key=True, autoincrement=True)
    run_id: Mapped[uuid.UUID] = mapped_column(
        postgresql.UUID(as_uuid=True),
        sa.ForeignKey("run.id", ondelete="CASCADE"),
        nullable=False,
    )
    name: Mapped[str] = mapped_column(sa.Text, nullable=False)
    value: Mapped[float] = mapped_column(sa.Double, nullable=False)
    unit: Mapped[str] = mapped_column(sa.Text, nullable=False)

    __table_args__ = (sa.Index("ix_metric_sample_run_name", "run_id", "name"),)


__all__ = [
    "PG_ENUM_TYPES",
    "UNREGISTERED",
    "UNREGISTERED_SEED",
    "Alert",
    "Base",
    "ChargingBay",
    "Detection",
    "MetricSample",
    "ParkingSpace",
    "ResourceLease",
    "Robot",
    "Run",
    "Telemetry",
    "Zone",
]
