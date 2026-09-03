"""Persistence for messages arriving off MQTT.

Kept separate from :mod:`parkscan_api.mqtt` so the database side is testable
against a real PostGIS instance without a broker, and the broker side is
testable without asserting on SQL.

Positions arrive in the simulator's metric frame. That frame is the source of
truth and is stored verbatim in ``local_x`` / ``local_y``; ``geog`` is derived
from it through the authored lot origin. When the lot spec cannot be read the
origin is unknown, and ``geog`` is written NULL rather than guessed — a wrong
origin would put every row on the map in the wrong place, silently.
"""

from __future__ import annotations

import uuid
from typing import Any

import sqlalchemy as sa
from geoalchemy2.elements import WKTElement
from sqlalchemy.dialects.postgresql import insert as pg_insert
from sqlalchemy.ext.asyncio import AsyncSession

from parkscan_api.models import (
    UNREGISTERED,
    UNREGISTERED_SEED,
    Detection,
    Robot,
    Run,
    Telemetry,
    Zone,
)
from parkscan_api.schemas import DetectionMessage, RunMeta, TelemetryMessage
from parkscan_core.geo import Origin, lot_to_lonlat
from parkscan_core.types import DataSource


def point_wkt(lon_deg: float, lat_deg: float) -> WKTElement:
    """A WGS84 point for a ``geography(Point,4326)`` column."""
    return WKTElement(f"POINT({lon_deg!r} {lat_deg!r})", srid=4326)


def project(
    local_x: float | None, local_y: float | None, origin: Origin | None
) -> WKTElement | None:
    """Lot metres -> ``geography`` point, or ``None`` when either input is unknown."""
    if origin is None or local_x is None or local_y is None:
        return None
    lon, lat = lot_to_lonlat(local_x, local_y, origin)
    return point_wkt(lon, lat)


async def ensure_run(
    session: AsyncSession,
    run_uuid: uuid.UUID,
    data_source: DataSource,
    *,
    meta: RunMeta | None = None,
) -> None:
    """Make sure the run row exists.

    With no ``meta`` this inserts a placeholder: ``seed = -1`` and every text
    field ``unregistered``. A negative seed is impossible for a real run, so a
    placeholder can never be mistaken for something reproducible. When a producer
    later attaches :class:`RunMeta`, the same statement upgrades the row in place.
    """
    values: dict[str, Any] = {
        "id": run_uuid,
        "seed": UNREGISTERED_SEED,
        "strategy": UNREGISTERED,
        "backend": UNREGISTERED,
        "git_sha": UNREGISTERED,
        "config_hash": UNREGISTERED,
        "data_source": data_source,
    }
    if meta is not None:
        values.update(
            seed=meta.seed,
            strategy=meta.strategy,
            backend=meta.backend,
            git_sha=meta.git_sha,
            config_hash=meta.config_hash,
        )
    stmt = pg_insert(Run).values(**values)
    if meta is None:
        stmt = stmt.on_conflict_do_nothing(index_elements=["id"])
    else:
        stmt = stmt.on_conflict_do_update(
            index_elements=["id"],
            set_={
                "seed": stmt.excluded.seed,
                "strategy": stmt.excluded.strategy,
                "backend": stmt.excluded.backend,
                "git_sha": stmt.excluded.git_sha,
                "config_hash": stmt.excluded.config_hash,
            },
        )
    await session.execute(stmt)


async def ensure_robot(
    session: AsyncSession, run_uuid: uuid.UUID, robot_id: str, data_source: DataSource
) -> None:
    """Insert a bare robot row if this id has never been seen.

    Every observation column stays NULL: "seen in a detection, no telemetry yet"
    is the truth, and a fabricated ``idle`` at the origin would not be.
    """
    stmt = (
        pg_insert(Robot)
        .values(run_id=run_uuid, id=robot_id, data_source=data_source)
        .on_conflict_do_nothing(index_elements=["run_id", "id"])
    )
    await session.execute(stmt)


def _known_zone(run_uuid: uuid.UUID, zone_id: str | None) -> Any:
    """Scalar subquery yielding the zone id only if that zone exists.

    ``robot.current_zone_id`` is a foreign key, and telemetry can name a zone the
    database has not been told about yet. Resolving through a subquery keeps the
    write to a single statement and turns an unknown zone into NULL instead of an
    integrity error that would drop the whole telemetry row.
    """
    if zone_id is None:
        return sa.null()
    return sa.select(Zone.id).where(Zone.run_id == run_uuid, Zone.id == zone_id).scalar_subquery()


async def record_telemetry(
    session: AsyncSession,
    run_uuid: uuid.UUID,
    robot_id: str,
    message: TelemetryMessage,
    *,
    origin: Origin | None,
) -> dict[str, Any]:
    """Upsert the robot's latest state and append one telemetry row.

    Returns the WebSocket payload for this sample.
    """
    stamp = message.resolved_stamp()
    geog = project(message.pose.x_m, message.pose.y_m, origin)

    await ensure_run(session, run_uuid, message.data_source, meta=message.run)

    robot_values: dict[str, Any] = {
        "run_id": run_uuid,
        "id": robot_id,
        "state": message.state,
        "coordinator_view": message.coordinator_view,
        "battery_soc": message.battery_soc,
        "local_x": message.pose.x_m,
        "local_y": message.pose.y_m,
        "geog": geog,
        "heading_rad": message.pose.theta_rad,
        "current_zone_id": _known_zone(run_uuid, message.zone_id),
        "assignment_epoch": message.epoch,
        "last_heartbeat_at": stamp,
        "data_source": message.data_source,
    }
    robot_stmt = pg_insert(Robot).values(**robot_values)
    await session.execute(
        robot_stmt.on_conflict_do_update(
            index_elements=["run_id", "id"],
            set_={
                key: robot_stmt.excluded[key]
                for key in (
                    "state",
                    "coordinator_view",
                    "battery_soc",
                    "local_x",
                    "local_y",
                    "geog",
                    "heading_rad",
                    "current_zone_id",
                    "assignment_epoch",
                    "last_heartbeat_at",
                    "data_source",
                )
            },
        )
    )

    session.add(
        Telemetry(
            run_id=run_uuid,
            robot_id=robot_id,
            stamp=stamp,
            local_x=message.pose.x_m,
            local_y=message.pose.y_m,
            geog=geog,
            heading_rad=message.pose.theta_rad,
            battery_soc=message.battery_soc,
            state=message.state,
            data_source=message.data_source,
        )
    )

    return {
        "run_id": str(run_uuid),
        "robot_id": robot_id,
        "stamp": stamp.isoformat(),
        "local_x": message.pose.x_m,
        "local_y": message.pose.y_m,
        "heading_rad": message.pose.theta_rad,
        "battery_soc": message.battery_soc,
        "state": message.state.value,
        "coordinator_view": (
            message.coordinator_view.value if message.coordinator_view is not None else None
        ),
        "zone_id": message.zone_id,
        "epoch": message.epoch,
        "data_source": message.data_source.value,
    }


async def record_detection(
    session: AsyncSession,
    run_uuid: uuid.UUID,
    robot_id: str,
    message: DetectionMessage,
    *,
    origin: Origin | None,
) -> dict[str, Any]:
    """Insert one detection. Returns the WebSocket payload."""
    stamp = message.resolved_stamp()
    geog = project(message.local_x, message.local_y, origin)

    await ensure_run(session, run_uuid, message.data_source, meta=message.run)
    await ensure_robot(session, run_uuid, robot_id, message.data_source)

    detection = Detection(
        run_id=run_uuid,
        robot_id=robot_id,
        kind=message.kind.value,
        det_source=message.det_source,
        data_source=message.data_source,
        detector_name=message.detector_name,
        confidence=message.confidence,
        severity=message.severity,
        bbox_x1=message.bbox.x1,
        bbox_y1=message.bbox.y1,
        bbox_x2=message.bbox.x2,
        bbox_y2=message.bbox.y2,
        local_x=message.local_x,
        local_y=message.local_y,
        geog=geog,
        range_m=message.range_m,
        geoloc_valid=message.geoloc_valid,
        evidence_path=message.evidence_path,
        stamp=stamp,
    )
    session.add(detection)
    await session.flush()

    return {
        "id": str(detection.id),
        "run_id": str(run_uuid),
        "robot_id": robot_id,
        "kind": message.kind.value,
        "det_source": message.det_source.value,
        "data_source": message.data_source.value,
        "detector_name": message.detector_name,
        "confidence": message.confidence,
        "severity": message.severity.value if message.severity is not None else None,
        "bbox": {
            "x1": message.bbox.x1,
            "y1": message.bbox.y1,
            "x2": message.bbox.x2,
            "y2": message.bbox.y2,
        },
        "local_x": message.local_x,
        "local_y": message.local_y,
        "range_m": message.range_m,
        "geoloc_valid": message.geoloc_valid,
        "evidence_path": message.evidence_path,
        "stamp": stamp.isoformat(),
    }
