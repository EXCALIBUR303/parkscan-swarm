"""REST and WebSocket routes for Phase 1.

Scope note: the routes here are the read side plus ``/health``. ``/tasks``,
``/coverage``, ``/alerts``, ``/reports`` and the evidence endpoint from
``docs/API_CONTRACT.md`` arrive with the phases that produce their data — an
endpoint that can only ever return ``[]`` is not a delivered endpoint.
"""

from __future__ import annotations

import datetime as dt
import json
from typing import Annotated, Any, Literal

import sqlalchemy as sa
import structlog
from fastapi import APIRouter, Query, WebSocket, WebSocketDisconnect
from geoalchemy2 import Geography
from pydantic import ValidationError

from parkscan_api.config import api_version, git_sha
from parkscan_api.deps import (
    RunIdQuery,
    SessionDep,
    SettingsDep,
    StateDep,
    app_state,
    resolve_run_uuid,
)
from parkscan_api.errors import ProblemException
from parkscan_api.models import UNREGISTERED_SEED, Detection, ParkingSpace, Robot, Run, Zone
from parkscan_api.schemas import (
    BBoxIn,
    ComponentStatus,
    CoverageOut,
    DetectionOut,
    HealthResponse,
    ParkingSpaceOut,
    RobotOut,
    RunOut,
    SubscribeCommand,
    ZoneOut,
)
from parkscan_api.ws import ClientChannel

log: structlog.stdlib.BoundLogger = structlog.get_logger("parkscan.api")

router = APIRouter(prefix="/api/v1")
ws_router = APIRouter()

MAX_LIMIT = 1000
_GEOG_POLYGON = Geography(geometry_type="POLYGON", srid=4326)


def _geojson(raw: str | None) -> dict[str, Any] | None:
    if raw is None:
        return None
    parsed: dict[str, Any] = json.loads(raw)
    return parsed


# ------------------------------------------------------------------- health --
@router.get("/health", response_model=HealthResponse, summary="Per-dependency liveness")
async def health(state: StateDep) -> HealthResponse:
    """Report each dependency separately.

    Deliberately still 200 when a dependency is down: a monitor that only sees a
    status code learns nothing, whereas ``{"db": {"status": "error", ...}}`` says
    which half of the system to look at. A hard 503 would also make the dashboard
    unable to explain itself while PostgreSQL restarts.
    """
    db_health = await state.database.health()
    mqtt_health = state.ingest.health()
    overall: Literal["ok", "degraded"] = "ok" if db_health.ok and mqtt_health.ok else "degraded"
    return HealthResponse(
        status=overall,
        db=ComponentStatus(status=db_health.status, detail=db_health.detail),
        mqtt=ComponentStatus(status=mqtt_health.status, detail=mqtt_health.detail),
        version=api_version(),
        git_sha=git_sha(),
        run_id=state.settings.run_id,
        data_source=state.settings.data_source,
    )


# --------------------------------------------------------------------- runs --
def _run_out(row: Run) -> RunOut:
    return RunOut(
        id=str(row.id),
        seed=row.seed,
        strategy=row.strategy,
        backend=row.backend,
        git_sha=row.git_sha,
        config_hash=row.config_hash,
        started_at=row.started_at,
        ended_at=row.ended_at,
        data_source=row.data_source,
        registered=row.seed != UNREGISTERED_SEED,
    )


@router.get("/runs", response_model=list[RunOut])
async def list_runs(
    session: SessionDep,
    limit: Annotated[int, Query(ge=1, le=MAX_LIMIT)] = 100,
    offset: Annotated[int, Query(ge=0)] = 0,
) -> list[RunOut]:
    stmt = sa.select(Run).order_by(Run.started_at.desc()).limit(limit).offset(offset)
    rows = (await session.execute(stmt)).scalars().all()
    return [_run_out(row) for row in rows]


@router.get("/runs/{run_id}", response_model=RunOut)
async def get_run(run_id: str, session: SessionDep, settings: SettingsDep) -> RunOut:
    run_uuid = resolve_run_uuid(run_id, settings)
    row = (await session.execute(sa.select(Run).where(Run.id == run_uuid))).scalar_one_or_none()
    if row is None:
        raise ProblemException(404, "Not Found", f"No run {run_id!r}.")
    return _run_out(row)


# ------------------------------------------------------------------- robots --
@router.get("/robots", response_model=list[RobotOut])
async def list_robots(
    session: SessionDep, settings: SettingsDep, run_id: RunIdQuery = None
) -> list[RobotOut]:
    run_uuid = resolve_run_uuid(run_id, settings)
    stmt = sa.select(Robot).where(Robot.run_id == run_uuid).order_by(Robot.id)
    rows = (await session.execute(stmt)).scalars().all()
    return [
        RobotOut(
            id=row.id,
            run_id=str(row.run_id),
            state=row.state,
            coordinator_view=row.coordinator_view,
            battery_soc=row.battery_soc,
            local_x=row.local_x,
            local_y=row.local_y,
            heading_rad=row.heading_rad,
            current_zone_id=row.current_zone_id,
            assignment_epoch=row.assignment_epoch,
            last_heartbeat_at=row.last_heartbeat_at,
            data_source=row.data_source,
        )
        for row in rows
    ]


# -------------------------------------------------------------------- zones --
@router.get("/zones", response_model=list[ZoneOut])
async def list_zones(
    session: SessionDep, settings: SettingsDep, run_id: RunIdQuery = None
) -> list[ZoneOut]:
    run_uuid = resolve_run_uuid(run_id, settings)
    stmt = (
        sa.select(Zone, sa.func.ST_AsGeoJSON(Zone.polygon).label("polygon_geojson"))
        .where(Zone.run_id == run_uuid)
        .order_by(Zone.id)
    )
    result = await session.execute(stmt)
    out: list[ZoneOut] = []
    for row, polygon_geojson in result.all():
        out.append(
            ZoneOut(
                id=row.id,
                run_id=str(row.run_id),
                polygon=_geojson(polygon_geojson),
                sweep_length_m=row.sweep_length_m,
                state=row.state,
                assigned_robot_id=row.assigned_robot_id,
                epoch=row.epoch,
                cells_total=row.cells_total,
                cells_done=row.cells_done,
                reassignment_count=row.reassignment_count,
                data_source=settings.data_source,
            )
        )
    return out


# --------------------------------------------------------------- detections --
def _parse_bbox(raw: str) -> tuple[float, float, float, float]:
    parts = raw.split(",")
    if len(parts) != 4:
        raise ProblemException(
            400, "Bad Request", "bbox must be 'minLon,minLat,maxLon,maxLat' in WGS84 degrees."
        )
    try:
        min_lon, min_lat, max_lon, max_lat = (float(p) for p in parts)
    except ValueError as exc:
        raise ProblemException(400, "Bad Request", f"bbox is not numeric: {exc}") from exc
    if min_lon > max_lon or min_lat > max_lat:
        raise ProblemException(400, "Bad Request", "bbox corners are out of order.")
    return min_lon, min_lat, max_lon, max_lat


@router.get("/coverage", response_model=list[CoverageOut])
async def coverage(
    session: SessionDep, settings: SettingsDep, run_id: RunIdQuery = None
) -> list[CoverageOut]:
    """Per-zone coverage, the data behind the dashboard's choropleth.

    Coverage is ``cells_done / cells_total`` — a discrete, countable quantity. It is
    deliberately not smoothed or interpolated into a heatmap: a kernel density estimate
    would smear across zone boundaries and imply a continuous measurement nobody took
    (ADR-0004).

    A zone with no planned cells reports 0.0 rather than 1.0. "Nothing to do" and "all
    done" are different facts, and rounding the first into the second would show a lot as
    fully inspected before a robot had moved.
    """
    run_uuid = resolve_run_uuid(run_id, settings)
    # Provenance is joined from the run rather than read off the zone. A zone is a piece of
    # the lot's geometry, not an observation, so it carries no data_source of its own; what
    # is simulated is the RUN that produced these coverage numbers.
    rows = (
        await session.execute(
            sa.select(Zone, Run.data_source)
            .join(Run, Run.id == Zone.run_id)
            .where(Zone.run_id == run_uuid)
            .order_by(Zone.id)
        )
    ).all()
    return [
        CoverageOut(
            zone_id=zone.id,
            state=zone.state,
            cells_done=zone.cells_done,
            cells_total=zone.cells_total,
            coverage=(zone.cells_done / zone.cells_total) if zone.cells_total else 0.0,
            assigned_robot_id=zone.assigned_robot_id,
            epoch=zone.epoch,
            data_source=data_source,
        )
        for zone, data_source in rows
    ]


@router.get("/detections", response_model=list[DetectionOut])
async def list_detections(
    session: SessionDep,
    settings: SettingsDep,
    run_id: RunIdQuery = None,
    kind: Annotated[str | None, Query(description="pothole | space_occupancy")] = None,
    severity: Annotated[str | None, Query(description="low | medium | high")] = None,
    min_confidence: Annotated[float | None, Query(ge=0.0, le=1.0)] = None,
    since: Annotated[dt.datetime | None, Query(description="ISO 8601, inclusive")] = None,
    robot_id: str | None = None,
    geoloc_valid: bool | None = None,
    bbox: Annotated[
        str | None, Query(description="minLon,minLat,maxLon,maxLat (WGS84), PostGIS filtered")
    ] = None,
    limit: Annotated[int, Query(ge=1, le=MAX_LIMIT)] = 200,
    offset: Annotated[int, Query(ge=0)] = 0,
) -> list[DetectionOut]:
    run_uuid = resolve_run_uuid(run_id, settings)
    stmt = sa.select(Detection).where(Detection.run_id == run_uuid)
    if kind is not None:
        stmt = stmt.where(Detection.kind == kind)
    if severity is not None:
        stmt = stmt.where(Detection.severity == severity)
    if min_confidence is not None:
        stmt = stmt.where(Detection.confidence >= min_confidence)
    if since is not None:
        stmt = stmt.where(Detection.stamp >= since)
    if robot_id is not None:
        stmt = stmt.where(Detection.robot_id == robot_id)
    if geoloc_valid is not None:
        stmt = stmt.where(Detection.geoloc_valid.is_(geoloc_valid))
    if bbox is not None:
        min_lon, min_lat, max_lon, max_lat = _parse_bbox(bbox)
        envelope = sa.cast(
            sa.func.ST_MakeEnvelope(min_lon, min_lat, max_lon, max_lat, 4326), _GEOG_POLYGON
        )
        stmt = stmt.where(sa.func.ST_Intersects(Detection.geog, envelope))
    stmt = stmt.order_by(Detection.stamp.desc()).limit(limit).offset(offset)

    rows = (await session.execute(stmt)).scalars().all()
    return [
        DetectionOut(
            id=str(row.id),
            run_id=str(row.run_id),
            robot_id=row.robot_id,
            kind=row.kind,
            det_source=row.det_source,
            data_source=row.data_source,
            detector_name=row.detector_name,
            confidence=row.confidence,
            severity=row.severity,
            bbox=BBoxIn(x1=row.bbox_x1, y1=row.bbox_y1, x2=row.bbox_x2, y2=row.bbox_y2),
            local_x=row.local_x,
            local_y=row.local_y,
            range_m=row.range_m,
            geoloc_valid=row.geoloc_valid,
            evidence_path=row.evidence_path,
            stamp=row.stamp,
        )
        for row in rows
    ]


# ---------------------------------------------------------- parking spaces --
@router.get("/parking-spaces", response_model=list[ParkingSpaceOut])
async def list_parking_spaces(
    session: SessionDep,
    settings: SettingsDep,
    run_id: RunIdQuery = None,
    state: Annotated[str | None, Query(description="vacant | occupied | unknown")] = None,
) -> list[ParkingSpaceOut]:
    run_uuid = resolve_run_uuid(run_id, settings)
    stmt = sa.select(
        ParkingSpace, sa.func.ST_AsGeoJSON(ParkingSpace.polygon).label("polygon_geojson")
    ).where(ParkingSpace.run_id == run_uuid)
    if state is not None:
        stmt = stmt.where(ParkingSpace.state == state)
    stmt = stmt.order_by(ParkingSpace.id)

    result = await session.execute(stmt)
    return [
        ParkingSpaceOut(
            id=row.id,
            run_id=str(row.run_id),
            zone_id=row.zone_id,
            polygon=_geojson(polygon_geojson),
            state=row.state,
            last_observed_at=row.last_observed_at,
            last_detection_id=(
                str(row.last_detection_id) if row.last_detection_id is not None else None
            ),
            data_source=row.data_source,
        )
        for row, polygon_geojson in result.all()
    ]


# ---------------------------------------------------------------- websocket --
@ws_router.websocket("/ws/live")
async def live(websocket: WebSocket) -> None:
    """Live fan-out.

    Read-mostly by design: the only accepted client message is ``subscribe``.
    Commands go through REST so they are logged and testable.
    """
    state = app_state(websocket)
    await websocket.accept()
    channel: ClientChannel = state.manager.connect(websocket)
    channel.offer(
        json.dumps(
            {
                "type": "run_state",
                "stamp": dt.datetime.now(tz=dt.UTC).isoformat(),
                "data_source": state.settings.data_source.value,
                "payload": {"run_id": state.settings.run_id, "connected": True},
            },
            separators=(",", ":"),
        )
    )
    try:
        while True:
            raw = await websocket.receive_text()
            try:
                command = SubscribeCommand.model_validate_json(raw)
            except ValidationError:
                await websocket.send_text(
                    json.dumps(
                        {
                            "type": "error",
                            "detail": "only {'type':'subscribe','topics':[...]} is accepted",
                        },
                        separators=(",", ":"),
                    )
                )
                continue
            channel.subscribe(command.topics or None)
    except WebSocketDisconnect:
        pass
    finally:
        await state.manager.disconnect(channel)


__all__ = ["router", "ws_router"]
