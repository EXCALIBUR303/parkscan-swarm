"""FastAPI application.

Startup and shutdown go through ``@asynccontextmanager`` lifespan. Never
``@app.on_event`` — Starlette 1.6 removed ``on_startup`` / ``on_shutdown``
outright, so that decorator is not merely deprecated here, it does not exist.

The lifespan is written so the API still comes up when its dependencies do not:
the SQLAlchemy engine is lazy and the MQTT subscriber retries in the background.
A developer with neither PostgreSQL nor mosquitto running still gets a server
that serves ``/health`` and tells them exactly which piece is missing.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

import structlog
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from parkscan_api.config import Settings, api_version, get_settings, git_sha
from parkscan_api.db import Database
from parkscan_api.deps import STATE_ATTR, AppState
from parkscan_api.errors import register_error_handlers
from parkscan_api.logging import configure_logging
from parkscan_api.mqtt import MqttIngest
from parkscan_api.routers import router, ws_router
from parkscan_api.ws import ConnectionManager
from parkscan_core.geo import Origin
from parkscan_core.lot import load_lot

log: structlog.stdlib.BoundLogger = structlog.get_logger("parkscan.api")

DESCRIPTION = """\
Backend for ParkScan Swarm: MQTT ingest, PostGIS storage, WebSocket fan-out.

Every observation carries a `data_source` / `det_source` that is NOT NULL in the
database. The dashboard's SIMULATED badge is rendered from that value, so it
cannot drift away from what the data actually is.

Local development only: no authentication, bound to 127.0.0.1. Production auth
(OAuth2 bearer, per-robot MQTT credentials, TLS) is a documented gap, not a
completed feature.
"""


def load_origin(path: Path) -> tuple[Origin | None, str | None]:
    """Read the lot origin used to project local metres to WGS84.

    Returns ``(None, reason)`` on any failure. The caller then writes ``geog``
    NULL: a guessed origin would place every row on the map somewhere plausible
    and wrong, which is worse than an absent geometry.
    """
    try:
        spec = load_lot(path)
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}".strip()[:300]
    return spec.origin.as_origin(), None


def create_app(settings: Settings | None = None) -> FastAPI:
    """Build the application. ``settings=None`` reads the process environment."""
    resolved = settings if settings is not None else get_settings()

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        configure_logging(resolved.log_level)
        database = Database(resolved.database_url)
        database.start()
        manager = ConnectionManager(
            maxsize=resolved.ws_queue_maxsize,
            send_timeout_s=resolved.ws_send_timeout_s,
        )
        origin, lot_error = load_origin(resolved.lot_path)
        if lot_error is not None:
            log.warning("lot_origin_unavailable", path=str(resolved.lot_path), detail=lot_error)
        ingest = MqttIngest(resolved, database, manager, origin=origin)
        state = AppState(
            settings=resolved,
            database=database,
            manager=manager,
            ingest=ingest,
            origin=origin,
            lot_error=lot_error,
        )
        setattr(app.state, STATE_ATTR, state)
        ingest.start()
        log.info(
            "api_started",
            version=api_version(),
            git_sha=git_sha(),
            run_id=resolved.run_id,
            data_source=resolved.data_source.value,
            mqtt=f"{resolved.mqtt_host}:{resolved.mqtt_port}",
        )
        try:
            yield
        finally:
            await ingest.stop()
            await manager.close()
            await database.stop()
            log.info("api_stopped")

    app = FastAPI(
        title="ParkScan Swarm API",
        description=DESCRIPTION,
        version=api_version(),
        lifespan=lifespan,
        docs_url="/docs",
        openapi_url="/openapi.json",
    )
    # The dashboard's Vite dev server is a different origin during `make dev`.
    app.add_middleware(
        CORSMiddleware,
        allow_origin_regex=r"http://(localhost|127\.0\.0\.1)(:\d+)?",
        allow_credentials=False,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
    )
    register_error_handlers(app)
    app.include_router(router)
    app.include_router(ws_router)
    return app


app = create_app()
