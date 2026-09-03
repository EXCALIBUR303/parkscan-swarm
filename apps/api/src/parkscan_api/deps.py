"""Shared application state and FastAPI dependencies.

Everything the request handlers need hangs off one :class:`AppState` object
created by the lifespan and stored on ``app.state``. Nothing is a module global,
so two apps can coexist in one test process.
"""

from __future__ import annotations

import uuid
from collections.abc import AsyncIterator
from dataclasses import dataclass
from typing import Annotated, cast

from fastapi import Depends, Query
from sqlalchemy.ext.asyncio import AsyncSession
from starlette.requests import HTTPConnection

from parkscan_api.config import RUN_NAMESPACE, Settings
from parkscan_api.db import Database
from parkscan_api.mqtt import MqttIngest
from parkscan_api.ws import ConnectionManager
from parkscan_core.geo import Origin

STATE_ATTR = "parkscan"


@dataclass(slots=True)
class AppState:
    """Everything the lifespan built, in one place."""

    settings: Settings
    database: Database
    manager: ConnectionManager
    ingest: MqttIngest
    #: Lot origin used to project local metres to WGS84. ``None`` when the lot
    #: spec could not be read; ``geog`` is then written NULL rather than guessed.
    origin: Origin | None = None
    lot_error: str | None = None


def app_state(connection: HTTPConnection) -> AppState:
    """Fetch :class:`AppState`. Works for both HTTP requests and WebSockets."""
    return cast(AppState, getattr(connection.app.state, STATE_ATTR))


def get_settings_dep(connection: HTTPConnection) -> Settings:
    return app_state(connection).settings


def get_database(connection: HTTPConnection) -> Database:
    return app_state(connection).database


async def get_session(connection: HTTPConnection) -> AsyncIterator[AsyncSession]:
    """One request-scoped unit of work."""
    async with get_database(connection).session() as session:
        yield session


StateDep = Annotated[AppState, Depends(app_state)]
SettingsDep = Annotated[Settings, Depends(get_settings_dep)]
SessionDep = Annotated[AsyncSession, Depends(get_session)]

RunIdQuery = Annotated[
    str | None,
    Query(
        description="Run to scope the query to: either a UUID or the label used in "
        "the MQTT topics. Defaults to PARKSCAN_RUN_ID.",
    ),
]


def resolve_run_uuid(run_id: str | None, settings: Settings) -> uuid.UUID:
    """Map a run label or UUID string onto the ``run.id`` primary key."""
    if run_id is None:
        return settings.run_uuid
    try:
        return uuid.UUID(run_id)
    except ValueError:
        return uuid.uuid5(RUN_NAMESPACE, run_id)
