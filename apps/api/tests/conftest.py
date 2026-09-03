"""Shared fixtures.

Service-dependent tests are marked and SKIP cleanly when the service is unreachable,
so the whole suite still runs on a machine with Docker stopped. That is deliberate:
a test suite you cannot run on your own laptop stops being run.
"""

from __future__ import annotations

import asyncio
import contextlib
import socket
from collections.abc import AsyncIterator

import pytest
import sqlalchemy as sa
from asgi_lifespan import LifespanManager
from httpx import ASGITransport, AsyncClient

from parkscan_api.config import Settings, get_settings
from parkscan_api.db import Database
from parkscan_api.main import create_app


def _port_open(host: str, port: int, timeout: float = 0.4) -> bool:
    with contextlib.suppress(OSError), socket.create_connection((host, port), timeout=timeout):
        return True
    return False


@pytest.fixture(scope="session")
def settings() -> Settings:
    return get_settings()


@pytest.fixture(scope="session")
def db_available(settings: Settings) -> bool:
    url = settings.database_url
    host = url.split("@", 1)[-1].split("/", 1)[0]
    hostname, _, port = host.partition(":")
    return _port_open(hostname or "127.0.0.1", int(port or 5432))


@pytest.fixture(scope="session")
def mqtt_available(settings: Settings) -> bool:
    return _port_open(settings.mqtt_host, settings.mqtt_port)


@pytest.fixture(autouse=True)
def _skip_when_service_missing(
    request: pytest.FixtureRequest, db_available: bool, mqtt_available: bool
) -> None:
    """Turn the `db` / `mqtt` markers into clean skips rather than errors."""
    if request.node.get_closest_marker("db") and not db_available:
        pytest.skip("no PostgreSQL reachable — run `make up`")
    if request.node.get_closest_marker("mqtt") and not mqtt_available:
        pytest.skip("no MQTT broker reachable — run `make up`")


@pytest.fixture
async def client() -> AsyncIterator[AsyncClient]:
    """An HTTP client bound to the real app, with lifespan actually running.

    httpx's ASGITransport does NOT run lifespan events on its own, so without
    LifespanManager this would exercise an app whose database engine and MQTT client
    were never started — every dependency would be None and the tests would pass
    against a shell. LifespanManager is what makes these tests mean anything.
    """
    app = create_app()
    async with LifespanManager(app, startup_timeout=30, shutdown_timeout=30):
        transport = ASGITransport(app=app)
        async with AsyncClient(transport=transport, base_url="http://test") as ac:
            yield ac


@pytest.fixture
async def database(settings: Settings, db_available: bool) -> AsyncIterator[Database]:
    if not db_available:
        pytest.skip("no PostgreSQL reachable")
    db = Database(settings.database_url)
    db.start()
    try:
        yield db
    finally:
        await db.stop()


@pytest.fixture
def anyio_backend() -> str:
    return "asyncio"


__all__ = ["asyncio", "sa"]
