"""Async engine and session factory.

There is no module-level session and no module-level engine: a :class:`Database`
instance is created by the application lifespan and handed to whoever needs it,
so tests can stand up an isolated one and the process can be torn down cleanly.

``start()`` deliberately does not connect. ``create_async_engine`` only builds a
lazy pool, which is what lets the API come up and report ``db: error`` on
``/health`` while PostgreSQL is down, instead of refusing to boot.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Literal

import sqlalchemy as sa
from sqlalchemy.ext.asyncio import (
    AsyncEngine,
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)

#: Mirrors ``schemas.ComponentStatus.status`` so the health response needs no cast.
HealthStatus = Literal["ok", "error", "stopped", "connecting"]


@dataclass(frozen=True, slots=True)
class ComponentHealth:
    """Liveness of one dependency. ``detail`` is populated only on failure."""

    status: HealthStatus
    detail: str | None = None

    @property
    def ok(self) -> bool:
        return self.status == "ok"


class DatabaseNotStartedError(RuntimeError):
    """Raised when the engine is used before ``start()`` or after ``stop()``."""

    def __init__(self) -> None:
        super().__init__("Database.start() has not been called")


class Database:
    """Owns one async engine and its session factory."""

    def __init__(self, url: str, *, echo: bool = False, pool_size: int = 5) -> None:
        self._url = url
        self._echo = echo
        self._pool_size = pool_size
        self._engine: AsyncEngine | None = None
        self._sessionmaker: async_sessionmaker[AsyncSession] | None = None

    @property
    def url(self) -> str:
        return self._url

    def start(self) -> None:
        """Build the engine. Does not open a connection."""
        if self._engine is not None:
            return
        self._engine = create_async_engine(
            self._url,
            echo=self._echo,
            pool_size=self._pool_size,
            max_overflow=5,
            pool_pre_ping=True,
        )
        self._sessionmaker = async_sessionmaker(
            self._engine, expire_on_commit=False, autoflush=False
        )

    async def stop(self) -> None:
        """Dispose the pool."""
        engine, self._engine = self._engine, None
        self._sessionmaker = None
        if engine is not None:
            await engine.dispose()

    @property
    def engine(self) -> AsyncEngine:
        if self._engine is None:
            raise DatabaseNotStartedError
        return self._engine

    @property
    def sessionmaker(self) -> async_sessionmaker[AsyncSession]:
        if self._sessionmaker is None:
            raise DatabaseNotStartedError
        return self._sessionmaker

    @asynccontextmanager
    async def session(self) -> AsyncIterator[AsyncSession]:
        """One unit of work. Commits on success, rolls back on any exception."""
        async with self.sessionmaker() as session:
            try:
                yield session
            except BaseException:
                await session.rollback()
                raise
            else:
                await session.commit()

    async def ping(self) -> None:
        """``SELECT 1``. Raises whatever the driver raises."""
        async with self.engine.connect() as conn:
            await conn.execute(sa.text("SELECT 1"))

    async def health(self) -> ComponentHealth:
        """Never raises — ``/health`` must answer while the database is down."""
        try:
            await self.ping()
        except DatabaseNotStartedError:
            return ComponentHealth("stopped", "engine not started")
        except Exception as exc:  # reporting liveness, not handling the failure
            return ComponentHealth("error", f"{type(exc).__name__}: {exc}".strip()[:300])
        return ComponentHealth("ok")


def sync_url(url: str) -> str:
    """``postgresql+asyncpg://...`` -> plain ``postgresql://...`` for asyncpg/psql."""
    return url.replace("+asyncpg", "", 1)
