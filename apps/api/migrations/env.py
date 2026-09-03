"""Alembic environment — async, over the same asyncpg driver the application uses.

Three things here are not boilerplate:

1. The database URL comes from :class:`parkscan_api.config.Settings`, not from
   ``alembic.ini``. One source of truth for the connection string; the ini file's
   ``sqlalchemy.url`` is intentionally blank.

2. Migrations run through **asyncpg**, not a second synchronous driver. The usual
   Alembic recipe strips ``+asyncpg`` and reaches for psycopg2, which would mean
   shipping two PostgreSQL drivers and two sets of connection semantics for no gain.
   ``async_engine_from_config`` + ``connection.run_sync`` keeps it to one.

3. GeoAlchemy2's alembic helpers are wired in. Without them, autogenerate emits
   ``geoalchemy2.types.Geography`` without the import, tries to manage the spatial
   indexes GeoAlchemy2 already creates for itself, and picks up PostGIS's own
   ``spatial_ref_sys`` / ``topology`` tables as if they were ours.
"""

from __future__ import annotations

import asyncio
from logging.config import fileConfig
from typing import Any

from alembic import context
from geoalchemy2 import alembic_helpers
from sqlalchemy import Connection, pool
from sqlalchemy.ext.asyncio import async_engine_from_config

from parkscan_api.config import get_settings
from parkscan_api.models import Base

config = context.config
if config.config_file_name is not None:
    fileConfig(config.config_file_name)

config.set_main_option("sqlalchemy.url", get_settings().database_url)

target_metadata = Base.metadata

# PostGIS installs these into the same database. They are not ours to migrate.
POSTGIS_OWNED = frozenset(
    {
        "spatial_ref_sys",
        "geography_columns",
        "geometry_columns",
        "raster_columns",
        "raster_overviews",
        "topology",
        "layer",
    }
)


def include_object(
    obj: Any, name: str | None, type_: str, reflected: bool, compare_to: Any
) -> bool:
    """Skip PostGIS's own objects, then defer to GeoAlchemy2."""
    if type_ == "table" and name in POSTGIS_OWNED:
        return False
    return bool(alembic_helpers.include_object(obj, name, type_, reflected, compare_to))


def _configure(connection: Connection) -> None:
    context.configure(
        connection=connection,
        target_metadata=target_metadata,
        include_object=include_object,
        render_item=alembic_helpers.render_item,
        process_revision_directives=alembic_helpers.writer,
        compare_type=True,
    )


def do_run_migrations(connection: Connection) -> None:
    _configure(connection)
    with context.begin_transaction():
        context.run_migrations()


def run_migrations_offline() -> None:
    """Emit SQL to stdout without a database (``alembic upgrade head --sql``)."""
    context.configure(
        url=config.get_main_option("sqlalchemy.url"),
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "named"},
        include_object=include_object,
        render_item=alembic_helpers.render_item,
        compare_type=True,
    )
    with context.begin_transaction():
        context.run_migrations()


async def run_migrations_online() -> None:
    section: dict[str, Any] = config.get_section(config.config_ini_section) or {}
    connectable = async_engine_from_config(section, prefix="sqlalchemy.", poolclass=pool.NullPool)
    async with connectable.connect() as connection:
        await connection.run_sync(do_run_migrations)
    await connectable.dispose()


if context.is_offline_mode():
    run_migrations_offline()
else:
    asyncio.run(run_migrations_online())
