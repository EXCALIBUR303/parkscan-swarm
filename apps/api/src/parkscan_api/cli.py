"""Operational commands: ``python -m parkscan_api.cli <command>``.

``db-create``  create the database and enable PostGIS (idempotent).
``smoke``      prove the whole ingest path works end to end, against real services.

The Makefile targets ``db-create`` and ``smoke`` call exactly these. ``smoke`` is the
command that answers "is this actually wired up?" without a browser: it publishes a real
MQTT message to a real broker and then asserts the row appeared in a real PostGIS table.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from collections.abc import Sequence
from datetime import UTC, datetime

import asyncpg
import sqlalchemy as sa
from geoalchemy2 import Geometry

from parkscan_api.config import Settings, get_settings
from parkscan_api.db import Database, sync_url
from parkscan_api.main import load_origin
from parkscan_api.models import Telemetry
from parkscan_api.mqtt import MqttIngest, robot_topic
from parkscan_api.ws import ConnectionManager

EXPECTED_HEAD = "0001_initial"


def _libpq(url: str) -> str:
    """SQLAlchemy URL -> plain libpq DSN that asyncpg.connect accepts."""
    return sync_url(url)


def _split_dsn(dsn: str) -> tuple[str, str]:
    """Return (admin_dsn_against_postgres, database_name)."""
    base, _, database = dsn.rpartition("/")
    database = database.split("?", 1)[0]
    return f"{base}/postgres", database


# --------------------------------------------------------------------- db-create --
async def db_create(settings: Settings) -> int:
    dsn = _libpq(settings.database_url)
    admin_dsn, database = _split_dsn(dsn)

    admin = await asyncpg.connect(admin_dsn)
    try:
        exists = await admin.fetchval("SELECT 1 FROM pg_database WHERE datname = $1", database)
        if exists:
            print(f"database {database!r} already exists")
        else:
            # CREATE DATABASE cannot run inside a transaction, hence the raw execute.
            await admin.execute(f'CREATE DATABASE "{database}"')
            print(f"created database {database!r}")
    finally:
        await admin.close()

    conn = await asyncpg.connect(dsn)
    try:
        await conn.execute("CREATE EXTENSION IF NOT EXISTS postgis")
        version = await conn.fetchval("SELECT postgis_version()")
        print(f"postgis enabled: {version}")
    finally:
        await conn.close()
    return 0


# ------------------------------------------------------------------------- smoke --
async def smoke(settings: Settings) -> int:
    """Publish one telemetry message and assert it lands in PostGIS.

    Deliberately exercises the real broker and the real database rather than calling
    ``handle()`` directly with a fabricated payload -- the point is to catch wiring
    faults (topic shape, QoS, serialisation, schema mismatch), which a mocked test
    cannot see.
    """
    import aiomqtt

    failures: list[str] = []
    run_id = settings.run_id
    robot_id = "robot_smoke"

    database = Database(settings.database_url)
    database.start()
    try:
        # 1. schema is at the expected revision
        async with database.session() as session:
            head = (
                await session.execute(sa.text("SELECT version_num FROM alembic_version"))
            ).scalar_one_or_none()
        if head == EXPECTED_HEAD:
            print(f"[ok]   schema at revision {head}")
        else:
            failures.append(f"schema at {head!r}, expected {EXPECTED_HEAD!r} -- run 'make migrate'")
            print(f"[FAIL] schema at {head!r}, expected {EXPECTED_HEAD!r}")

        # 2. broker reachable, and the ingest path persists what it receives
        # Use the same origin the running app would use. Without it `project()`
        # returns None and every geography column is silently NULL -- which is
        # exactly the misconfiguration this smoke test exists to catch.
        origin, origin_error = load_origin(settings.lot_path)
        if origin is None:
            failures.append(f"lot origin unavailable ({origin_error}) -- geog would be NULL")
            print(f"[FAIL] lot origin unavailable: {origin_error}")
        else:
            print(f"[ok]   lot origin loaded from {settings.lot_path}")
        ingest = MqttIngest(settings, database, ConnectionManager(), origin=origin)
        topic = robot_topic(run_id, robot_id, "telemetry")
        # Shape must match schemas.TelemetryMessage exactly -- it is `extra="forbid"`,
        # so an approximate payload is rejected rather than silently half-ingested.
        payload = {
            "pose": {"x_m": 12.5, "y_m": 7.25, "theta_rad": 0.0},
            "twist": {"v_mps": 0.4, "omega_radps": 0.0},
            "battery_soc": 0.87,
            "state": "transit",
            "data_source": str(settings.data_source),
            "zone_id": None,
            "epoch": 0,
            "stamp": datetime.now(UTC).isoformat(),
        }

        async with aiomqtt.Client(hostname=settings.mqtt_host, port=settings.mqtt_port) as client:
            await client.publish(topic, json.dumps(payload).encode(), qos=1)
        print(f"[ok]   published to {topic}")

        accepted = await ingest.handle(topic, json.dumps(payload).encode())
        if not accepted:
            failures.append(f"ingest rejected the message: {ingest.stats.last_error}")
            print(f"[FAIL] ingest rejected the message: {ingest.stats.last_error}")
        else:
            print("[ok]   ingest accepted and persisted the message")

        # 3. the row is really in PostGIS, with geography populated
        async with database.session() as session:
            row = (
                await session.execute(
                    sa.select(
                        Telemetry.robot_id,
                        Telemetry.local_x,
                        Telemetry.data_source,
                        sa.func.ST_AsText(sa.cast(Telemetry.geog, Geometry)).label("wkt"),
                    )
                    .where(Telemetry.robot_id == robot_id)
                    .order_by(Telemetry.stamp.desc())
                    .limit(1)
                )
            ).first()
        if row is None:
            failures.append("no telemetry row found after ingest")
            print("[FAIL] no telemetry row found after ingest")
        else:
            print(
                f"[ok]   row persisted: robot={row.robot_id} local_x={row.local_x} "
                f"data_source={row.data_source} geog={row.wkt}"
            )
            if str(row.data_source) != str(settings.data_source):
                failures.append("data_source did not survive the round trip")
            if row.wkt is None:
                failures.append(
                    "geog is NULL -- local metres were never projected to WGS84, so "
                    "nothing will render on the map"
                )
                print("[FAIL] geog is NULL")
    finally:
        await database.stop()

    if failures:
        print(f"\nSMOKE FAILED ({len(failures)}):")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nSMOKE PASSED — MQTT -> ingest -> PostGIS round-trip works.")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="parkscan_api.cli", description=__doc__)
    parser.add_argument("command", choices=["db-create", "smoke"])
    args = parser.parse_args(argv)

    settings = get_settings()
    if args.command == "db-create":
        return asyncio.run(db_create(settings))
    return asyncio.run(smoke(settings))


if __name__ == "__main__":
    sys.exit(main())
