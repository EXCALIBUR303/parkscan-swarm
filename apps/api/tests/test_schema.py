"""Schema invariants that the project's honesty and correctness rules depend on.

These assert against the LIVE database, not against the model classes, because the
thing that matters is what PostgreSQL will actually enforce at 3am — not what the
declarative layer intended.
"""

from __future__ import annotations

import pytest
import sqlalchemy as sa

from parkscan_api.db import Database

pytestmark = pytest.mark.db

PROVENANCE_COLUMNS = {
    ("run", "data_source"),
    ("robot", "data_source"),
    ("telemetry", "data_source"),
    ("detection", "data_source"),
    ("detection", "det_source"),
    ("parking_space", "data_source"),
}


async def test_migrations_are_at_the_expected_head(database: Database) -> None:
    async with database.session() as session:
        head = (
            await session.execute(sa.text("SELECT version_num FROM alembic_version"))
        ).scalar_one()
    assert head == "0001_initial", "run `make migrate`"


async def test_every_provenance_column_is_not_null(database: Database) -> None:
    """ACCEPTANCE H1: honesty is enforced by the schema, not by a disclaimer.

    If any of these were nullable, a row could be written with no declared origin and
    the dashboard would have to guess whether to show the SIMULATED badge.
    """
    async with database.session() as session:
        rows = (
            await session.execute(
                sa.text(
                    "SELECT table_name, column_name, is_nullable "
                    "FROM information_schema.columns "
                    "WHERE table_schema = 'public' "
                    "AND column_name IN ('data_source', 'det_source')"
                )
            )
        ).all()

    found = {(r.table_name, r.column_name) for r in rows}
    assert found >= PROVENANCE_COLUMNS, f"missing provenance columns: {PROVENANCE_COLUMNS - found}"

    nullable = [(r.table_name, r.column_name) for r in rows if r.is_nullable == "YES"]
    assert nullable == [], f"provenance columns must be NOT NULL, these are not: {nullable}"


async def test_spatial_columns_are_geography_not_geometry(database: Database) -> None:
    """ST_Distance on geometry(4326) returns DEGREES, on geography it returns METRES.

    Getting this wrong makes 'distance to the nearest charging bay' silently nonsense,
    so it is asserted rather than trusted.
    """
    async with database.session() as session:
        rows = (
            await session.execute(
                sa.text(
                    "SELECT table_name, column_name, udt_name "
                    "FROM information_schema.columns "
                    "WHERE table_schema = 'public' "
                    "AND column_name IN ('geog', 'polygon', 'entry_waypoint', 'exit_waypoint')"
                )
            )
        ).all()

    assert rows, "no spatial columns found — did the migration run?"
    wrong = [(r.table_name, r.column_name, r.udt_name) for r in rows if r.udt_name != "geography"]
    assert wrong == [], f"these must be geography, not geometry: {wrong}"


async def test_geography_distance_is_in_metres(database: Database) -> None:
    """The behavioural half of the test above."""
    async with database.session() as session:
        metres = (
            await session.execute(
                sa.text(
                    "SELECT ST_Distance("
                    "  'SRID=4326;POINT(0 0)'::geography,"
                    "  'SRID=4326;POINT(0.001 0)'::geography)"
                )
            )
        ).scalar_one()
    # ~111.3 m per 0.001 degree of longitude at the equator. If this came back as
    # 0.001, the column would be geometry and every distance query would be wrong.
    assert 111.0 < float(metres) < 111.5


async def test_spatial_indexes_exist(database: Database) -> None:
    async with database.session() as session:
        names = {
            r[0]
            for r in (
                await session.execute(
                    sa.text(
                        "SELECT indexname FROM pg_indexes "
                        "WHERE schemaname = 'public' AND indexdef ILIKE '%gist%'"
                    )
                )
            ).all()
        }
    assert {"idx_telemetry_geog", "idx_detection_geog", "idx_robot_geog"} <= names


async def test_fencing_token_columns_default_to_zero(database: Database) -> None:
    """zone.epoch and charging_bay.lease_token are the fencing tokens.

    They must be NOT NULL with a concrete default: a NULL token would compare
    false against everything and silently disable the fencing check.
    """
    async with database.session() as session:
        rows = (
            await session.execute(
                sa.text(
                    "SELECT table_name, column_name, is_nullable, column_default "
                    "FROM information_schema.columns "
                    "WHERE table_schema = 'public' "
                    "AND (table_name, column_name) IN "
                    "(('zone','epoch'), ('charging_bay','lease_token'))"
                )
            )
        ).all()

    assert len(rows) == 2, f"expected both fencing columns, got {rows}"
    for row in rows:
        assert row.is_nullable == "NO", f"{row.table_name}.{row.column_name} is nullable"
        assert row.column_default is not None
