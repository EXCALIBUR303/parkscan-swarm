"""Runtime configuration and build identity.

Values come from the process environment first, then the repository ``.env`` (the
same file ``make dev`` sources), then the defaults below — which are kept in step
with ``.env.example``.

``PARKSCAN_RUN_ID`` is a human label (``dev``), while ``run.id`` in the database is
a UUID. :attr:`Settings.run_uuid` bridges the two deterministically: a label that
already parses as a UUID is used as-is, anything else is hashed with UUIDv5. The
mapping is therefore stable across restarts without inventing a second column.
"""

from __future__ import annotations

import uuid
from functools import lru_cache
from pathlib import Path

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from parkscan_core.types import DataSource

#: Namespace for deriving ``run.id`` from a non-UUID ``PARKSCAN_RUN_ID`` label.
RUN_NAMESPACE = uuid.UUID("6f1d5f1e-6a3f-5c9a-9d1a-9f5c7c1b0a01")


def _find_repo_root(start: Path) -> Path:
    """Walk upwards for the workspace root (the one holding ``docs/``)."""
    for candidate in (start, *start.parents):
        if (candidate / "pyproject.toml").is_file() and (candidate / "docs").is_dir():
            return candidate
    return start


REPO_ROOT = _find_repo_root(Path(__file__).resolve().parent)


class Settings(BaseSettings):
    """Process configuration. Immutable once constructed."""

    model_config = SettingsConfigDict(
        env_file=REPO_ROOT / ".env",
        env_file_encoding="utf-8",
        extra="ignore",
        case_sensitive=False,
        frozen=True,
    )

    database_url: str = Field(
        default="postgresql+asyncpg://parkscan:parkscan@127.0.0.1:5433/parkscan",
        validation_alias="DATABASE_URL",
    )
    mqtt_host: str = Field(default="127.0.0.1", validation_alias="MQTT_HOST")
    mqtt_port: int = Field(default=1883, validation_alias="MQTT_PORT")
    api_host: str = Field(default="127.0.0.1", validation_alias="API_HOST")
    api_port: int = Field(default=8000, validation_alias="API_PORT")
    log_level: str = Field(default="INFO", validation_alias="LOG_LEVEL")

    run_id: str = Field(default="dev", validation_alias="PARKSCAN_RUN_ID")
    data_source: DataSource = Field(
        default=DataSource.SIMULATED, validation_alias="PARKSCAN_DATA_SOURCE"
    )

    #: Authored lot spec. Its origin turns lot metres into WGS84 for the ``geog``
    #: columns. When it cannot be read, ``geog`` is written NULL rather than
    #: guessed — a wrong origin would silently misplace every row on the map.
    lot_path: Path = Field(
        default=REPO_ROOT / "sim" / "lot" / "lot.yaml", validation_alias="PARKSCAN_LOT_PATH"
    )

    #: Per-browser WebSocket queue bound. Overflow drops the oldest frame and
    #: emits a ``lag`` notice (docs/API_CONTRACT.md).
    ws_queue_maxsize: int = Field(default=100, ge=1, validation_alias="PARKSCAN_WS_QUEUE_MAXSIZE")
    #: One stalled browser tab must not stall the fan-out to the others.
    ws_send_timeout_s: float = Field(
        default=5.0, gt=0.0, validation_alias="PARKSCAN_WS_SEND_TIMEOUT_S"
    )

    #: MQTT 5 session expiry, seconds. Long enough that a broker blip does not
    #: lose queued QoS-1 robot telemetry.
    mqtt_session_expiry_s: int = Field(
        default=300, ge=0, validation_alias="PARKSCAN_MQTT_SESSION_EXPIRY_S"
    )
    mqtt_reconnect_min_s: float = Field(
        default=0.5, gt=0.0, validation_alias="PARKSCAN_MQTT_RECONNECT_MIN_S"
    )
    mqtt_reconnect_max_s: float = Field(
        default=30.0, gt=0.0, validation_alias="PARKSCAN_MQTT_RECONNECT_MAX_S"
    )
    mqtt_qos: int = Field(default=1, ge=0, le=2, validation_alias="PARKSCAN_MQTT_QOS")

    @property
    def run_uuid(self) -> uuid.UUID:
        """The database key for :attr:`run_id`."""
        try:
            return uuid.UUID(self.run_id)
        except ValueError:
            return uuid.uuid5(RUN_NAMESPACE, self.run_id)

    @property
    def mqtt_client_id(self) -> str:
        """Stable identifier — required for MQTT 5 session resumption to work."""
        return f"parkscan-api-{self.run_id}"


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    """Process-wide settings, read once."""
    return Settings()


def api_version() -> str:
    """Installed ``parkscan-api`` version, or ``0+unknown`` outside an install."""
    from importlib.metadata import PackageNotFoundError, version

    try:
        return version("parkscan-api")
    except PackageNotFoundError:  # pragma: no cover - only when run from a source tree
        return "0+unknown"


def git_sha(repo_root: Path | None = None) -> str:
    """Short git SHA of the checkout, or ``unknown``.

    Read straight off ``.git`` rather than shelling out: no subprocess in a
    request path, and it degrades to ``unknown`` on a fresh repository that has
    no commits yet instead of raising.
    """
    root = repo_root if repo_root is not None else REPO_ROOT
    head = root / ".git" / "HEAD"
    try:
        raw = head.read_text(encoding="utf-8").strip()
    except OSError:
        return "unknown"
    if not raw.startswith("ref: "):
        return raw[:12] or "unknown"
    ref = raw.removeprefix("ref: ").strip()
    ref_file = root / ".git" / ref
    try:
        return ref_file.read_text(encoding="utf-8").strip()[:12]
    except OSError:
        pass
    packed = root / ".git" / "packed-refs"
    try:
        lines = packed.read_text(encoding="utf-8").splitlines()
    except OSError:
        return "unknown"
    for line in lines:
        if line.endswith(f" {ref}"):
            return line.split(" ", 1)[0][:12]
    return "unknown"
