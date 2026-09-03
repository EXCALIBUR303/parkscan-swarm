"""MQTT ingest: broker -> validation -> PostGIS -> WebSocket fan-out.

Client configuration is deliberate, not boilerplate:

* **MQTT 5 with a stable client identifier and ``clean_start=False``**, plus a
  ``SessionExpiryInterval``. The broker therefore keeps this subscriber's session
  and its queued QoS-1 messages across a short disconnect, so a broker blip or an
  API restart does not silently lose robot telemetry.
* **Its own reconnect loop with exponential backoff.** aiomqtt surfaces a
  disconnect as ``MqttError`` out of the message iterator; a fresh
  :class:`aiomqtt.Client` is built for each attempt because the client object is
  not re-enterable.

The task is owned by the FastAPI lifespan. It never raises into the application:
a broker that is down leaves the API up and ``/health`` reporting ``mqtt: error``.
"""

from __future__ import annotations

import asyncio
import contextlib
import datetime as dt
import json
from dataclasses import dataclass, field
from typing import Any

import aiomqtt
import structlog
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties
from pydantic import ValidationError

from parkscan_api.config import Settings
from parkscan_api.db import ComponentHealth, Database
from parkscan_api.ingest import record_detection, record_telemetry
from parkscan_api.schemas import DetectionMessage, TelemetryMessage
from parkscan_api.ws import ConnectionManager
from parkscan_core.geo import Origin

TELEMETRY = "telemetry"
DETECTION = "detection"

log: structlog.stdlib.BoundLogger = structlog.get_logger("parkscan.mqtt")


def topic_root(run_id: str) -> str:
    """Versioned prefix. A schema change cannot be mistaken for live data."""
    return f"parkscan/v1/{run_id}"


def telemetry_filter(run_id: str) -> str:
    return f"{topic_root(run_id)}/robot/+/telemetry"


def detection_filter(run_id: str) -> str:
    return f"{topic_root(run_id)}/robot/+/detection"


def robot_topic(run_id: str, robot_id: str, suffix: str) -> str:
    return f"{topic_root(run_id)}/robot/{robot_id}/{suffix}"


@dataclass(frozen=True, slots=True)
class ParsedTopic:
    run_id: str
    robot_id: str
    suffix: str


def parse_topic(topic: str) -> ParsedTopic | None:
    """``parkscan/v1/{run}/robot/{id}/{suffix}`` -> parts, or ``None``."""
    parts = topic.split("/")
    if len(parts) != 6:
        return None
    prefix, version, run_id, robot_literal, robot_id, suffix = parts
    if prefix != "parkscan" or version != "v1" or robot_literal != "robot":
        return None
    if not run_id or not robot_id:
        return None
    return ParsedTopic(run_id=run_id, robot_id=robot_id, suffix=suffix)


@dataclass
class IngestStats:
    """Counters surfaced on ``/health``. Silent drops are not acceptable."""

    received: int = 0
    persisted: int = 0
    invalid: int = 0
    ignored: int = 0
    failed: int = 0
    connects: int = 0
    last_error: str | None = field(default=None)

    def as_dict(self) -> dict[str, Any]:
        return {
            "received": self.received,
            "persisted": self.persisted,
            "invalid": self.invalid,
            "ignored": self.ignored,
            "failed": self.failed,
            "connects": self.connects,
        }


class MqttIngest:
    """Background subscriber feeding the database and the WebSocket manager."""

    def __init__(
        self,
        settings: Settings,
        database: Database,
        manager: ConnectionManager,
        *,
        origin: Origin | None = None,
    ) -> None:
        self._settings = settings
        self._database = database
        self._manager = manager
        self._origin = origin
        self._task: asyncio.Task[None] | None = None
        self._connected = False
        self.stats = IngestStats()

    # ------------------------------------------------------------- lifecycle --
    def start(self) -> None:
        """Spawn the subscriber task. Returns immediately."""
        if self._task is not None:
            return
        self._task = asyncio.create_task(self._run(), name="mqtt-ingest")

    async def stop(self) -> None:
        """Cancel and await the subscriber task."""
        task, self._task = self._task, None
        if task is None:
            return
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def health(self) -> ComponentHealth:
        if self._task is None:
            return ComponentHealth("stopped", "ingest task not started")
        if self._connected:
            return ComponentHealth("ok")
        return ComponentHealth("error", self.stats.last_error or "not connected")

    # ---------------------------------------------------------------- client --
    def _build_client(self) -> aiomqtt.Client:
        # paho-mqtt ships no type stubs, so Properties() is untyped to mypy --strict.
        # Narrow it here at the single boundary rather than relaxing the whole module.
        properties: Any = Properties(PacketTypes.CONNECT)  # type: ignore[no-untyped-call]
        # Keep the broker-side session (and its queued QoS-1 messages) alive
        # across a disconnect, so telemetry survives a broker blip.
        properties.SessionExpiryInterval = self._settings.mqtt_session_expiry_s
        return aiomqtt.Client(
            hostname=self._settings.mqtt_host,
            port=self._settings.mqtt_port,
            identifier=self._settings.mqtt_client_id,
            protocol=aiomqtt.ProtocolVersion.V5,
            clean_start=False,
            properties=properties,
            keepalive=30,
        )

    async def _run(self) -> None:
        settings = self._settings
        delay = settings.mqtt_reconnect_min_s
        while True:
            try:
                async with self._build_client() as client:
                    await client.subscribe(telemetry_filter(settings.run_id), qos=settings.mqtt_qos)
                    await client.subscribe(detection_filter(settings.run_id), qos=settings.mqtt_qos)
                    self._connected = True
                    self.stats.connects += 1
                    self.stats.last_error = None
                    delay = settings.mqtt_reconnect_min_s
                    log.info(
                        "mqtt_connected",
                        host=settings.mqtt_host,
                        port=settings.mqtt_port,
                        run_id=settings.run_id,
                    )
                    async for message in client.messages:
                        await self.handle(str(message.topic), _payload_bytes(message.payload))
            except asyncio.CancelledError:
                self._connected = False
                raise
            except Exception as exc:
                self._connected = False
                self.stats.last_error = f"{type(exc).__name__}: {exc}".strip()[:300]
                log.warning("mqtt_disconnected", detail=self.stats.last_error, retry_in_s=delay)
            else:  # pragma: no cover - the iterator only ends on disconnect
                self._connected = False
            await asyncio.sleep(delay)
            delay = min(delay * 2.0, settings.mqtt_reconnect_max_s)

    # --------------------------------------------------------------- handling --
    async def handle(self, topic: str, payload: bytes) -> bool:
        """Validate, persist and fan out one message. Never raises.

        Returns ``True`` when the message reached the database.
        """
        self.stats.received += 1
        parsed = parse_topic(topic)
        if parsed is None or parsed.run_id != self._settings.run_id:
            self.stats.ignored += 1
            log.debug("mqtt_ignored", topic=topic)
            return False
        if parsed.suffix not in (TELEMETRY, DETECTION):
            self.stats.ignored += 1
            return False

        try:
            body = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.stats.invalid += 1
            log.warning("mqtt_payload_not_json", topic=topic, detail=str(exc)[:200])
            return False

        try:
            if parsed.suffix == TELEMETRY:
                message: TelemetryMessage | DetectionMessage = TelemetryMessage.model_validate(body)
            else:
                message = DetectionMessage.model_validate(body)
        except ValidationError as exc:
            self.stats.invalid += 1
            log.warning("mqtt_payload_invalid", topic=topic, errors=exc.error_count())
            return False

        try:
            envelope = await self._persist(parsed, message)
        except Exception as exc:
            self.stats.failed += 1
            self.stats.last_error = f"{type(exc).__name__}: {exc}".strip()[:300]
            log.error("mqtt_persist_failed", topic=topic, detail=self.stats.last_error)
            return False

        self.stats.persisted += 1
        self._manager.broadcast(envelope)
        return True

    async def _persist(
        self, parsed: ParsedTopic, message: TelemetryMessage | DetectionMessage
    ) -> dict[str, Any]:
        run_uuid = self._settings.run_uuid
        async with self._database.session() as session:
            if isinstance(message, TelemetryMessage):
                payload = await record_telemetry(
                    session, run_uuid, parsed.robot_id, message, origin=self._origin
                )
                kind = TELEMETRY
            else:
                payload = await record_detection(
                    session, run_uuid, parsed.robot_id, message, origin=self._origin
                )
                kind = DETECTION
        return {
            "type": kind,
            "stamp": dt.datetime.now(tz=dt.UTC).isoformat(),
            "data_source": message.data_source.value,
            "payload": payload,
        }


def _payload_bytes(payload: object) -> bytes:
    """paho hands back bytes, str, or a number depending on how it was published."""
    if isinstance(payload, bytes):
        return payload
    if isinstance(payload, bytearray):
        return bytes(payload)
    if isinstance(payload, str):
        return payload.encode("utf-8")
    return str(payload).encode("utf-8")
