"""Turning an agent's :class:`TelemetrySample` into an MQTT payload.

This module is where provenance is stamped. The agent deliberately does not carry a
``data_source`` field -- it cannot honestly know whether it is driving a simulator or
hardware -- so the runner, which *constructed* the backend, is the only component
entitled to make that claim (ACCEPTANCE H1).
"""

from __future__ import annotations

import datetime as dt
import math
from typing import Any

from parkscan_agent.robot import TelemetrySample
from parkscan_perception.geolocation import GeolocResult

from parkscan_core.types import DataSource, Detection

_NS_PER_S = 1_000_000_000

#: Stand-in for an infinite range, so the value fits a SQL ``real``. Only reached on an
#: above-horizon rejection, where the reason field carries the real explanation.
_UNBOUNDED_RANGE_M = 1.0e6


def sim_stamp(stamp_ns: int, epoch: dt.datetime) -> str:
    """Map the SIM clock onto a wall-clock instant for storage.

    The simulator's clock starts at zero and advances by exact ticks. Storing that raw
    would make every run overlap in the database, so it is offset from a single run
    epoch captured once at startup. The offset is applied here and nowhere else, so the
    sim clock stays the source of truth for ordering.
    """
    return (epoch + dt.timedelta(microseconds=stamp_ns / 1000)).isoformat()


def telemetry_payload(
    sample: TelemetrySample, *, data_source: DataSource, epoch: dt.datetime
) -> dict[str, Any]:
    """Serialise one tick to the wire shape in docs/API_CONTRACT.md."""
    return {
        "pose": {
            "x_m": sample.pose.x_m,
            "y_m": sample.pose.y_m,
            "theta_rad": sample.pose.theta_rad,
        },
        "twist": {
            "v_mps": sample.twist.v_mps,
            "omega_radps": sample.twist.omega_radps,
        },
        "battery_soc": sample.battery_soc,
        "state": str(sample.state),
        "data_source": str(data_source),
        "zone_id": sample.zone_id,
        "epoch": sample.epoch,
        "stamp": sim_stamp(sample.stamp_ns, epoch),
    }


def progress_payload(sample: TelemetrySample, *, epoch: dt.datetime) -> dict[str, Any] | None:
    """Coverage progress, or ``None`` when this tick covered nothing new.

    Reported per CELL rather than per zone. The granularity is the point: Phase 4
    requeues a failed robot's *remaining* cells, which is only possible if progress was
    ever reported more finely than the assignment.
    """
    if not sample.covered_cells:
        return None
    return {
        "zone_id": sample.zone_id,
        "epoch": sample.epoch,
        "cells_done": list(sample.covered_cells),
        "stamp": sim_stamp(sample.stamp_ns, epoch),
    }


def detection_payload(
    detection: Detection,
    result: GeolocResult,
    *,
    data_source: DataSource,
    stamp_ns: int,
    epoch: dt.datetime,
) -> dict[str, Any]:
    """Serialise one detection to the wire shape in docs/API_CONTRACT.md.

    ``local_x``/``local_y`` are present ONLY when the geolocation is valid. The API
    enforces the same rule (a valid result must carry a point, an invalid one must not),
    because a rejected ray has no world position and storing a plausible-looking one is
    exactly the failure C11 exists to prevent.

    ``range_m`` is finite even on a range-gate rejection — a rejection at 6.5 m is a
    different fact from one at 40 m — but ``above_horizon`` yields infinity, which is not
    representable in a SQL ``real``, so it is clamped to the gate distance and the reason
    carries the detail.
    """
    valid = result.valid
    range_m = result.range_m if math.isfinite(result.range_m) else _UNBOUNDED_RANGE_M
    payload: dict[str, Any] = {
        "kind": str(detection.kind),
        "det_source": str(detection.det_source),
        "data_source": str(data_source),
        "detector_name": detection.detector_name,
        "confidence": detection.confidence,
        "bbox": {
            "x1": detection.bbox.x1,
            "y1": detection.bbox.y1,
            "x2": detection.bbox.x2,
            "y2": detection.bbox.y2,
        },
        "range_m": range_m,
        "geoloc_valid": valid,
        "severity": str(detection.severity) if detection.severity is not None else None,
        "stamp": sim_stamp(stamp_ns, epoch),
    }
    if valid and result.point is not None:
        payload["local_x"] = result.point[0]
        payload["local_y"] = result.point[1]
    return payload
