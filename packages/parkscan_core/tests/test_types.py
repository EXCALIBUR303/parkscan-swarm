"""Core type behaviour: the bottom-centre rule and the cheap validation guards."""

from __future__ import annotations

import dataclasses
import json

import numpy as np
import pytest

from parkscan_core.types import (
    BayState,
    BBox,
    CoordView,
    DataSource,
    Detection,
    DetKind,
    DetSource,
    Frame,
    Pose2D,
    RobotState,
    SensorFrame,
    Severity,
    TwistCmd,
    ZoneState,
)


def test_bbox_bottom_centre_is_the_ground_contact_point() -> None:
    box = BBox(x1=100.0, y1=40.0, x2=140.0, y2=90.0)
    assert box.bottom_centre == (120.0, 90.0)
    # It must be the bottom edge, never the centre: the centre would sit 25 px higher
    # here, which back-projects to a point metres further away.
    centre_y = (box.y1 + box.y2) / 2.0
    assert box.bottom_centre[1] != centre_y
    assert box.width == 40.0
    assert box.height == 50.0


def test_bbox_bottom_centre_on_a_degenerate_width_box() -> None:
    box = BBox(x1=7.0, y1=1.0, x2=7.0, y2=3.0)
    assert box.bottom_centre == (7.0, 3.0)


@pytest.mark.parametrize(
    ("x1", "y1", "x2", "y2"),
    [(10.0, 0.0, 5.0, 4.0), (0.0, 10.0, 4.0, 5.0), (10.0, 10.0, 5.0, 5.0)],
)
def test_bbox_rejects_inverted_corners(x1: float, y1: float, x2: float, y2: float) -> None:
    with pytest.raises(ValueError, match="degenerate bbox"):
        BBox(x1=x1, y1=y1, x2=x2, y2=y2)


def test_bbox_is_frozen() -> None:
    box = BBox(0.0, 0.0, 1.0, 1.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        box.x1 = 5.0  # type: ignore[misc]


@pytest.mark.parametrize("confidence", [-0.001, 1.001, 2.0, -1.0])
def test_detection_rejects_out_of_range_confidence(confidence: float) -> None:
    with pytest.raises(ValueError, match=r"confidence must be in \[0, 1\]"):
        Detection(
            kind=DetKind.POTHOLE,
            det_source=DetSource.SIMULATED,
            detector_name="SimulatedDetector",
            confidence=confidence,
            severity=Severity.HIGH,
            bbox=BBox(0.0, 0.0, 10.0, 10.0),
            stamp_ns=1,
        )


def test_detection_rejects_empty_detector_name() -> None:
    with pytest.raises(ValueError, match="detector_name"):
        Detection(
            kind=DetKind.POTHOLE,
            det_source=DetSource.SIMULATED,
            detector_name="",
            confidence=0.5,
            severity=None,
            bbox=BBox(0.0, 0.0, 10.0, 10.0),
            stamp_ns=1,
        )


def test_detection_accepts_the_boundary_values() -> None:
    for confidence in (0.0, 1.0):
        det = Detection(
            kind=DetKind.SPACE_OCCUPANCY,
            det_source=DetSource.GROUND_TRUTH,
            detector_name="ground-truth",
            confidence=confidence,
            severity=None,
            bbox=BBox(0.0, 0.0, 1.0, 1.0),
            stamp_ns=7,
        )
        assert det.confidence == confidence
        assert det.severity is None


def test_detection_has_no_default_source() -> None:
    """det_source must be supplied explicitly -- honesty cannot be defaulted away."""
    fields = {f.name: f for f in dataclasses.fields(Detection)}
    assert fields["det_source"].default is dataclasses.MISSING
    assert fields["det_source"].default_factory is dataclasses.MISSING


@pytest.mark.parametrize("soc", [-0.0001, 1.0001, 1.5, -3.0])
def test_sensor_frame_rejects_out_of_range_battery(soc: float) -> None:
    with pytest.raises(ValueError, match=r"battery_soc must be in \[0, 1\]"):
        SensorFrame(
            stamp_ns=0,
            pose=Pose2D(0.0, 0.0, 0.0),
            twist=TwistCmd(0.0, 0.0),
            battery_soc=soc,
            ranges=(1.0, 2.0),
            camera=None,
        )


def test_sensor_frame_holds_an_immutable_range_scan() -> None:
    frame = SensorFrame(
        stamp_ns=1_000_000,
        pose=Pose2D(1.0, 2.0, 0.5),
        twist=TwistCmd(0.4, -0.1),
        battery_soc=0.83,
        ranges=(1.0, float("inf"), 3.5),
        camera=None,
    )
    assert isinstance(frame.ranges, tuple)
    assert frame.ranges[1] == float("inf")
    with pytest.raises(dataclasses.FrozenInstanceError):
        frame.battery_soc = 0.1  # type: ignore[misc]


def test_frame_accepts_a_matching_image_and_rejects_a_mismatched_one() -> None:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    frame = Frame(frame_id=3, stamp_ns=9, camera_id="front", width=160, height=120, image=image)
    assert frame.image is not None
    assert frame.image.shape == (120, 160, 3)

    with pytest.raises(ValueError, match="does not match declared"):
        Frame(frame_id=4, stamp_ns=9, camera_id="front", width=320, height=240, image=image)


def test_frame_allows_a_geometry_only_tick() -> None:
    frame = Frame(frame_id=1, stamp_ns=1, camera_id="front", width=160, height=120, image=None)
    assert frame.image is None


@pytest.mark.parametrize(("w", "h"), [(0, 120), (160, 0), (-1, 120)])
def test_frame_rejects_non_positive_dimensions(w: int, h: int) -> None:
    with pytest.raises(ValueError, match="frame dimensions must be positive"):
        Frame(frame_id=1, stamp_ns=1, camera_id="front", width=w, height=h, image=None)


def test_enum_values_are_the_exact_database_spellings() -> None:
    """These strings are the PostgreSQL enum labels in docs/DATA_MODEL.md."""
    assert [e.value for e in DataSource] == ["simulated", "replay", "live"]
    assert [e.value for e in DetSource] == ["simulated", "model", "ground_truth"]
    assert [e.value for e in DetKind] == ["pothole", "space_occupancy"]
    assert [e.value for e in Severity] == ["low", "medium", "high"]
    assert [e.value for e in RobotState] == [
        "idle",
        "assigned",
        "transit",
        "inspecting",
        "waiting_on_lease",
        "returning",
        "charging",
        "failed",
        "offline",
    ]
    assert [e.value for e in CoordView] == ["online", "suspect", "offline", "stuck"]
    assert [e.value for e in ZoneState] == [
        "unassigned",
        "assigned",
        "in_progress",
        "complete",
        "unrecoverable",
    ]
    assert [e.value for e in BayState] == ["vacant", "occupied", "unknown"]


def test_str_enums_serialise_directly() -> None:
    assert f"{RobotState.WAITING_ON_LEASE}" == "waiting_on_lease"
    assert json.dumps({"det_source": DetSource.SIMULATED}) == '{"det_source": "simulated"}'
    assert RobotState("transit") is RobotState.TRANSIT
