"""The synthetic scene must be lot-derived, seed-free in its geometry, and honest."""

from __future__ import annotations

import math

import pytest
from parkscan_eval.camera import CameraModel, project_ground_point
from parkscan_eval.scene import (
    SceneConfig,
    aisle_segment,
    build_targets,
    drive_and_record,
)
from parkscan_sim.config import SimConfig
from parkscan_sim.world import SimLite

from parkscan_core.lot import LotSpec

CAMERA = CameraModel(
    width=640,
    height=480,
    fx=457.0073621574767,
    fy=457.0073621574767,
    cx=320.0,
    cy=240.0,
    height_m=0.45,
    pitch_rad=-0.2617993877991494,
)


def _sim(lot: LotSpec, seed: int = 42) -> SimLite:
    sim = SimLite(SimConfig())
    sim.reset(seed, lot)
    return sim


def test_aisle_segment_comes_from_the_zone_waypoints(lot: LotSpec) -> None:
    entry, exit_ = aisle_segment(lot, "zone_01")
    assert entry == (5.0, 5.0)
    assert exit_ == (45.0, 5.0)


def test_unknown_zone_is_rejected_by_name(lot: LotSpec) -> None:
    with pytest.raises(ValueError, match="has no zone 'zone_99'"):
        aisle_segment(lot, "zone_99")


def test_targets_lie_inside_the_lot_and_inside_the_aisle(lot: LotSpec) -> None:
    config = SceneConfig()
    targets = build_targets(lot, config)
    assert len(targets) > 10
    for target in targets:
        x, y = target.position
        assert 0.0 <= x <= lot.size_m.w
        assert 0.0 <= y <= lot.size_m.h
        # zone_01's aisle spans y in [2, 8]; every target must sit on drivable surface.
        assert 2.0 < y < 8.0


def test_target_geometry_does_not_depend_on_a_seed(lot: LotSpec) -> None:
    """The seed controls what the detector and the error budget do, never where things are."""
    assert build_targets(lot, SceneConfig()) == build_targets(lot, SceneConfig())


def test_target_ids_are_unique_and_severities_cycle(lot: LotSpec) -> None:
    targets = build_targets(lot, SceneConfig())
    assert len({t.id for t in targets}) == len(targets)
    assert len({t.severity for t in targets}) == 3


def test_a_too_short_aisle_is_an_error_not_an_empty_scene(lot: LotSpec) -> None:
    with pytest.raises(ValueError, match="too short"):
        build_targets(lot, SceneConfig(lead_in_m=30.0, tail_m=30.0))


def test_the_drive_produces_frames_that_carry_no_fabricated_pixels(lot: LotSpec) -> None:
    frames = drive_and_record(lot, _sim(lot), CAMERA, SceneConfig())
    assert len(frames) > 50
    for recorded in frames:
        assert recorded.frame.image is None
        assert recorded.frame.width == CAMERA.width
        assert recorded.frame.height == CAMERA.height


def test_frame_ids_are_dense_and_stamps_are_monotonic(lot: LotSpec) -> None:
    frames = drive_and_record(lot, _sim(lot), CAMERA, SceneConfig())
    assert [f.frame.frame_id for f in frames] == list(range(len(frames)))
    stamps = [f.frame.stamp_ns for f in frames]
    assert stamps == sorted(stamps)
    assert len(set(stamps)) == len(stamps)


def test_the_robot_actually_drives_down_the_aisle(lot: LotSpec) -> None:
    frames = drive_and_record(lot, _sim(lot), CAMERA, SceneConfig())
    entry, exit_ = aisle_segment(lot, "zone_01")
    assert frames[0].pose.x_m == pytest.approx(entry[0])
    assert frames[-1].pose.x_m > entry[0] + 30.0
    assert frames[-1].pose.x_m <= exit_[0]
    xs = [f.pose.x_m for f in frames]
    assert xs == sorted(xs)


def test_every_sighting_bottom_centre_is_the_projection_of_its_known_target(
    lot: LotSpec,
) -> None:
    """The pixel the detector is handed must be the pixel geolocation is meant to invert."""
    frames = drive_and_record(lot, _sim(lot), CAMERA, SceneConfig())
    checked = 0
    for recorded in frames:
        for sighting in recorded.sightings:
            projection = project_ground_point(sighting.target.position, recorded.pose, CAMERA)
            u, v = sighting.bbox.bottom_centre
            assert u == pytest.approx(projection.u)
            assert v == pytest.approx(projection.v)
            checked += 1
    assert checked > 500


def test_every_sighting_box_is_inside_the_frame_with_room_for_jitter(lot: LotSpec) -> None:
    config = SceneConfig()
    frames = drive_and_record(lot, _sim(lot), CAMERA, config)
    for recorded in frames:
        for sighting in recorded.sightings:
            box = sighting.bbox
            assert box.x1 >= config.frame_margin_px
            assert box.x2 <= CAMERA.width - config.frame_margin_px
            assert box.y1 >= config.frame_margin_px
            assert box.y2 <= CAMERA.height - config.frame_margin_px
            assert box.width >= config.min_box_px


def test_true_range_matches_the_pose_and_the_known_position(lot: LotSpec) -> None:
    frames = drive_and_record(lot, _sim(lot), CAMERA, SceneConfig())
    for recorded in frames[:20]:
        for sighting in recorded.sightings:
            expected = math.dist((recorded.pose.x_m, recorded.pose.y_m), sighting.target.position)
            assert sighting.true_range_m == pytest.approx(expected)


def test_a_target_is_seen_over_a_wide_span_of_ranges(lot: LotSpec) -> None:
    """The whole experiment depends on each target being observed near AND far.

    ``pothole_11`` sits 28 m down a 40 m aisle on the robot's own line, so it is picked up
    at the far limit set by ``min_box_px`` and stays visible until it drops out of the
    bottom of the frame. Targets close to the aisle entry can only ever be seen from at most
    their own distance from the start, and laterally offset ones leave the horizontal field
    of view before they get close -- both are correct, and neither would test this.
    """
    frames = drive_and_record(lot, _sim(lot), CAMERA, SceneConfig())
    ranges = [s.true_range_m for f in frames for s in f.sightings if s.target.id == "pothole_11"]
    assert min(ranges) < 1.5
    assert max(ranges) > 20.0


def test_the_recorded_drive_is_reproducible(lot: LotSpec) -> None:
    first = drive_and_record(lot, _sim(lot), CAMERA, SceneConfig())
    second = drive_and_record(lot, _sim(lot), CAMERA, SceneConfig())
    assert [f.pose for f in first] == [f.pose for f in second]
    assert [f.ground_truth() for f in first] == [f.ground_truth() for f in second]


@pytest.mark.parametrize(
    "kwargs",
    [
        {"speed_mps": 0.0},
        {"camera_period_ticks": 0},
        {"target_spacing_m": -1.0},
        {"lateral_offsets_m": ()},
        {"target_size_m": 0.0},
        {"min_box_px": 0.0},
        {"max_steps": 0},
    ],
)
def test_scene_config_validates_itself(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        SceneConfig(**kwargs)  # type: ignore[arg-type]
