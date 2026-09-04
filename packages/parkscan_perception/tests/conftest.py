"""Shared fixtures: a camera, a mount, and frames that carry no image data.

Every frame here has ``image=None``. That is not a shortcut -- ``sim-lite`` renders
nothing and :class:`~parkscan_perception.simulated.SimulatedDetector` never reads pixels,
so a fabricated array would only disguise the fact that no vision is happening.

``make_frame`` is handed out as a fixture rather than imported across test modules: the
test tree has no ``__init__.py`` (pytest runs with ``--import-mode=importlib``), so a
cross-module import would depend on how pytest happens to name modules today.
"""

from __future__ import annotations

from collections.abc import Callable

import pytest
from parkscan_perception.geolocation import CameraIntrinsics, CameraMount

from parkscan_core.types import Frame

FRAME_COUNT = 24
STAMP_STEP_NS = 100_000_000  # 10 Hz
FIRST_STAMP_NS = 1_000_000_000


def _make_frame(frame_id: int, *, width: int = 640, height: int = 480) -> Frame:
    return Frame(
        frame_id=frame_id,
        stamp_ns=FIRST_STAMP_NS + frame_id * STAMP_STEP_NS,
        camera_id="cam_front",
        width=width,
        height=height,
        image=None,
    )


@pytest.fixture
def make_frame() -> Callable[[int], Frame]:
    return _make_frame


@pytest.fixture
def intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics()


@pytest.fixture
def mount() -> CameraMount:
    return CameraMount()


@pytest.fixture
def frames() -> tuple[Frame, ...]:
    return tuple(_make_frame(i) for i in range(FRAME_COUNT))
