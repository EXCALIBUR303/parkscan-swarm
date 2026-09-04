"""Perception: the detector interface, a simulated detector, and monocular geolocation.

Two stages that never touch each other's inputs. A
:class:`~parkscan_perception.detector.Detector` sees a frame and emits boxes in image
space; :mod:`~parkscan_perception.geolocation` takes a box and a pose and produces a point
in the lot. The detector is never given the pose, so it cannot report a position it did not
see, and each stage is tested with no trace of the other present.

Nothing in this package is a claim about real-world detection accuracy.
:class:`~parkscan_perception.simulated.SimulatedDetector` performs no vision at all -- it
converts geometry the simulator already knows into detection-shaped records so that
everything downstream is genuinely exercised. Its ``det_source`` is ``simulated``, that
value is persisted, and the dashboard's SIMULATED badge is rendered from it
(``docs/ACCEPTANCE.md`` H1--H4).

Public surface::

    from parkscan_perception import (
        CameraIntrinsics, CameraMount, GroundTruthTarget, SimulatedDetector,
        StaticTargets, assert_detector_contract, geolocate_detection,
    )

    det = SimulatedDetector(StaticTargets({0: (target,)}), seed=42)
    assert_detector_contract(det, frames)
    result = geolocate_detection(det.detect(frame)[0], pose, CameraIntrinsics(), CameraMount())
"""

from parkscan_perception.contract import (
    DEFAULT_CONTRACT_THRESHOLDS,
    assert_detector_contract,
)
from parkscan_perception.detector import (
    DEFAULT_SCORE_THRESHOLD,
    Detector,
    frame_rng,
    frame_seed,
)
from parkscan_perception.geolocation import (
    DEFAULT_MAX_RANGE_M,
    R_C_OPT,
    REASON_ABOVE_HORIZON,
    REASON_BEYOND_MAX_RANGE,
    REASON_NON_FINITE,
    REASON_OK,
    CameraIntrinsics,
    CameraMount,
    GeolocResult,
    backproject_pixel,
    camera_ray,
    geolocate_detection,
    project_ground_point,
)
from parkscan_perception.simulated import (
    DEFAULT_CLASSES,
    DEFAULT_NOISE,
    GroundTruthTarget,
    NoiseModel,
    SimulatedDetector,
    StaticTargets,
    TargetProvider,
)

__all__ = [
    "DEFAULT_CLASSES",
    "DEFAULT_CONTRACT_THRESHOLDS",
    "DEFAULT_MAX_RANGE_M",
    "DEFAULT_NOISE",
    "DEFAULT_SCORE_THRESHOLD",
    "REASON_ABOVE_HORIZON",
    "REASON_BEYOND_MAX_RANGE",
    "REASON_NON_FINITE",
    "REASON_OK",
    "R_C_OPT",
    "CameraIntrinsics",
    "CameraMount",
    "Detector",
    "GeolocResult",
    "GroundTruthTarget",
    "NoiseModel",
    "SimulatedDetector",
    "StaticTargets",
    "TargetProvider",
    "assert_detector_contract",
    "backproject_pixel",
    "camera_ray",
    "frame_rng",
    "frame_seed",
    "geolocate_detection",
    "project_ground_point",
]
