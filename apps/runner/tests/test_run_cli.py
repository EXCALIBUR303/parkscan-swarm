"""``parse_args``'s ``--backend`` flag (ADR-0008 §7). Everything else about the runner
gets exercised for real by the failure-injection tests, which run the default
``sim-lite`` path end to end -- there is no equivalent for ``ros2-gz`` here, since
constructing it for real dials an MQTT broker in its own constructor. That path is
exercised by actually running the ``ros2-gz`` compose profile, not by a unit test."""

from __future__ import annotations

import pytest
from parkscan_runner.run import RunnerConfig, parse_args


def test_backend_defaults_to_sim_lite() -> None:
    cfg = parse_args([])
    assert cfg.backend == "sim-lite"


def test_backend_flag_selects_ros2_gz() -> None:
    cfg = parse_args(["--backend", "ros2-gz"])
    assert cfg.backend == "ros2-gz"


def test_backend_flag_rejects_an_unknown_value() -> None:
    with pytest.raises(SystemExit):
        parse_args(["--backend", "not-a-real-backend"])


def test_runner_config_backend_field_defaults_to_sim_lite() -> None:
    assert RunnerConfig().backend == "sim-lite"
