"""The simulation runner: owns the loop, ticks the agents, publishes telemetry.

This is the only place the three halves of the system meet. It constructs a
``WorldBackend`` (sim-lite today, a ROS 2 bridge later -- ADR-0001), gives each robot a
:class:`~parkscan_agent.robot.RobotAgent`, and publishes what comes back over MQTT.

Two properties are deliberate:

* **The loop is driven by the SIM clock, not the wall clock.** ``--realtime`` throttles
  to roughly real time so the dashboard is watchable; without it the run goes as fast as
  the CPU allows, which is what makes 30-seed experiments possible in Phase 4.
* **Provenance is stamped here.** The runner constructed the backend, so it is the only
  component that honestly knows the data is simulated (ACCEPTANCE H1).
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import datetime as dt
import json
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import cast

import aiomqtt
import structlog
from parkscan_agent.robot import AgentConfig, RobotAgent, TelemetrySample
from parkscan_perception.geolocation import (
    DEFAULT_MAX_RANGE_M,
    CameraIntrinsics,
    CameraMount,
    geolocate_detection,
)
from parkscan_perception.simulated import SimulatedDetector
from parkscan_sim.config import SimConfig
from parkscan_sim.world import SimLite

from parkscan_core.lot import LotSpec, load_lot
from parkscan_core.types import DataSource, RobotState
from parkscan_core.world import WorldBackend
from parkscan_runner.perception import (
    FrameTargets,
    make_frame,
    targets_for_frame,
    visible_targets,
)
from parkscan_runner.telemetry import (
    detection_payload,
    progress_payload,
    telemetry_payload,
)

log = structlog.get_logger(__name__)

DEFAULT_LOT = Path(__file__).resolve().parents[4] / "sim" / "lot" / "lot.yaml"


@dataclass(frozen=True, slots=True)
class RunnerConfig:
    lot_path: Path = DEFAULT_LOT
    seed: int = 42
    run_id: str = "dev"
    mqtt_host: str = "127.0.0.1"
    mqtt_port: int = 1883
    max_steps: int = 20_000
    realtime: bool = False
    publish_every: int = 2
    """Publish one sample per N ticks. At dt=0.05 s, 2 gives 10 Hz — the rate the
    dashboard's per-frame coalescing is designed around."""
    data_source: DataSource = DataSource.SIMULATED
    robots: int = 1
    zones: tuple[str, ...] = ()
    detect: bool = True
    """Run the perception stage. Off makes a pure-navigation run for debugging."""
    detect_every: int = 4
    """Camera ticks per N sim steps. At dt=0.05 s, 4 gives 5 Hz — the rate the
    research budgeted for a CPU-rendered camera, kept here so the message volume
    matches what a real backend could sustain."""
    start_soc: float | None = None
    """Override the simulator's initial charge. Exists so the return-to-charge and
    docking behaviour can be demonstrated in a short, reproducible run instead of
    waiting out a full discharge."""


def topic(run_id: str, robot_id: str, suffix: str) -> str:
    return f"parkscan/v1/{run_id}/robot/{robot_id}/{suffix}"


class Runner:
    """One simulation run."""

    def __init__(self, cfg: RunnerConfig) -> None:
        self._cfg = cfg
        self._lot: LotSpec = load_lot(cfg.lot_path)
        self._backend: WorldBackend = SimLite(SimConfig())
        self._backend.reset(cfg.seed, self._lot)
        if cfg.start_soc is not None:
            for robot_id in self._backend.robots():
                self._backend.set_battery_soc(robot_id, cfg.start_soc)
        # A single wall-clock instant, captured once, that the sim clock is offset from.
        self._epoch = dt.datetime.now(dt.UTC)

        available = list(self._backend.robots())
        self._robot_ids = available[: cfg.robots]
        self._agents = {
            robot_id: RobotAgent(robot_id, self._lot, self._backend, AgentConfig())
            for robot_id in self._robot_ids
        }
        self._assigned: dict[str, str] = {}

        self._intrinsics = CameraIntrinsics()
        self._mount = CameraMount()
        # One detector per robot, each seeded from the run seed and the robot's index, so
        # two robots do not draw an identical noise sequence and the whole run still
        # reproduces from a single --seed.
        self._targets = {robot_id: FrameTargets() for robot_id in self._robot_ids}
        self._detectors = {
            robot_id: SimulatedDetector(self._targets[robot_id], seed=cfg.seed * 1000 + index)
            for index, robot_id in enumerate(self._robot_ids)
        }
        self._frame_id = 0

    @property
    def lot(self) -> LotSpec:
        return self._lot

    def assign_initial_zones(self) -> None:
        """Phase 2 is one robot and a fixed zone list. Phase 4 replaces this wholesale
        with the coordinator; keeping it dumb here makes that swap obvious."""
        zones = list(self._cfg.zones) or [z.id for z in self._lot.zones]
        for index, robot_id in enumerate(self._robot_ids):
            zone_id = zones[index % len(zones)]
            self._agents[robot_id].assign(zone_id)
            self._assigned[robot_id] = zone_id
            log.info("zone_assigned", robot=robot_id, zone=zone_id)

    async def _perceive(self, client: aiomqtt.Client, samples: Sequence[object]) -> int:
        """Detect, then geolocate, then publish. Never merged, never reordered.

        The detector is handed an image-space Frame and nothing else. The pose is applied
        afterwards, by the geolocation stage, so a detection cannot read its own answer
        out of the map.
        """
        potholes = self._backend.potholes() if isinstance(self._backend, SimLite) else ()
        if not potholes:
            return 0

        count = 0
        for sample in samples:
            telemetry = cast("TelemetrySample", sample)
            robot_id = telemetry.robot_id
            pose = telemetry.pose

            visible = visible_targets(
                potholes,
                pose,
                self._intrinsics,
                self._mount,
                max_range_m=DEFAULT_MAX_RANGE_M,
            )
            self._frame_id += 1
            frame = make_frame(self._frame_id, telemetry.stamp_ns, robot_id, self._intrinsics)
            if not visible:
                continue

            self._targets[robot_id].set(frame.frame_id, targets_for_frame(visible))
            detector = self._detectors[robot_id]
            for detection in detector.detect(frame):
                result = geolocate_detection(detection, pose, self._intrinsics, self._mount)
                await client.publish(
                    topic(self._cfg.run_id, robot_id, "detection"),
                    json.dumps(
                        detection_payload(
                            detection,
                            result,
                            data_source=self._cfg.data_source,
                            stamp_ns=telemetry.stamp_ns,
                            epoch=self._epoch,
                        )
                    ).encode(),
                    qos=1,
                )
                count += 1
        return count

    async def run(self) -> int:
        cfg = self._cfg
        dt_s = SimConfig().dt_s
        published = 0
        detections = 0
        covered: set[str] = set()

        async with aiomqtt.Client(hostname=cfg.mqtt_host, port=cfg.mqtt_port) as client:
            log.info(
                "runner_started",
                seed=cfg.seed,
                robots=len(self._robot_ids),
                realtime=cfg.realtime,
                broker=f"{cfg.mqtt_host}:{cfg.mqtt_port}",
            )
            for step_index in range(cfg.max_steps):
                samples = [agent.tick() for agent in self._agents.values()]
                self._backend.step(dt_s)

                if step_index % cfg.publish_every == 0:
                    for sample in samples:
                        await client.publish(
                            topic(cfg.run_id, sample.robot_id, "telemetry"),
                            json.dumps(
                                telemetry_payload(
                                    sample, data_source=cfg.data_source, epoch=self._epoch
                                )
                            ).encode(),
                            qos=1,
                        )
                        published += 1

                if cfg.detect and step_index % cfg.detect_every == 0:
                    detections += await self._perceive(client, samples)

                for sample in samples:
                    covered.update(sample.covered_cells)
                    progress = progress_payload(sample, epoch=self._epoch)
                    if progress is not None:
                        await client.publish(
                            topic(cfg.run_id, sample.robot_id, "progress"),
                            json.dumps(progress).encode(),
                            qos=1,
                        )

                if cfg.realtime:
                    await asyncio.sleep(dt_s)

                if all(a.state is RobotState.IDLE for a in self._agents.values()) and covered:
                    log.info("all_work_complete", step=step_index)
                    break

            log.info(
                "runner_finished",
                steps=step_index + 1,
                published=published,
                detections=detections,
                cells_covered=len(covered),
                states={r: str(a.state) for r, a in self._agents.items()},
            )
        return 0


def parse_args(argv: Sequence[str] | None = None) -> RunnerConfig:
    parser = argparse.ArgumentParser(prog="parkscan_runner", description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--robots", type=int, default=1)
    parser.add_argument("--zones", default="", help="comma-separated zone ids")
    parser.add_argument("--steps", type=int, default=20_000)
    parser.add_argument("--run-id", default="dev")
    parser.add_argument("--mqtt-host", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--lot", type=Path, default=DEFAULT_LOT)
    parser.add_argument(
        "--start-soc",
        type=float,
        default=None,
        help="initial state of charge in [0,1]; a low value demonstrates return-to-charge",
    )
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="throttle to ~real time so the dashboard is watchable (default: as fast as possible)",
    )
    args = parser.parse_args(argv)
    return RunnerConfig(
        lot_path=args.lot,
        seed=args.seed,
        run_id=args.run_id,
        mqtt_host=args.mqtt_host,
        mqtt_port=args.mqtt_port,
        max_steps=args.steps,
        realtime=args.realtime,
        robots=args.robots,
        zones=tuple(z for z in args.zones.split(",") if z),
        start_soc=args.start_soc,
    )


def main(argv: Sequence[str] | None = None) -> int:
    structlog.configure(wrapper_class=structlog.make_filtering_bound_logger(20))
    cfg = parse_args(argv)
    runner = Runner(cfg)
    runner.assign_initial_zones()
    with contextlib.suppress(KeyboardInterrupt):
        return asyncio.run(runner.run())
    return 130


if __name__ == "__main__":
    sys.exit(main())
