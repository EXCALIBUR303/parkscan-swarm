"""``make bench`` — the 30-seed, 3-strategy experiment.

Deliberately runs the simulator and coordinator **with no MQTT, no database and no
dashboard**. The experiment measures allocation, not plumbing, and a broker in the loop
would add wall-clock noise to a result that is otherwise exactly reproducible from
``(seed, strategy)``. The live pipeline is exercised by ``make demo`` and ``make smoke``
instead.

Every number the README quotes must come from ``results.csv`` written here, and the file
is committed, so a reader can check any claim against the row that produced it.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from parkscan_agent.robot import AgentConfig, RobotAgent
from parkscan_coord.allocation import Strategy
from parkscan_coord.coordinator import Coordinator, CoordinatorConfig
from parkscan_coord.metrics import paired_bootstrap
from parkscan_sim.config import SimConfig
from parkscan_sim.world import SimLite

from parkscan_core.lot import LotSpec, load_lot
from parkscan_core.types import RobotState
from parkscan_runner.fleet import StrategyAllocator
from parkscan_runner.run import DEFAULT_LOT

MAX_STEPS = 60_000


@dataclass(frozen=True, slots=True)
class RunResult:
    """One (seed, strategy) run."""

    seed: int
    strategy: str
    makespan_s: float
    total_distance_m: float
    coverage_pct: float
    duplicate_visit_rate: float
    reassignment_churn: float
    min_battery_at_dock: float
    depletion_events: int
    conflict_count: int
    steps: int
    completed: bool

    def row(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "strategy": self.strategy,
            "makespan_s": round(self.makespan_s, 3),
            "total_distance_m": round(self.total_distance_m, 3),
            "coverage_pct": round(self.coverage_pct, 3),
            "duplicate_visit_rate": round(self.duplicate_visit_rate, 5),
            "reassignment_churn": round(self.reassignment_churn, 5),
            "min_battery_at_dock": round(self.min_battery_at_dock, 4),
            "depletion_events": self.depletion_events,
            "conflict_count": self.conflict_count,
            "steps": self.steps,
            "completed": self.completed,
        }


def run_once(lot: LotSpec, seed: int, strategy: Strategy) -> RunResult:
    """One headless run. Pure: same (seed, strategy) gives the same result, always."""
    sim = SimLite(SimConfig())
    sim.reset(seed, lot)
    robot_ids = list(sim.robots())
    agents = {r: RobotAgent(r, lot, sim, AgentConfig()) for r in robot_ids}
    coord = Coordinator(
        lot, agents, allocator=StrategyAllocator(strategy), config=CoordinatorConfig()
    )
    coord.start(0, poses={r: sim.sense(r).pose for r in robot_ids})

    dt_s = sim.config.dt_s
    visits: dict[str, int] = {}
    distance = 0.0
    last = {r: (sim.pose(r).x_m, sim.pose(r).y_m) for r in robot_ids}
    min_dock_soc = 1.0
    completed = False
    steps = 0

    for step in range(MAX_STEPS):
        steps = step + 1
        now_ns = sim.now_ns()
        for agent in agents.values():
            sample = agent.tick()
            coord.ingest(sample)
            for cell in sample.covered_cells:
                visits[cell] = visits.get(cell, 0) + 1
        coord.tick(now_ns)
        sim.step(dt_s)

        for robot in robot_ids:
            pose = sim.pose(robot)
            here = (pose.x_m, pose.y_m)
            distance += ((here[0] - last[robot][0]) ** 2 + (here[1] - last[robot][1]) ** 2) ** 0.5
            last[robot] = here
            if sim.is_charging(robot):
                min_dock_soc = min(min_dock_soc, sim.battery_soc(robot))

        if all(a.state is RobotState.IDLE for a in agents.values()) and visits:
            completed = True
            break

    planned: set[str] = set()
    done: set[str] = set()
    for zone in lot.zones:
        status = coord.zone_status(zone.id)
        planned.update(status.cells_total)
        done.update(status.cells_done)

    duplicates = sum(count - 1 for count in visits.values() if count > 1)
    return RunResult(
        seed=seed,
        strategy=str(strategy),
        makespan_s=steps * dt_s,
        total_distance_m=distance,
        coverage_pct=100.0 * len(done & planned) / len(planned) if planned else 0.0,
        duplicate_visit_rate=duplicates / len(planned) if planned else 0.0,
        reassignment_churn=(
            coord.voluntary_reassignments / coord.assignments_total
            if coord.assignments_total
            else 0.0
        ),
        min_battery_at_dock=min_dock_soc,
        depletion_events=sum(1 for r in robot_ids if sim.is_depleted(r)),
        conflict_count=len(sim.collisions()),
        steps=steps,
        completed=completed,
    )


def _git_sha() -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except (OSError, subprocess.CalledProcessError):  # pragma: no cover - not in a repo
        return "unknown"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="parkscan_runner.bench", description=__doc__)
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--out", type=Path, default=Path("apps/runner/results"))
    parser.add_argument("--lot", type=Path, default=DEFAULT_LOT)
    args = parser.parse_args(argv)

    lot = load_lot(args.lot)
    strategies = list(Strategy)
    results: list[RunResult] = []

    for seed in range(args.seeds):
        for strategy in strategies:
            result = run_once(lot, seed, strategy)
            results.append(result)
        print(
            f"  seed {seed:2d}  "
            + "  ".join(
                f"{r.strategy}: makespan {r.makespan_s:7.1f} s, {r.total_distance_m:7.1f} m"
                for r in results[-len(strategies) :]
            )
        )

    args.out.mkdir(parents=True, exist_ok=True)
    csv_path = args.out / "results.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(results[0].row()))
        writer.writeheader()
        for result in results:
            writer.writerow(result.row())

    by_strategy = {str(s): [r for r in results if r.strategy == str(s)] for s in strategies}
    comparisons: list[dict[str, Any]] = []
    baseline = str(Strategy.GREEDY)
    for name, runs in by_strategy.items():
        if name == baseline:
            continue
        for metric in ("makespan_s", "total_distance_m"):
            a = [getattr(r, metric) for r in by_strategy[baseline]]
            b = [getattr(r, metric) for r in runs]
            stat = paired_bootstrap(
                a, b, seed=0, metric=metric, strategy_a=baseline, strategy_b=name
            )
            comparisons.append(
                {
                    "baseline": baseline,
                    "challenger": name,
                    "metric": metric,
                    "median_difference": stat.median_difference,
                    "ci_low": stat.ci_low,
                    "ci_high": stat.ci_high,
                    "wins": stat.wins,
                    "losses": stat.losses,
                    "ties": stat.ties,
                    "n_pairs": stat.n_pairs,
                    "significant": stat.significant,
                    "summary": stat.summary(),
                }
            )

    summary = {
        "git_sha": _git_sha(),
        "seeds": args.seeds,
        "strategies": [str(s) for s in strategies],
        "comparisons": comparisons,
        "correctness": {
            "runs": len(results),
            "all_completed": all(r.completed for r in results),
            "max_conflict_count": max(r.conflict_count for r in results),
            "total_depletion_events": sum(r.depletion_events for r in results),
            "min_coverage_pct": min(r.coverage_pct for r in results),
            "max_duplicate_visit_rate": max(r.duplicate_visit_rate for r in results),
            "max_reassignment_churn": max(r.reassignment_churn for r in results),
        },
    }
    (args.out / "results.json").write_text(json.dumps(summary, indent=2) + "\n")

    print(f"\n  Wrote {csv_path} ({len(results)} rows) and {args.out / 'results.json'}")
    print("\n  Correctness gates (ACCEPTANCE C1-C4):")
    for key, value in summary["correctness"].items():
        print(f"    {key:26s} {value}")
    print("\n  Paired comparisons vs greedy (bootstrap 95% CI over the same seeds):")
    for comparison in comparisons:
        print(f"    {comparison['summary']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
