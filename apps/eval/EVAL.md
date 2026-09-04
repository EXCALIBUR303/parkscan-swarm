# EVAL.md — what the perception pipeline was measured at, and what it was not

**Every number in this document is Tier 1.** Tier 1 means: the behaviour of *this pipeline*
against *known simulated ground truth*, on a synthetic lot, under a *stated* error budget.
Nothing here is a measurement of how well anything finds a real pothole in a real car park,
and nothing here may be re-quoted as if it were (`docs/ACCEPTANCE.md` H3, H4).

Regenerate everything below with:

```
PYTHONPATH=apps/eval/src .venv/bin/python -m parkscan_eval.run_eval --seed 42 --out apps/eval/out
```

---

## The three tiers, stated before any number

### Tier 1 — what a simulated detector legitimately measures (and what is here)

`SimulatedDetector` does not look at pixels. It takes geometry the simulator already knows
and returns detection-shaped records with seeded box jitter, a drawn confidence, a miss rate
and a false-positive rate. Running it therefore exercises **the pipeline**, and the pipeline
is a real thing with real properties that can be measured honestly:

| Tier 1 metric | Reported below |
|---|---|
| Geolocation accuracy against known simulated truth, **as a function of range** | [§ Error vs range](#geolocation-error-versus-range-seed-42) |
| Which error source dominates, measured by ablation | [§ Ranked error sources](#ranked-error-sources-measured) |
| Horizon- and range-rejection rate, with reasons broken out | [§ Rejection](#rejection-behaviour) |
| Detector-contract conformance (acceptance criterion C12) | [§ Contract](#detector-contract-c12) |
| Per-frame pipeline latency on this machine | [§ Latency](#pipeline-latency-this-machine-only) |

### Tier 2 — what only a real model on a public benchmark could give (NOT MEASURED HERE)

These are absent from this repository, from `results.json` and from the README. They are not
"coming soon" and they have not been approximated:

- **mAP@50** and **mAP@50-95**
- **Precision and recall at a stated confidence threshold**
- Per-class breakdowns of any of the above

Producing them requires three things this harness does not have: a **real detector** (the
Apache-2.0 RF-DETR Nano of ADR-0006 — never Ultralytics YOLO, which is AGPL-3.0, and never
`rfdetr_plus` or the XL/2XL checkpoints, which are PML-1.0), **real images**, and a **named
public test split**. When they are produced, the split must be named in the same sentence as
the number. The intended splits are listed in `docs/adr/0006-detector-licensing.md`: ACPDS
(MIT) is the honest cross-lot generalisation test because its lots are disjoint across
splits, while PKLot and CNRPark+EXT reuse cameras across splits and flatter results. Pothole
Mix is excluded outright: CC BY-NC 3.0.

A Tier 2 number may **never** be blended with a Tier 1 number into one headline figure. "94%
accurate" formed from a simulated detector's miss rate is the exact overclaim H4 exists to
forbid — that number would be reporting a constant somebody typed into `NoiseModel`.

### Tier 3 — what may NEVER be claimed, by anyone, at any point in this project

- Real-world detection accuracy of any kind.
- Any number, phrasing or screenshot implying validation on a physical car park.
- "Field-tested", "deployed", "validated on real data", "production-ready".
- A `sim-lite` screenshot presented as a physics simulation, or this system described as a
  ROS 2 system before the `ros2-gz` backend actually runs (H12).

The guarantee is structural rather than editorial: every detection carries
`det_source = simulated`, it is a NOT NULL column in Postgres, and the dashboard's SIMULATED
badge is rendered from that column (H1, H2). A test in this package asserts that every row
this harness produces is labelled `simulated`, and another asserts that `results.json`
contains no key named `map`, `precision`, `recall`, `f1` or `accuracy`.

---

## How the Tier 1 experiment works

1. Fifteen ground-truth targets (0.45 m modelled potholes) are placed at **exactly known**
   positions along zone_01's drive aisle. Their positions come from the committed lot file —
   the aisle is the segment between the zone's `entry_waypoint` and `exit_waypoint` lane
   nodes — and from **no random draw at all**. The seed controls what the detector and the
   error budget *do*, never where anything *is*.
2. A robot is driven down that aisle by `SimLite` itself, at 1 m/s, with the camera firing
   every 4 ticks (5 Hz at `dt = 0.05 s`).
3. At each exposure, every target that is genuinely visible is projected into the image by
   `parkscan_eval.camera` — an **independent** forward projection, written from the formula
   in `docs/ARCHITECTURE.md` §5 rather than imported from the module being graded — to
   produce a box whose **bottom-centre is exactly the ground-contact pixel**.
4. `SimulatedDetector` turns those boxes into detections, adding its own bounded jitter,
   confidence, misses and false positives.
5. Each detection is geolocated by `parkscan_perception.geolocation` from a **perturbed**
   pose and mount, then compared with the known truth.

**Frames carry `image=None`.** `sim-lite` renders nothing and `SimulatedDetector` never reads
pixels; fabricating an array of tarmac-coloured noise would be a lie that every check
downstream would happily pass.

### Why the pose and mount are perturbed

With a perfect pose and mount, back-projection inverts forward projection to about `1e-13` m
at every range. A harness that reported *that* would be reporting the accuracy of IEEE-754
arithmetic. So the geolocation stage is given the pose and mount a real robot would have: the
true ones plus a stated error budget.

| Term | σ | What it models |
|---|---|---|
| `pitch_sigma_rad` | 0.5° | attitude estimation error, the dominant term |
| `yaw_sigma_rad` | 0.5° | heading error |
| `height_sigma_m` | 5 mm | mount-height error |
| `position_sigma_m` | 20 mm per axis | pose drift |
| detector `jitter_px` | ±2 px per box edge | bbox bottom-edge localisation |

**These are authored magnitudes, not measurements of any hardware.** Every number below is
only meaningful relative to them: halve `pitch_sigma_rad` and roughly every error halves. They
are hashed into `results.json` so a figure cannot be quoted without the budget that produced
it. Draws are Gaussian and therefore unbounded — deliberately, because the interesting failure
at long range is exactly the tail, where an attitude error larger than the depression angle
tips the ray *above the horizon* and geolocation refuses to answer at all.

### The self-check that runs before every evaluation

The harness owns a forward projection; the perception package owns the back-projection. If
those two ever disagree about a pitch sign or an axis permutation, **nothing crashes** — the
pipeline keeps producing confident, plausible, wrong points on the ground, and this document
would report their error as a pipeline result. So `run_eval` round-trips a grid of known
ground points through both stages, with no error injected and the range gate effectively off,
and aborts the run if any point fails to return within 1 µm. Measured worst-case round-trip
on this machine: **below 1e-9 m**.

---

## Geolocation error versus range (seed 42)

Errors are Euclidean, in metres, binned by the target's **true** range — not its estimated
range. That choice matters: binning by the estimate contaminates the far bins in the
direction that flatters the result, because a sample whose true range is 18 m and whose
estimate lands at 40 m leaves the 16–20 m bin and joins the 20+ one, so every bin below
reports only the errors that happened to stay small.

The range gate is raised to 1000 m for this table (a clearly-labelled second pass over the
same detections and the same perturbed poses) so that the growth **past** the production gate
is visible. A table that stopped at 6 m could only assert the gate; this one justifies it.

| true range | n | mean (m) | p50 (m) | p95 (m) |
|---|---:|---:|---:|---:|
| 0–2 m | 40 | 0.050 | 0.037 | 0.106 |
| 2–4 m | 132 | 0.176 | 0.141 | 0.465 |
| 4–6 m | 135 | 0.471 | 0.387 | 1.094 |
| 6–8 m | 119 | 0.884 | 0.750 | 2.226 |
| 8–12 m | 223 | 1.914 | 1.383 | 5.127 |
| 12–16 m | 194 | 3.888 | 2.736 | 10.537 |
| 16–20 m | 162 | 6.582 | 4.621 | 21.722 |
| 20+ m | 194 | 16.096 | 7.077 | 52.864 |

Inside the 6 m production gate, over 320 accepted detections: **mean 0.315 m, p50 0.184 m,
p95 0.986 m**.

The growth is quadratic, and visibly so: dividing each bin's mean by the square of its
midpoint gives 0.020, 0.019, 0.018, 0.019, 0.019, 0.020 m/m² across the 2–20 m bins. That is
the geometry, not a coincidence — for a camera at height `h` whose ray meets the ground at
range `R`, the depression angle satisfies `tan(a) = h/R`, so an attitude error `da` displaces
the ground point by about `da · (R² + h²) / h`. The 0–2 m bin sits above the trend because
there the constant 20 mm pose term, not the attitude term, is what is left.

`p95` and the median are reported next to the mean because the distribution is
heavy-tailed at range: at 16–20 m the mean is 6.6 m while the median is 4.6 m and the 95th
percentile is 21.7 m. Quoting the mean alone at long range would understate the tail by a
factor of three.

---

## Ranked error sources (measured)

Each row is a **separate run of the harness with exactly one error source enabled**, seed 42,
same scene, same targets. Values are mean error in metres.

| rank | source | magnitude | 0–2 m | 2–4 m | 4–6 m | 6–8 m | 8–12 m | 12–16 m | 16–20 m |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | **Pitch / attitude** | σ = 0.5° | 0.040 | 0.166 | 0.462 | 0.823 | 1.909 | 5.239 | 6.953 |
| 2 | *Flat-ground assumption* | see below | — | — | — | — | — | — | — |
| 3 | **Bbox bottom-edge localisation** | ±2 px | 0.010 | 0.048 | 0.120 | 0.232 | 0.465 | 0.907 | 1.456 |
| 4 | **Camera height** | σ = 5 mm | 0.013 | 0.025 | 0.039 | 0.056 | 0.081 | 0.107 | 0.149 |
| 5 | **Yaw** | σ = 0.5° | 0.010 | 0.024 | 0.040 | 0.056 | 0.080 | 0.111 | 0.142 |
| 6 | **Pose drift (translation)** | σ = 20 mm/axis | 0.027 | 0.025 | 0.026 | 0.025 | 0.026 | 0.026 | 0.025 |
| 7 | *Lens distortion* | not modelled | — | — | — | — | — | — | — |
| 8 | *Time synchronisation* | not modelled | — | — | — | — | — | — | — |
| | **All of the above, together** | | 0.050 | 0.176 | 0.471 | 0.884 | 1.914 | 3.888 | 6.582 |

Reading the ranking:

- **Pitch dominates by an order of magnitude** and is the only term that grows with `R²`.
  Everything else is linear in `R` (yaw, height, bbox) or flat (pose translation). This is why
  a range gate, rather than a better estimator, is the right primary control.
- **Rank 2, the flat-ground assumption, is not measured by this harness and cannot be**: the
  harness places every target *exactly on* the plane `z = 0`, so the assumption it makes is
  true by construction. In reality it is false by definition — a pothole is a depression
  *below* the plane. The analytic magnitude is large enough that it belongs at rank 2 anyway:
  for a hole of depth `d` whose visible bottom is back-projected onto the plane, the recovered
  range is short by `R · d / (h + d)`. At the modelled 0.45 m mount height, a 50 mm-deep hole
  costs **10% of range** — 0.6 m at 6 m — which is comparable with the entire attitude budget
  at that range and *systematic* rather than zero-mean. Measuring it needs a simulator with
  depth, which `sim-lite` does not have and does not pretend to have.
- **Ranks 4, 5 and 6 are close together, and their order is a property of the assumed
  magnitudes, not of the geometry.** With 20 mm of pose drift against 5 mm of height error,
  translation ranks last; at 5 mm of pose drift it would rank above yaw. `docs/ARCHITECTURE.md`
  §5 lists pose drift above camera height for that reason. Treat ranks 4–6 as one band.
- **Ranks 7 and 8 are absent because they are not modelled.** `sim-lite` renders nothing, so
  there is no lens to distort and no acquisition delay to be out of sync with the pose. They
  are listed so the omission is visible rather than implied. On real hardware both would enter
  the budget, and time sync in particular scales with vehicle speed rather than with range.

---

## The valid range, justified by the table rather than asserted

The production gate is **6 m** (`DEFAULT_MAX_RANGE_M`), and the case for it is the practical
consequence of the error, not the error itself. The lot's parking stalls are 2.5 m wide, so a
geolocation error above **1.25 m** is enough to place a pothole in the neighbouring stall, and
an error above 2.5 m puts it a whole stall away.

| true range | n | error > 0.5 m | error > 1.25 m (wrong stall) | error > 2.5 m |
|---|---:|---:|---:|---:|
| 0–2 m | 40 | 0.0% | 0.0% | 0.0% |
| 2–4 m | 132 | 3.0% | 0.0% | 0.0% |
| 4–6 m | 135 | 34.8% | 3.0% | 0.7% |
| 6–8 m | 119 | 67.2% | **22.7%** | 4.2% |
| 8–12 m | 223 | 84.3% | **52.0%** | 26.0% |
| 12–16 m | 194 | 91.2% | 72.7% | 54.6% |
| 16–20 m | 162 | 94.4% | 88.9% | 72.8% |
| 20+ m | 194 | 97.9% | 94.3% | 83.5% |

The wrong-stall rate is 3.0% inside the gate and jumps to 22.7% in the very next bin, then to
52.0% by 12 m. That step is the argument. A gate at 8 m would let one detection in five land
against the wrong stall; a gate at 4 m would discard usable data — 0.0% wrong-stall in 2–4 m.
6 m is where the curve turns, and it is chosen from this table.

Two independent facts point at the same limit:

- **Above-horizon rejections begin at about 16 m.** In this run the seven `above_horizon`
  rejections occurred at true ranges of 15.9, 18.4, 19.6, 20.9, 21.8, 23.4 and 24.3 m. Past
  about 16 m the depression angle (`atan(0.45 / 16)` ≈ 1.6°) is small enough that a
  three-sigma pitch error tips the ray above the horizon entirely. Beyond that range the
  stage cannot answer at all, however good the detector is.
- **Targets stop being resolvable at about 25 m** anyway: a 0.45 m pothole is 8 px wide at
  25.7 m with this lens, and the harness refuses to synthesise a box below `min_box_px = 8`
  rather than characterise the pipeline over a range nothing could detect at.

**What the gate does at the boundary.** Rays at or above the horizon are **rejected, never
clamped** (`docs/ACCEPTANCE.md` C11). Clamping a near-horizon ray to some maximum yields a
confident, well-formed, plausible world point that is simply wrong, and it poisons the map
silently rather than loudly. Rejected results carry `geoloc_valid = false` and a reason, and
range rejections carry the range that failed the gate so the failure is analysable.

---

## Rejection behaviour

Over 1209 geolocation attempts in the seed-42 run:

| outcome | n | share |
|---|---:|---:|
| `accepted` | 320 | 26.5% |
| `beyond_max_range` | 882 | 73.0% |
| `above_horizon` | 7 | 0.6% |

**The 73% range-gate share is a property of where the targets were placed, not a defect
rate.** The scene deliberately observes every target from as far away as it is resolvable
(up to 26.4 m) down to 0.6 m, because that is what makes the error-versus-range table
possible. A production sweep, which only needs to inspect the aisle it is driving down,
would see a completely different mix. The figure that means something on its own is the
`above_horizon` share: 0.6% of rays never met the ground, and the stage said so instead of
inventing a point.

---

## Detector contract (C12)

`assert_detector_contract` passes for `SimulatedDetector` over all 201 frames of this run:
**1 of 1 detectors passed, 0 failed.** The contract checks determinism across repeated calls,
that `warmup()` does not change the answer, that emitted classes are declared, that scores
respect the requested threshold, that boxes lie inside the frame, that `det_source` matches
the detector's declared source, and that raising the threshold never adds detections. A
contract failure makes `run_eval` exit non-zero: it is a build failure, not a chart.

---

## Pipeline latency (this machine only)

Per **camera frame** — not per detection — over 201 frames averaging 6.0 detections each,
on an Apple M5 (macOS 27.0, arm64, CPython 3.12.12, numpy 2.5.2):

| stage | p50 | p95 |
|---|---:|---:|
| `detect` | 0.049 ms | 0.073 ms |
| `geolocate` (production gate, all detections in the frame) | 0.063 ms | 0.097 ms |
| frame total | 0.113 ms | 0.165 ms |

Three caveats, all of which matter more than the numbers:

- **`SimulatedDetector` performs no inference.** These figures measure a seeded noise model
  and some 3×3 matrix arithmetic. The real-model cost is a different measurement entirely:
  RF-DETR Nano was measured at a **median 31.1 ms per 640×480 frame** on this machine
  (`docs/RESEARCH_LOG.md` §5) — roughly 600× the whole simulated pipeline. Do not add these
  numbers to that one and present the sum as a system latency.
- **These are the only non-reproducible values the harness emits**, and `results.json` marks
  the block `"reproducible": false`. Everything else in that file, and every byte of
  `results.csv`, is identical between two runs of the same seed.
- The gate-relaxed second geolocation pass is deliberately **outside** the timed region. The
  running system does not perform it, and charging its cost to the pipeline would misreport
  the pipeline by a factor of two.

---

## Reproducibility

`results.csv` is **byte-identical** between two runs of the same seed on the same lot with the
same configuration, and a test in `apps/eval/tests/test_run_eval.py` asserts exactly that by
running the CLI twice and comparing bytes. What makes it hold:

- The scene geometry is a function of the committed lot file alone — no draw places anything.
- `SimLite` is seeded and bit-reproducible; `now_ns()` is `tick × dt`, never a wall clock.
- `SimulatedDetector` derives its generator from `(seed, frame_id)` via SplitMix64, so it does
  not depend on `PYTHONHASHSEED` or on how many frames preceded this one.
- The error budget is drawn from one generator, seeded from the run seed on a stream that
  cannot collide with the detector's, in a **single fixed-shape call** — so what any frame
  gets never depends on how many detections an earlier frame produced.
- Floats are written with `repr()`, the shortest exactly round-trippable form, and the CSV is
  written with `\n` line endings rather than `csv`'s CRLF default.
- No dict-ordering dependence anywhere: rejection reasons are sorted by `(-count, reason)`,
  matching is greedy over a list sorted by `(distance, detection index, sighting index)`.

`results.json` records the git SHA (and whether the tree was dirty), a SHA-256 hash of the
full configuration, the seed, and the installed version of every library that could move a
number. It is byte-identical too, **except** its `latency` block.

## Honest limitations of this experiment

- One zone, one aisle, one straight drive, one lot. The lot is synthetic and anchored at
  latitude 0 / longitude 0 — an obviously artificial location — so no reader can mistake it
  for a real address.
- The ground is exactly flat, so the largest *systematic* error source in the real problem is
  structurally absent (rank 2 above).
- The miss rate and false-positive rate are constants somebody typed into `NoiseModel`. Real
  detectors miss in *correlated* ways — distance, occlusion, motion blur, class confusion —
  and no aggregate of these constants describes any real detector.
- There is no sensor noise model, no lens, no motion blur and no rolling shutter.
- Detections are associated with targets by proximity of the bbox bottom-centre within a gate
  derived from the detector's own jitter bound (4 px at the default). Unmatched detections —
  the detector's false positives — are kept in `results.csv`, counted, and excluded from the
  error table, because a harness that quietly dropped them would be measuring a detector that
  does not exist.
- **Single seed.** Every table above is seed 42. Unlike `make bench` in Phase 4, this harness
  does not yet run a seed sweep or report confidence intervals, so the bin-to-bin figures
  carry sampling noise that is not quantified here — visible in the 12–16 m pitch-only cell
  (5.239 m) sitting above the corresponding combined-budget cell (3.888 m), which is sampling
  scatter in a heavy-tailed distribution, not a real effect.
