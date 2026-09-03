# ADR-0006 — RF-DETR Nano (Apache-2.0); Ultralytics YOLO banned

**Status:** Accepted (Phase 0)

## Context
Ultralytics YOLO (v5/v8/v11/v12/YOLO26) is **AGPL-3.0**. This project serves a FastAPI backend and
React dashboard over a network, so AGPL §13 would force the **entire repository** — ROS nodes,
backend, frontend, MQTT glue and any fine-tuned weights — under AGPL-3.0.

## Decision
Real-model detector is **RF-DETR Nano, Apache-2.0**, pinned. `ultralytics` is banned, including
`from ultralytics import RTDETR` and any `yolov8*.pt` weights. **Only** the Apache-2.0
Nano/Small/Medium/Large checkpoints — never `rfdetr_plus`, and never the XL/2XL checkpoints, which
are PML-1.0: not OSI-approved, contingent on an active Roboflow agreement, and revocable.

A CI guard makes the trap unreintroducible:
```bash
pip freeze | grep -Eiq 'ultralytics|rfdetr_plus' && { echo "AGPL/PML dependency"; exit 1; }
```

## Rationale
Verified on this machine: `uv pip install rfdetr` succeeded, weights auto-downloaded with MD5
validation, **median 31.1 ms** on a 640×480 frame, and the dependency tree is provably AGPL-clean
(`ultralytics` absent across all 62 installed distributions). Alternatives considered: torchvision
(BSD-3, kept as the zero-extra-dependency fallback); D-FINE (Apache-2.0 throughout, but
clone-and-configure friction); MMDetection and YOLOX (Apache-2.0 but effectively stale — last pushes
2024-08 and 2025-06 — and will fight torch 2.14 / numpy 2.5).

## Consequences
- Perception runs **natively on the host**, never in Docker: neither MPS nor the CoreML EP is
  reachable from a Linux container, so a containerised detector silently degrades to slow CPU.
- Default runtime is PyTorch MPS (one dependency, no export step). ONNX Runtime + CoreML EP is a
  measured upgrade **only after benchmarking** — measured here, CoreML was *slower* than CPU
  (125.6 ms vs 85.8 ms) on a detector with in-graph NMS, because unsupported ops force partitioning.
  Any ONNX export must keep NMS **outside** the graph.
- Datasets are listed in `DATASETS.md` with licence and citation, downloaded by script, never
  committed. **Pothole Mix (CC BY-NC 3.0) is excluded.** ACPDS (MIT, disjoint lots per split) is the
  honest cross-lot generalisation test; PKLot and CNRPark reuse cameras across splits and flatter results.
