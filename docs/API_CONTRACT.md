# API Contract

> **Status.** Rows marked **NOT IMPLEMENTED** are specified but not built. They are left in
> rather than deleted so the gap between the design and the code is visible, and so nobody
> reads this file as a description of what exists. Everything unmarked is implemented and
> covered by tests in `apps/api/tests/`.

Base `/api/v1`. OpenAPI served at `/docs`. All responses include `data_source`.
Auth: **local dev mode only** for the MVP (no auth, bound to 127.0.0.1); the README documents
exactly what production auth would add (OAuth2 bearer + per-robot MQTT credentials + TLS).
That is stated as a known limitation, not implied to be done.

## REST

| Method | Path | Returns |
|---|---|---|
| `GET` | `/health` | `{status, db, mqtt, version, git_sha}` — liveness of each dependency, not a bare 200 |
| `GET` | `/runs` · `/runs/{id}` | run list / detail incl. `seed`, `strategy`, `backend`, `git_sha` |
| `GET` | `/robots` | all robots: state, coordinator_view, battery, pose, zone, epoch |
| `GET` | `/robots/{id}/telemetry?since=&limit=` | time series page |
| `GET` | `/zones` | zone polygons, state, `cells_done/cells_total`, assigned robot, epoch |
| `GET` | `/tasks` | **NOT IMPLEMENTED.** Assignments live in the coordinator's memory and reach the API only as the `zone` row's `assigned_robot_id`; the reassignment history is not persisted |
| `GET` | `/detections?kind=&severity=&min_confidence=&bbox=&since=` | filtered; `bbox` uses PostGIS |
| `GET` | `/detections/{id}/evidence` | image crop, or 404 when the backend emitted no frame |
| `GET` | `/parking-spaces` | occupancy per space + `last_observed_at` |
| `GET` | `/coverage` | per-zone coverage % + lot total (the choropleth's data source) |
| `GET` | `/alerts?acknowledged=` | **NOT IMPLEMENTED.** The `alert` table and its writer exist; no endpoint or UI reads them |
| `POST` | `/alerts/{id}/ack` | **NOT IMPLEMENTED** |
| `GET` | `/reports/run/{id}` | **NOT IMPLEMENTED.** Run metrics come from `make bench`, which writes `apps/runner/results/results.csv` directly and does not go through the API |

Errors are RFC 9457 `application/problem+json`.

## WebSocket `/ws/live`

Server→client, newline-free JSON, one envelope:
```json
{"type":"telemetry|detection|zone|alert|assignment|run_state",
 "stamp":"2026-09-03T10:00:00Z","data_source":"simulated","payload":{...}}
```
Client→server: `{"type":"subscribe","topics":["telemetry","alerts"]}` only. The socket is
**read-mostly by design** — commands go through REST so they are logged and testable.
Per-client queue is bounded (100, drop-oldest); the server sends a `{"type":"lag","dropped":N}`
notice rather than silently losing data.

## MQTT topics

`parkscan/v1/{run_id}/...` — versioned prefix so a schema change cannot be mistaken for live data.

| Topic | Dir | Payload |
|---|---|---|
| `robot/{id}/telemetry` | robot→ | pose, twist, battery, state, epoch — 1–10 Hz |
| `robot/{id}/connection` | robot→ | **retained + Last Will**: `{"state":"online\|connection_broken"}` |
| `robot/{id}/detection` | robot→ | one detection incl. `det_source`, bbox, geolocated point, `range_m` |
| `robot/{id}/progress` | robot→ | `{zone_id, epoch, cells_done:[...]}` — finer than assignment granularity |
| `coord/assignment` | coord→ | `{robot_id, zone_id, epoch}` |
| `coord/lease/{resource}` | coord→ | `{robot_id, token, expires_at}` |
| `coord/alert` | coord→ | alert records |

The Last Will is registered with the **broker** — never emulated in application code. The whole
credibility of "offline detected via broker last will" rests on the broker actually doing it.
