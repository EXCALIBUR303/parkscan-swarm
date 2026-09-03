# Data Model

PostgreSQL 17 + PostGIS 3.5. Migrations via Alembic. All timestamps `timestamptz`, UTC.

**Two invariants enforced at the schema level, not by convention:**
1. Every row that represents an observation carries `data_source` **NOT NULL** — honesty is structural.
2. Positions are stored **twice**: `local_x/local_y` (the simulator's metric frame, the source of
   truth) and `geog geography(Point,4326)` (GIST-indexed, for proximity queries and the map).
   `geography`, never `geometry` — `ST_Distance` on `geometry(4326)` returns **degrees**.

```sql
CREATE TYPE data_source  AS ENUM ('simulated','replay','live');
CREATE TYPE det_source   AS ENUM ('simulated','model','ground_truth');
CREATE TYPE robot_state  AS ENUM ('idle','assigned','transit','inspecting',
                                  'waiting_on_lease','returning','charging','failed','offline');
CREATE TYPE coord_view   AS ENUM ('online','suspect','offline','stuck');
CREATE TYPE zone_state   AS ENUM ('unassigned','assigned','in_progress','complete','unrecoverable');
CREATE TYPE severity     AS ENUM ('low','medium','high');
CREATE TYPE bay_state    AS ENUM ('vacant','occupied','unknown');
```

### `run` — one simulation run; everything else is scoped to it
| column | type | notes |
|---|---|---|
| `id` | uuid PK | |
| `seed` | int NOT NULL | the whole point: any run is replayable |
| `strategy` | text NOT NULL | `greedy` \| `hungarian` \| `ssi` |
| `backend` | text NOT NULL | `sim-lite` \| `ros2-gz` |
| `data_source` | data_source NOT NULL DEFAULT 'simulated' | |
| `git_sha`, `config_hash` | text NOT NULL | every metric traceable to code + config |
| `started_at`, `ended_at` | timestamptz | |

### `robot`
`id` (text PK, `robot_1..3`), `run_id` FK, `state` robot_state, `coordinator_view` coord_view,
`battery_soc` real CHECK 0–1, `local_x/local_y` double precision, `geog` geography(Point,4326),
`heading_rad` real, `current_zone_id` FK NULL, `assignment_epoch` bigint, `last_heartbeat_at`,
`data_source` NOT NULL.
> `state` (self-reported) and `coordinator_view` (judged) are separate columns **on purpose** —
> the divergence between them is exactly what the recovery logic exists to handle.

### `zone`
`id` PK, `run_id` FK, `polygon` geography(Polygon,4326), `entry_waypoint`/`exit_waypoint`
geography(Point,4326), `sweep_length_m` real, `state` zone_state, `assigned_robot_id` FK NULL,
`epoch` bigint NOT NULL DEFAULT 0, `cells_total` int, `cells_done` int, `reassignment_count` int.
> `epoch` is the fencing token. A progress report is accepted **only** if `report.epoch == zone.epoch`.
> `cells_done` is why a requeued zone resumes instead of restarting.

### `telemetry` — time series, the high-volume table
`id` bigserial PK, `run_id`, `robot_id`, `stamp` timestamptz NOT NULL, `local_x/local_y`, `geog`,
`heading_rad`, `battery_soc`, `state`, `data_source` NOT NULL.
Index: `(run_id, robot_id, stamp DESC)`. Retention: truncated per run; not intended as a TSDB.

### `detection` — potholes and space occupancy
| column | type | notes |
|---|---|---|
| `id` | uuid PK | |
| `run_id`, `robot_id` | FK | |
| `kind` | text NOT NULL | `pothole` \| `space_occupancy` |
| `det_source` | det_source NOT NULL | `simulated` drives the SIMULATED badge |
| `detector_name` | text NOT NULL | e.g. `SimulatedDetector` / `rfdetr-nano` |
| `confidence` | real NOT NULL CHECK 0–1 | |
| `severity` | severity NULL | potholes only |
| `bbox_x1..y2` | real | image space, as returned by the detector |
| `local_x/local_y`, `geog` | | **geolocated**, post back-projection |
| `range_m` | real NOT NULL | ground range at detection — the error-budget axis |
| `geoloc_valid` | bool NOT NULL | false ⇒ rejected (above horizon, or range > 6 m) |
| `evidence_path` | text NULL | image crop on disk; NULL when the sim emits no frame |
| `stamp` | timestamptz NOT NULL | |

### `parking_space`
`id` PK, `run_id`, `zone_id` FK, `polygon` geography(Polygon,4326), `state` bay_state,
`last_observed_at`, `last_detection_id` FK NULL, `data_source` NOT NULL.

### `charging_bay`
`id` PK, `run_id`, `geog` geography(Point,4326), `lease_holder_id` FK NULL,
`lease_token` bigint NOT NULL DEFAULT 0, `lease_expires_at` timestamptz NULL.
> `lease_token` is monotonic per bay and only ever increases. `enter_bay(robot, token)` rejects
> `token < lease_token` — this is the fencing check that a TTL alone cannot provide.

### `resource_lease` — lanes and bays, one audit row per grant
`id` bigserial, `run_id`, `resource_id` text, `robot_id`, `token` bigint, `granted_at`,
`expires_at`, `released_at` NULL, `outcome` text (`released`\|`expired`\|`fenced_out`).

### `alert`
`id`, `run_id`, `robot_id` NULL, `zone_id` NULL, `kind` text
(`robot_offline`\|`robot_stuck`\|`battery_critical`\|`zone_unrecoverable`\|`lease_expired`\|`depletion`),
`message`, `stamp`, `acknowledged` bool.

### `metric_sample` — what `make bench` writes
`run_id`, `name` text, `value` double precision, `unit` text.
Names are fixed: `coverage_pct`, `makespan_s`, `total_distance_m`, `duplicate_visit_rate`,
`reassignment_latency_s`, `detection_latency_s`, `reassignment_churn`, `min_battery_at_dock`,
`depletion_events`, `conflict_count`.
> `reassignment_latency_s` and `detection_latency_s` are stored **separately and never summed**.
> Detection latency is just `k × heartbeat_period` — a config value, not an achievement.
> Conflating them into one "recovered in 3.2 s" figure is the standard overclaim in this space.
