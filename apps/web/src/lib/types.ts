/**
 * Domain types mirrored from docs/DATA_MODEL.md.
 *
 * These are the Postgres enums, not invented UI states. If a value here drifts from the
 * migration, the API contract test is where it should break — the dashboard must never
 * quietly widen an enum to make a render succeed.
 */

/** `data_source` — NOT NULL on every observation row (ACCEPTANCE H1). */
export type DataSource = 'simulated' | 'replay' | 'live';

/** `det_source` — NOT NULL on every detection row. */
export type DetSource = 'simulated' | 'model' | 'ground_truth';

/** `robot_state` — what the robot says about itself. */
export type RobotState =
  | 'idle'
  | 'assigned'
  | 'transit'
  | 'inspecting'
  | 'waiting_on_lease'
  | 'returning'
  | 'charging'
  | 'failed'
  | 'offline';

/** `coord_view` — what the coordinator judges, deliberately separate from `RobotState`. */
export type CoordinatorView = 'online' | 'suspect' | 'offline' | 'stuck';

/** `zone_state`. */
export type ZoneState = 'unassigned' | 'assigned' | 'in_progress' | 'complete' | 'unrecoverable';

/** `severity` — potholes only. */
export type Severity = 'low' | 'medium' | 'high';

/** `bay_state` — parking-space occupancy. */
export type BayState = 'vacant' | 'occupied' | 'unknown';

const DATA_SOURCES: readonly DataSource[] = ['simulated', 'replay', 'live'];
const BAY_STATES: readonly BayState[] = ['vacant', 'occupied', 'unknown'];

/** Narrow an untrusted value (API payload, GeoJSON property) to a `DataSource`. */
export function isDataSource(value: unknown): value is DataSource {
  return typeof value === 'string' && (DATA_SOURCES as readonly string[]).includes(value);
}

/** Narrow an untrusted value to a `BayState`. */
export function isBayState(value: unknown): value is BayState {
  return typeof value === 'string' && (BAY_STATES as readonly string[]).includes(value);
}
