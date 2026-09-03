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

/**
 * Every `robot_state` value, in the order docs/ARCHITECTURE.md §3 lists them.
 *
 * Exhaustive on purpose: an unknown state arriving over the WebSocket must be rejected
 * at the boundary rather than widened into the union to make a render succeed.
 */
export const ROBOT_STATES: readonly RobotState[] = [
  'idle',
  'assigned',
  'transit',
  'inspecting',
  'waiting_on_lease',
  'returning',
  'charging',
  'failed',
  'offline',
];

export const COORDINATOR_VIEWS: readonly CoordinatorView[] = [
  'online',
  'suspect',
  'offline',
  'stuck',
];

/** Narrow an untrusted value (API payload, GeoJSON property) to a `DataSource`. */
export function isDataSource(value: unknown): value is DataSource {
  return typeof value === 'string' && (DATA_SOURCES as readonly string[]).includes(value);
}

/** Narrow an untrusted value to a `BayState`. */
export function isBayState(value: unknown): value is BayState {
  return typeof value === 'string' && (BAY_STATES as readonly string[]).includes(value);
}

/** Narrow an untrusted WebSocket payload field to a `RobotState`. */
export function isRobotState(value: unknown): value is RobotState {
  return typeof value === 'string' && (ROBOT_STATES as readonly string[]).includes(value);
}

/** Narrow an untrusted WebSocket payload field to a `CoordinatorView`. */
export function isCoordinatorView(value: unknown): value is CoordinatorView {
  return typeof value === 'string' && (COORDINATOR_VIEWS as readonly string[]).includes(value);
}
