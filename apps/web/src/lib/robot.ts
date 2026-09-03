/**
 * The robot view model and the words the UI uses for it.
 *
 * Kept out of `store.ts` so that the wire decoder (`socket.ts`) and the store can both
 * speak this shape without importing each other. Everything here is pure: no React, no
 * store, no I/O — which is what lets the label strings be asserted directly in tests.
 *
 * Positions are **lot metres** throughout. Longitude/latitude exists only inside
 * `LotMap`, at the `enu.ts` boundary (ADR-0004).
 */

import type { LotPoint } from './enu';
import type { CoordinatorView, DataSource, RobotState } from './types';

/** One robot as the dashboard sees it. */
export interface RobotView {
  readonly id: string;
  /** Self-reported. */
  readonly state: RobotState;
  /**
   * The coordinator's independent judgement — deliberately not merged with `state`.
   *
   * `null` means the coordinator has not published a judgement for this robot yet.
   * That is not the same as `online`, and the UI must not render it as if it were:
   * `coordinator_view` is nullable on the wire (`TelemetryMessage.coordinator_view`)
   * and claiming "online" for an absent judgement would invent a fact.
   */
  readonly coordinatorView: CoordinatorView | null;
  /** State of charge, 0..1. */
  readonly batterySoc: number;
  readonly position: LotPoint;
  readonly headingRad: number;
  readonly zoneId: string | null;
  readonly epoch: number;
  /** Telemetry stamp, epoch milliseconds. */
  readonly stampMs: number;
  readonly dataSource: DataSource;
}

/**
 * Liveness of the `/ws/live` connection, as the UI needs to talk about it.
 *
 * `reconnecting` is separate from `error` on purpose: partysocket retries with backoff
 * after a drop, and a dashboard that says "error" while it is quietly recovering trains
 * people to ignore it.
 */
export type ConnectionStatus = 'idle' | 'connecting' | 'open' | 'reconnecting' | 'closed' | 'error';

export const CONNECTION_LABEL: Record<ConnectionStatus, string> = {
  idle: 'not connected',
  connecting: 'connecting',
  open: 'live',
  reconnecting: 'reconnecting',
  closed: 'closed',
  error: 'error',
};

/* ------------------------------------------------------------------- battery -- */

/**
 * Display bands for state of charge.
 *
 * **These are labels, not policy.** The return-to-charge decision is a feasibility
 * check in the coordinator — `soc < k * d_travel(pos, nearest_bay) * (1 + margin) +
 * soc_reserve` (docs/ARCHITECTURE.md §3) — and it is deliberately *not* a fixed
 * percentage, because a fixed threshold is wasteful near the bays and fatal far from
 * them. Nothing on this screen should be read as the trigger the robot actually uses.
 *
 * Derived, never stored: the store keeps `batterySoc` and the band is computed by a
 * selector, so there is exactly one definition of "low".
 */
export type BatteryBand = 'ok' | 'fair' | 'low' | 'critical';

/** Upper bound (exclusive) of each band, in state of charge. */
export const BATTERY_BREAKS = {
  critical: 0.1,
  low: 0.2,
  fair: 0.5,
} as const;

export function batteryBand(soc: number): BatteryBand {
  if (!Number.isFinite(soc)) return 'critical';
  if (soc < BATTERY_BREAKS.critical) return 'critical';
  if (soc < BATTERY_BREAKS.low) return 'low';
  if (soc < BATTERY_BREAKS.fair) return 'fair';
  return 'ok';
}

/**
 * Bands worth announcing. Takes a plain string because the announcer works on encoded
 * signal keys, where the band has already been flattened to text.
 */
export function isAlarmingBand(band: string): boolean {
  return band === 'low' || band === 'critical';
}

export function batteryPercent(soc: number): number {
  return Math.round(soc * 100);
}

/* -------------------------------------------------------------------- naming -- */

/**
 * `robot_1` -> `robot 1`, `zone_05` -> `zone 5`.
 *
 * Zero padding is stripped so a screen reader says "zone five" rather than "zone zero
 * five", and underscores become spaces so it does not spell the identifier out.
 * Anything that is not in `word_number` shape is passed through with underscores
 * replaced — never dropped, because an unrecognised id is still the robot's real name.
 */
export function humaniseId(id: string): string {
  return id
    .split('_')
    .map((part) => (/^\d+$/.test(part) ? String(Number(part)) : part))
    .join(' ');
}

export function capitalise(text: string): string {
  return text.charAt(0).toUpperCase() + text.slice(1);
}

/**
 * The accessible name of a robot marker and the text of its card heading.
 *
 * Example: `Robot 1, zone 5, battery 63 percent, inspecting`.
 *
 * The coordinator's view is appended only when it exists and disagrees with `online` —
 * that disagreement (a robot that believes it is inspecting while the coordinator has
 * declared it offline) is exactly the case the recovery logic exists for, so it must be
 * audible, while repeating "coordinator view online" three times a second is noise.
 */
export function describeRobot(robot: RobotView): string {
  const zone = robot.zoneId === null ? 'no zone' : humaniseId(robot.zoneId);
  const parts = [
    capitalise(humaniseId(robot.id)),
    zone,
    `battery ${String(batteryPercent(robot.batterySoc))} percent`,
    robot.state.replace(/_/g, ' '),
  ];
  if (robot.coordinatorView !== null && robot.coordinatorView !== 'online') {
    parts.push(`coordinator view ${robot.coordinatorView}`);
  }
  return parts.join(', ');
}
