/**
 * The words the provenance banner uses, keyed by `data_source`.
 *
 * Kept out of the component so the copy can be imported by tests, the document title and
 * (later) the live-region announcer without any of them reaching for a literal. If this
 * table is ever bypassed, ACCEPTANCE H1 is broken: the badge must say what the data says.
 */

import type { DataSource } from './types';

export const APP_NAME = 'ParkScan Swarm';

export interface BadgeCopy {
  /** Short form, used in `document.title`. */
  readonly short: string;
  /** The banner text. */
  readonly full: string;
  /** Longer explanation, rendered next to the banner text. */
  readonly detail: string;
}

export const BADGE_COPY: Record<DataSource, BadgeCopy> = {
  simulated: {
    short: 'SIMULATED',
    full: 'SIMULATED DATA — no physical robots',
    detail:
      'Every position, detection and battery reading on this page was produced by a ' +
      'deterministic software simulator. No hardware exists.',
  },
  replay: {
    short: 'REPLAY',
    full: 'REPLAY DATA — recorded run, not live',
    detail: 'This page is replaying a stored run. Nothing shown is happening now.',
  },
  live: {
    short: 'LIVE',
    full: 'LIVE DATA — hardware in the loop',
    detail: 'This page is showing telemetry from physical hardware.',
  },
};
