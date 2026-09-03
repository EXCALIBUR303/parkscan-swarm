/**
 * What the polite live region is allowed to say.
 *
 * THE RULE THIS FILE EXISTS TO ENFORCE
 * ------------------------------------
 * Telemetry arrives at up to 10 Hz per robot. A live region wired to that stream reads
 * a new pose to a screen-reader user thirty times a second, which does not inform
 * anyone — it makes the page unusable and is worse than having no live region at all
 * (docs/ARCHITECTURE.md section 7: "one *polite* live region for state **changes** only,
 * never pose").
 *
 * So the announcer never sees a pose. It subscribes to an encoded **signal key** per
 * robot — id, self-reported state, coordinator view, battery *band* — and a pose update
 * produces a byte-identical key, which `useShallow` then discards without a render. The
 * only way a pose can reach the live region is if someone adds a coordinate to
 * `fleetSignalKey`, and the test suite fails when they do.
 *
 * Changes are then **debounced and coalesced**: everything that happened inside one
 * window becomes a single sentence, so three robots changing state at once is one
 * announcement rather than three interruptions.
 */

import { useCallback, useEffect, useRef, useState } from 'react';

import { capitalise, humaniseId, isAlarmingBand, type ConnectionStatus } from './robot';

/**
 * Field separator inside a signal key.
 *
 * A robot id containing this character would merge two fields, which can only ever cost
 * a spurious or missing announcement — never a crash and never a wrong pose, because no
 * pose is in the key to begin with.
 */
const SEP = '|';

/** How long changes are gathered before one coalesced announcement is published. */
export const ANNOUNCE_DEBOUNCE_MS = 600;

/**
 * The only facts the announcer is allowed to know about a robot.
 *
 * Note what is absent: `position`, `headingRad`, `batterySoc`, `stampMs`, `epoch`. Those
 * change continuously, and none of them belongs in a live region.
 */
export interface AnnouncerInput {
  readonly id: string;
  readonly state: string;
  readonly coordinatorView: string | null;
  readonly batteryBand: string;
}

export function fleetSignalKey(input: AnnouncerInput): string {
  return [input.id, input.state, input.coordinatorView ?? '-', input.batteryBand].join(SEP);
}

interface Signal {
  readonly id: string;
  readonly state: string;
  readonly coordinatorView: string;
  readonly band: string;
}

function parseSignalKey(key: string): Signal {
  const [id = '', state = '', coordinatorView = '-', band = ''] = key.split(SEP);
  return { id, state, coordinatorView, band };
}

function name(id: string): string {
  return capitalise(humaniseId(id));
}

function readable(value: string): string {
  return value.replace(/_/g, ' ');
}

/**
 * Sentences describing how the fleet changed between two sorted key lists.
 *
 * Pure and total: identical inputs produce an empty array, which is what makes "a pose
 * update announces nothing" a testable claim rather than a hope.
 */
export function diffFleetKeys(previous: readonly string[], next: readonly string[]): string[] {
  const before = new Map(previous.map(parseSignalKey).map((s) => [s.id, s] as const));
  const after = new Map(next.map(parseSignalKey).map((s) => [s.id, s] as const));
  const messages: string[] = [];

  for (const [id, signal] of after) {
    const prior = before.get(id);

    if (prior === undefined) {
      messages.push(`${name(id)} is reporting, ${readable(signal.state)}.`);
      continue;
    }

    if (prior.state !== signal.state) {
      messages.push(
        `${name(id)} changed from ${readable(prior.state)} to ${readable(signal.state)}.`,
      );
    }

    if (prior.coordinatorView !== signal.coordinatorView && signal.coordinatorView !== '-') {
      messages.push(`Coordinator now judges ${name(id)} ${readable(signal.coordinatorView)}.`);
    }

    if (prior.band !== signal.band) {
      // Only the bands that matter. An ok/fair wobble around 50% is not news.
      if (isAlarmingBand(signal.band)) {
        messages.push(`${name(id)} battery is ${signal.band}.`);
      } else if (isAlarmingBand(prior.band)) {
        messages.push(`${name(id)} battery recovered to ${signal.band}.`);
      }
    }
  }

  for (const id of before.keys()) {
    if (!after.has(id)) messages.push(`${name(id)} stopped reporting.`);
  }

  return messages;
}

/**
 * The sentence for a connection transition, or `null` when there is nothing to say.
 *
 * `connecting` is silent on purpose: it is transient, and announcing it on every backoff
 * attempt would produce exactly the flood this module exists to prevent.
 */
export function connectionAnnouncement(
  status: ConnectionStatus,
  error: string | null,
): string | null {
  switch (status) {
    case 'open':
      return 'Telemetry connection open.';
    case 'reconnecting':
      return 'Telemetry connection lost. Reconnecting.';
    case 'closed':
      return 'Telemetry connection closed.';
    case 'error':
      return `Telemetry connection error: ${error ?? 'unknown'}.`;
    case 'idle':
    case 'connecting':
      return null;
  }
}

export interface AnnouncementQueue {
  readonly message: string;
  readonly push: (messages: readonly string[]) => void;
}

/**
 * Collect messages and publish them as one string per debounce window.
 *
 * The queue itself is a ref, not state, so appending to it never causes a render — only
 * publishing does.
 */
export function useAnnouncementQueue(debounceMs: number = ANNOUNCE_DEBOUNCE_MS): AnnouncementQueue {
  const [message, setMessage] = useState('');
  const pending = useRef<string[]>([]);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);

  const push = useCallback(
    (messages: readonly string[]) => {
      if (messages.length === 0) return;
      pending.current.push(...messages);
      if (timer.current !== null) return;
      timer.current = setTimeout(() => {
        timer.current = null;
        const batch = pending.current;
        pending.current = [];
        if (batch.length > 0) setMessage(batch.join(' '));
      }, debounceMs);
    },
    [debounceMs],
  );

  useEffect(
    () => () => {
      if (timer.current !== null) clearTimeout(timer.current);
    },
    [],
  );

  return { message, push };
}
