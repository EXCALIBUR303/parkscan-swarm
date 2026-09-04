import { useEffect } from 'react';

import { useParkscanStore } from '../store';

/** How often to re-read coverage, in milliseconds. */
const POLL_MS = 3000;

interface CoverageRow {
  zone_id: string;
  coverage: number;
}

/**
 * Poll `GET /api/v1/coverage` into the store.
 *
 * Polled rather than streamed on purpose. Coverage is a slow, monotonic aggregate — a
 * zone gains a few percent every several seconds — and pushing it down the telemetry
 * socket would put a whole-lot recomputation on the 10 Hz path to save a request every
 * three seconds. The socket carries what changes fast; this carries what changes slowly.
 *
 * A failed poll is left to the next tick rather than surfaced: the choropleth simply keeps
 * its previous colours, which is the correct degradation for an aggregate that is already
 * a few seconds stale by design. Connection health is reported from the socket, which is
 * the thing whose liveness actually matters.
 */
export function useCoverage(enabled = true): void {
  const applyCoverage = useParkscanStore((s) => s.applyCoverage);

  useEffect(() => {
    if (!enabled) return;
    let cancelled = false;

    const poll = async (): Promise<void> => {
      try {
        const response = await fetch('/api/v1/coverage');
        if (!response.ok) return;
        const rows = (await response.json()) as CoverageRow[];
        if (cancelled) return;
        const byZone: Record<string, number> = {};
        for (const row of rows) byZone[row.zone_id] = row.coverage;
        applyCoverage(byZone);
      } catch {
        // Left for the next tick — see the note above.
      }
    };

    void poll();
    const timer = window.setInterval(() => void poll(), POLL_MS);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, [applyCoverage, enabled]);
}
