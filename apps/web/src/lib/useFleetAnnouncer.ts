import { useEffect, useRef } from 'react';
import { useShallow } from 'zustand/react/shallow';

import {
  connectionAnnouncement,
  diffFleetKeys,
  fleetSignalKey,
  useAnnouncementQueue,
} from './announce';
import { batteryBand, type ConnectionStatus } from './robot';
import { useParkscanStore } from '../store';

/**
 * The text of the fleet live region.
 *
 * The selector below is the load-bearing part. It maps each robot to a signal key built
 * from **state, coordinator view and battery band only**, and `useShallow` compares the
 * resulting array element by element. A pose update therefore produces an identical
 * array and this hook does not even re-render, let alone announce — which is the whole
 * requirement (docs/ARCHITECTURE.md section 7: state changes only, never pose).
 *
 * Connection transitions are announced from a second, separate effect so that "the
 * stream dropped" is not silently coalesced away with robot news it has nothing to do
 * with — though both do share the one debounce window, because they share one region.
 */
export function useFleetAnnouncer(debounceMs?: number): string {
  const keys = useParkscanStore(
    useShallow((s) =>
      Object.keys(s.robots)
        .sort()
        .flatMap((id) => {
          const robot = s.robots[id];
          return robot === undefined
            ? []
            : [
                fleetSignalKey({
                  id: robot.id,
                  state: robot.state,
                  coordinatorView: robot.coordinatorView,
                  batteryBand: batteryBand(robot.batterySoc),
                }),
              ];
        }),
    ),
  );
  const status = useParkscanStore((s) => s.connection);
  const error = useParkscanStore((s) => s.lastError);

  const { message, push } = useAnnouncementQueue(debounceMs);
  const previousKeys = useRef<readonly string[]>([]);
  const previousStatus = useRef<ConnectionStatus | null>(null);

  useEffect(() => {
    push(diffFleetKeys(previousKeys.current, keys));
    previousKeys.current = keys;
  }, [keys, push]);

  useEffect(() => {
    const prior = previousStatus.current;
    previousStatus.current = status;
    // The first observed status is the mount value, not a transition.
    if (prior === null || prior === status) return;
    const sentence = connectionAnnouncement(status, error);
    if (sentence !== null) push([sentence]);
  }, [status, error, push]);

  return message;
}
