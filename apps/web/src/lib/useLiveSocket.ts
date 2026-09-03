import { useEffect } from 'react';

import { LiveSocket, type LiveSocketOptions } from './socket';
import { useParkscanStore } from '../store';

export interface UseLiveSocketOptions extends LiveSocketOptions {
  /** Set false to mount the dashboard without opening a socket. */
  readonly enabled?: boolean;
}

/**
 * Open `/ws/live` for the lifetime of the component and pump it into the store.
 *
 * The store is reached through `getState()` rather than through a subscription: the
 * callbacks only ever *write*, so subscribing would re-create the socket on every
 * telemetry frame. Each batch is one `applyBatch` call, which is one commit.
 *
 * The effect deliberately depends on primitives, not on the options object: a caller
 * that passes a fresh object literal every render must not tear the socket down and
 * rebuild it every render. Pass module-level constants for `topics` / `clock` if you
 * need to override them.
 */
export function useLiveSocket(options: UseLiveSocketOptions = {}): void {
  const {
    enabled = true,
    url,
    topics,
    clock,
    webSocketImpl,
    now,
    minReconnectionDelayMs,
    maxReconnectionDelayMs,
    connectionTimeoutMs,
  } = options;

  useEffect(() => {
    if (!enabled) return;

    const socket = new LiveSocket(
      {
        onBatch: (batch) => {
          useParkscanStore.getState().applyBatch(batch);
        },
        onStatus: (status, error) => {
          useParkscanStore.getState().setConnection(status, error);
        },
      },
      {
        url,
        topics,
        clock,
        webSocketImpl,
        now,
        minReconnectionDelayMs,
        maxReconnectionDelayMs,
        connectionTimeoutMs,
      },
    );

    return () => {
      socket.close();
    };
  }, [
    enabled,
    url,
    topics,
    clock,
    webSocketImpl,
    now,
    minReconnectionDelayMs,
    maxReconnectionDelayMs,
    connectionTimeoutMs,
  ]);
}
