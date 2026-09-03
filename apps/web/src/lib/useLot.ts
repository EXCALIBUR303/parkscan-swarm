import { useCallback, useEffect, useState } from 'react';

import { loadLot, type LoadedLot } from './lot';
import { useParkscanStore } from '../store';

/**
 * Fetch `lot.geojson` once, with explicit loading / error / ready states.
 *
 * On success the lot's declared provenance is pushed into the store, which is what the
 * SIMULATED badge renders from. That is the Phase 1 shape of ACCEPTANCE H1: the badge
 * already reads a `data_source` that travelled with the data, and when the API is wired
 * in Phase 2 the same store field is written by telemetry instead.
 *
 * `reload()` — not the effect — is what resets the state to `loading`, so nothing here
 * calls `setState` synchronously inside an effect body. `url` is expected to be constant
 * for the lifetime of the hook; if it does change, the previously loaded lot stays on
 * screen until the new one arrives rather than flashing an empty map.
 */

export type LotStatus = 'loading' | 'ready' | 'error';

export interface LotResult {
  status: LotStatus;
  lot: LoadedLot | null;
  error: string | null;
  reload: () => void;
}

interface LotFetchState {
  attempt: number;
  status: LotStatus;
  lot: LoadedLot | null;
  error: string | null;
}

const INITIAL: LotFetchState = { attempt: 0, status: 'loading', lot: null, error: null };

export function useLot(url = '/lot.geojson'): LotResult {
  const [state, setState] = useState<LotFetchState>(INITIAL);
  const setDataSource = useParkscanStore((s) => s.setDataSource);
  const attempt = state.attempt;

  const reload = useCallback(() => {
    setState((prev) => ({ attempt: prev.attempt + 1, status: 'loading', lot: null, error: null }));
  }, []);

  useEffect(() => {
    const controller = new AbortController();

    loadLot(url, controller.signal)
      .then((loaded) => {
        if (controller.signal.aborted) return;
        setDataSource(loaded.dataSource);
        setState((prev) => ({ ...prev, status: 'ready', lot: loaded, error: null }));
      })
      .catch((cause: unknown) => {
        // An abort is this effect being cleaned up, not a failure worth showing.
        if (controller.signal.aborted) return;
        setState((prev) => ({
          ...prev,
          status: 'error',
          error: cause instanceof Error ? cause.message : String(cause),
        }));
      });

    return () => {
      controller.abort();
    };
  }, [url, attempt, setDataSource]);

  return { status: state.status, lot: state.lot, error: state.error, reload };
}
