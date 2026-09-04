/**
 * Dashboard state.
 *
 * Shaped for 10 Hz telemetry from three robots, which is the only reason it is a store
 * and not React Context: at that rate a Context provider re-renders every consumer on
 * every pose update (docs/ARCHITECTURE.md §7).
 *
 * Two rules keep it fast, and both are tested rather than asserted:
 *
 * 1. **Keyed, not rebuilt.** The robots slice is a record keyed by robot id and updated
 *    key by key, so moving `robot_1` leaves the `robot_2` and `robot_3` objects
 *    reference-identical and their cards do not re-render. `useRobot(id)` relies on
 *    exactly that; anything selecting a derived collection goes through `useShallow`.
 *
 * 2. **One commit per frame.** `applyBatch` takes a whole `requestAnimationFrame` worth
 *    of decoded events and performs a single `set()`. The socket never calls
 *    `applyTelemetry` in a loop — see `src/lib/socket.ts`.
 *
 * Derived values are **selectors, not fields**. The battery band, for instance, is
 * computed from `batterySoc` on read; storing it would create a second definition of
 * "low" that can drift from the first.
 */

import { create } from 'zustand';
import { useShallow } from 'zustand/react/shallow';

import { batteryBand, type BatteryBand, type ConnectionStatus, type RobotView } from './lib/robot';
import type { DetectionView, LiveEvent } from './lib/socket';
import type { DataSource } from './lib/types';

export type { ConnectionStatus, RobotView } from './lib/robot';
export type { DetectionView } from './lib/socket';

/** Most detections held in memory. See `detections` for why this is bounded. */
export const MAX_DETECTIONS = 500;

export interface ParkscanState {
  /**
   * Provenance of everything on screen. The SIMULATED badge renders from this value —
   * it is never a hardcoded string (ACCEPTANCE H1).
   */
  dataSource: DataSource;
  connection: ConnectionStatus;
  lastError: string | null;
  /** Run the socket says it is streaming, from the server's `run_state` greeting. */
  runId: string | null;
  robots: Readonly<Record<string, RobotView>>;
  /**
   * Frames the server's bounded per-client queue discarded, cumulative.
   *
   * Reported by `{"type":"lag","dropped":N}` (docs/API_CONTRACT.md). Surfaced in the UI:
   * a lossy stream that looks lossless is worse than one that admits it.
   */
  droppedFrames: number;
  /** Frames this client could not decode, cumulative. Schema drift shows up here. */
  malformedFrames: number;
  /** Zone id -> coverage fraction 0..1, from `GET /api/v1/coverage`. */
  coverageByZone: Readonly<Record<string, number>>;
  /**
   * Detections, newest last, capped at MAX_DETECTIONS.
   *
   * A cap rather than unbounded growth: a long run emits thousands, and a dashboard that
   * degrades over an afternoon is a dashboard nobody leaves open. The cap is stated in
   * the UI so the list never silently claims to be complete.
   */
  detections: readonly DetectionView[];

  setDataSource: (dataSource: DataSource) => void;
  setConnection: (connection: ConnectionStatus, error?: string | null) => void;
  /** Upsert one robot. Other robots keep their object identity. */
  applyTelemetry: (robot: RobotView) => void;
  /** Apply one animation frame's worth of socket events in a single commit. */
  applyBatch: (batch: readonly LiveEvent[]) => void;
  applyCoverage: (coverageByZone: Readonly<Record<string, number>>) => void;
  applyDetection: (detection: DetectionView) => void;
  removeRobot: (id: string) => void;
  reset: () => void;
}

const INITIAL = {
  dataSource: 'simulated' as DataSource,
  connection: 'idle' as ConnectionStatus,
  lastError: null,
  runId: null,
  robots: {} as Readonly<Record<string, RobotView>>,
  droppedFrames: 0,
  malformedFrames: 0,
  coverageByZone: {} as Readonly<Record<string, number>>,
  detections: [] as readonly DetectionView[],
};

export const useParkscanStore = create<ParkscanState>()((set) => ({
  ...INITIAL,

  setDataSource: (dataSource) => {
    set({ dataSource });
  },

  setConnection: (connection, error = null) => {
    set({ connection, lastError: error });
  },

  applyTelemetry: (robot) => {
    set((state) => ({
      robots: { ...state.robots, [robot.id]: robot },
      dataSource: robot.dataSource,
    }));
  },

  applyBatch: (batch) => {
    if (batch.length === 0) return;
    set((state) => {
      let robots: Record<string, RobotView> | null = null;
      let dataSource = state.dataSource;
      let runId = state.runId;
      let dropped = state.droppedFrames;
      let malformed = state.malformedFrames;
      let lastError = state.lastError;
      let detections: DetectionView[] | null = null;
      let touched = false;

      for (const event of batch) {
        switch (event.kind) {
          case 'telemetry':
            robots ??= { ...state.robots };
            robots[event.robot.id] = event.robot;
            dataSource = event.robot.dataSource;
            touched = true;
            break;
          case 'detection':
            detections ??= [...state.detections];
            detections.push(event.detection);
            touched = true;
            break;
          case 'run_state':
            runId = event.runId;
            dataSource = event.dataSource;
            touched = true;
            break;
          case 'lag':
            dropped += event.dropped;
            touched = true;
            break;
          case 'malformed':
            malformed += 1;
            touched = true;
            break;
          case 'server_error':
            lastError = event.detail;
            touched = true;
            break;
        }
      }

      if (!touched) return state;
      return {
        robots: robots ?? state.robots,
        dataSource,
        runId,
        droppedFrames: dropped,
        malformedFrames: malformed,
        lastError,
        detections:
          detections === null
            ? state.detections
            : detections.length > MAX_DETECTIONS
              ? detections.slice(-MAX_DETECTIONS)
              : detections,
      };
    });
  },

  applyDetection: (detection) => {
    set((state) => {
      const next = [...state.detections, detection];
      return {
        detections: next.length > MAX_DETECTIONS ? next.slice(-MAX_DETECTIONS) : next,
        dataSource: state.dataSource,
      };
    });
  },

  applyCoverage: (coverageByZone) => {
    set({ coverageByZone: { ...coverageByZone } });
  },

  removeRobot: (id) => {
    set((state) => {
      if (!(id in state.robots)) return state;
      const next = { ...state.robots };
      delete next[id];
      return { robots: next };
    });
  },

  reset: () => {
    set({ ...INITIAL });
  },
}));

/* ------------------------------------------------------------------ selectors -- */

export function useDataSource(): DataSource {
  return useParkscanStore((s) => s.dataSource);
}

export function useConnection(): { status: ConnectionStatus; error: string | null } {
  return useParkscanStore(useShallow((s) => ({ status: s.connection, error: s.lastError })));
}

/** Cumulative stream losses. Shallow-compared so it does not re-render on every pose. */
export function useStreamLoss(): { dropped: number; malformed: number } {
  return useParkscanStore(
    useShallow((s) => ({ dropped: s.droppedFrames, malformed: s.malformedFrames })),
  );
}

export function useRunId(): string | null {
  return useParkscanStore((s) => s.runId);
}

/** Stable, sorted robot ids. `useShallow` keeps the array identity from thrashing. */
export function useRobotIds(): string[] {
  return useParkscanStore(useShallow((s) => Object.keys(s.robots).sort()));
}

/**
 * One robot. Returns the stored object by reference, so a component subscribed to
 * `robot_2` does not re-render when `robot_1` moves.
 */
export function useRobot(id: string): RobotView | undefined {
  return useParkscanStore((s) => s.robots[id]);
}

/**
 * One robot's battery band.
 *
 * Derived on read, deliberately: the band is a display grouping of `batterySoc`, and a
 * component subscribed to it re-renders only when the *band* changes, not on every
 * percent. Storing it would duplicate the definition of "low".
 */
export function useBatteryBand(id: string): BatteryBand | undefined {
  return useParkscanStore((s) => {
    const robot = s.robots[id];
    return robot === undefined ? undefined : batteryBand(robot.batterySoc);
  });
}

export function useCoverageByZone(): Readonly<Record<string, number>> {
  return useParkscanStore((s) => s.coverageByZone);
}

/** Every robot, for the accessible data table. Shallow-compared, so poses still batch. */
export function useRobotList(): RobotView[] {
  return useParkscanStore(
    useShallow((s) =>
      Object.keys(s.robots)
        .sort()
        .map((id) => s.robots[id])
        .filter((r): r is RobotView => r !== undefined),
    ),
  );
}

/** Detections held in memory, newest last. */
export function useDetections(): readonly DetectionView[] {
  return useParkscanStore((s) => s.detections);
}

/**
 * Only the detections that have a world position.
 *
 * A rejected geolocation has no point (criterion C11), so it is listed in the panel but
 * never plotted — a pin at a made-up location would be worse than no pin at all.
 */
export function usePlottableDetections(): readonly DetectionView[] {
  return useParkscanStore(useShallow((s) => s.detections.filter((d) => d.position !== null)));
}
