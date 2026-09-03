/**
 * Dashboard state.
 *
 * Shaped for 10 Hz telemetry from three robots, which is the only reason it is a store
 * and not React Context: at that rate a Context provider re-renders every consumer on
 * every pose update (docs/ARCHITECTURE.md §7).
 *
 * The robots slice is keyed by robot id and updated key-by-key, so moving `robot_1`
 * leaves the `robot_2` and `robot_3` objects reference-identical and their cards do not
 * re-render. `useRobot(id)` relies on exactly that; anything selecting a derived
 * collection must go through `useShallow`.
 *
 * Empty until Phase 2 wires MQTT -> FastAPI -> WebSocket.
 */

import { create } from 'zustand';
import { useShallow } from 'zustand/react/shallow';

import type { LotPoint } from './lib/enu';
import type { CoordinatorView, DataSource, RobotState } from './lib/types';

/** One robot as the dashboard sees it. Positions are lot metres, never lng/lat. */
export interface RobotView {
  readonly id: string;
  /** Self-reported. */
  readonly state: RobotState;
  /** The coordinator's independent judgement — deliberately not merged with `state`. */
  readonly coordinatorView: CoordinatorView;
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

export type ConnectionStatus = 'idle' | 'connecting' | 'open' | 'reconnecting' | 'closed' | 'error';

export interface ParkscanState {
  /**
   * Provenance of everything on screen. The SIMULATED badge renders from this value —
   * it is never a hardcoded string (ACCEPTANCE H1).
   */
  dataSource: DataSource;
  connection: ConnectionStatus;
  lastError: string | null;
  robots: Readonly<Record<string, RobotView>>;
  /** Zone id -> coverage fraction 0..1, from `GET /api/v1/coverage`. */
  coverageByZone: Readonly<Record<string, number>>;

  setDataSource: (dataSource: DataSource) => void;
  setConnection: (connection: ConnectionStatus, error?: string | null) => void;
  /** Upsert one robot. Other robots keep their object identity. */
  applyTelemetry: (robot: RobotView) => void;
  applyCoverage: (coverageByZone: Readonly<Record<string, number>>) => void;
  removeRobot: (id: string) => void;
  reset: () => void;
}

const INITIAL = {
  dataSource: 'simulated' as DataSource,
  connection: 'idle' as ConnectionStatus,
  lastError: null,
  robots: {} as Readonly<Record<string, RobotView>>,
  coverageByZone: {} as Readonly<Record<string, number>>,
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
