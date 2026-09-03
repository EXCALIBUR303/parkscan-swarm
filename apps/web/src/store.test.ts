import { beforeEach, describe, expect, it } from 'vitest';

import { useParkscanStore, type RobotView } from './store';

function robot(id: string, x: number): RobotView {
  return {
    id,
    state: 'transit',
    coordinatorView: 'online',
    batterySoc: 0.8,
    position: { x, y: 10 },
    headingRad: 0,
    zoneId: 'zone_01',
    epoch: 1,
    stampMs: 1_700_000_000_000,
    dataSource: 'simulated',
  };
}

beforeEach(() => {
  useParkscanStore.getState().reset();
});

describe('robots slice', () => {
  it('starts empty and simulated', () => {
    const state = useParkscanStore.getState();
    expect(state.robots).toEqual({});
    expect(state.dataSource).toBe('simulated');
    expect(state.connection).toBe('idle');
  });

  it('keeps other robots reference-identical when one moves', () => {
    const { applyTelemetry } = useParkscanStore.getState();
    applyTelemetry(robot('robot_1', 1));
    applyTelemetry(robot('robot_2', 2));

    const before = useParkscanStore.getState().robots['robot_2'];
    applyTelemetry(robot('robot_1', 99));
    const after = useParkscanStore.getState().robots['robot_2'];

    // This is what stops robot_1 moving at 10 Hz from re-rendering robot_2's card.
    expect(after).toBe(before);
    expect(useParkscanStore.getState().robots['robot_1']?.position.x).toBe(99);
  });

  it('takes provenance from the telemetry rather than assuming it', () => {
    useParkscanStore.getState().applyTelemetry({ ...robot('robot_1', 0), dataSource: 'replay' });
    expect(useParkscanStore.getState().dataSource).toBe('replay');
  });

  it('removes a robot without disturbing the others', () => {
    const { applyTelemetry, removeRobot } = useParkscanStore.getState();
    applyTelemetry(robot('robot_1', 1));
    applyTelemetry(robot('robot_2', 2));
    const kept = useParkscanStore.getState().robots['robot_2'];

    removeRobot('robot_1');
    expect(useParkscanStore.getState().robots['robot_1']).toBeUndefined();
    expect(useParkscanStore.getState().robots['robot_2']).toBe(kept);
  });

  it('is a no-op when removing an unknown robot', () => {
    const before = useParkscanStore.getState().robots;
    useParkscanStore.getState().removeRobot('nope');
    expect(useParkscanStore.getState().robots).toBe(before);
  });
});

describe('coverage and connection slices', () => {
  it('stores coverage by zone', () => {
    useParkscanStore.getState().applyCoverage({ zone_01: 0.5 });
    expect(useParkscanStore.getState().coverageByZone).toEqual({ zone_01: 0.5 });
  });

  it('carries an error alongside the connection status', () => {
    useParkscanStore.getState().setConnection('error', 'ECONNREFUSED');
    expect(useParkscanStore.getState().connection).toBe('error');
    expect(useParkscanStore.getState().lastError).toBe('ECONNREFUSED');

    useParkscanStore.getState().setConnection('open');
    expect(useParkscanStore.getState().lastError).toBeNull();
  });
});
