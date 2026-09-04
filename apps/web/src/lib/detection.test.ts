import { describe, expect, it } from 'vitest';

import { parseLiveMessage } from './socket';
import { MAX_DETECTIONS, useParkscanStore } from '../store';

function frame(overrides: Record<string, unknown> = {}): string {
  return JSON.stringify({
    type: 'detection',
    stamp: '2026-09-04T09:00:00Z',
    data_source: 'simulated',
    payload: {
      id: 'det-1',
      robot_id: 'robot_1',
      kind: 'pothole',
      det_source: 'simulated',
      detector_name: 'SimulatedDetector',
      confidence: 0.82,
      severity: 'high',
      local_x: 16.8,
      local_y: 5.3,
      range_m: 3.3,
      geoloc_valid: true,
      ...overrides,
    },
  });
}

const now = (): number => 1_000;

describe('detection decoding', () => {
  it('decodes a geolocated detection into a plottable view', () => {
    const event = parseLiveMessage(frame(), now);
    expect(event.kind).toBe('detection');
    if (event.kind !== 'detection') return;
    expect(event.detection.position).toEqual({ x: 16.8, y: 5.3 });
    expect(event.detection.severity).toBe('high');
    // Provenance comes off the wire, never from a literal in the UI (ACCEPTANCE H1).
    expect(event.detection.detSource).toBe('simulated');
  });

  it('gives a REJECTED geolocation no position at all', () => {
    // C11: a rejected ray has no world point. Snapping it to the robot, or to the last
    // known good fix, would put a confident pothole somewhere it is not.
    const event = parseLiveMessage(
      frame({ geoloc_valid: false, local_x: null, local_y: null }),
      now,
    );
    if (event.kind !== 'detection') throw new Error('expected a detection');
    expect(event.detection.position).toBeNull();
    expect(event.detection.geolocValid).toBe(false);
  });

  it('does not invent a position when the server sends one on an invalid fix', () => {
    // Defensive: even if a buggy producer contradicts itself, the client refuses to plot.
    const event = parseLiveMessage(frame({ geoloc_valid: false }), now);
    if (event.kind !== 'detection') throw new Error('expected a detection');
    expect(event.detection.position).toBeNull();
  });

  it('treats an unknown severity as unlabelled rather than guessing', () => {
    const event = parseLiveMessage(frame({ severity: 'catastrophic' }), now);
    if (event.kind !== 'detection') throw new Error('expected a detection');
    expect(event.detection.severity).toBeNull();
  });

  it('rejects a detection missing its identity fields', () => {
    const event = parseLiveMessage(frame({ id: null }), now);
    expect(event.kind).toBe('malformed');
  });
});

describe('detection store', () => {
  it('keeps detections in arrival order and bounds the list', () => {
    useParkscanStore.getState().reset();
    const total = MAX_DETECTIONS + 25;
    for (let i = 0; i < total; i += 1) {
      const event = parseLiveMessage(frame({ id: `det-${i}` }), now);
      if (event.kind !== 'detection') throw new Error('expected a detection');
      useParkscanStore.getState().applyDetection(event.detection);
    }
    const kept = useParkscanStore.getState().detections;
    // Bounded, and it is the OLDEST that are dropped — a live dashboard wants the recent ones.
    expect(kept).toHaveLength(MAX_DETECTIONS);
    expect(kept[kept.length - 1]?.id).toBe(`det-${total - 1}`);
    expect(kept[0]?.id).toBe(`det-${total - MAX_DETECTIONS}`);
  });
});
