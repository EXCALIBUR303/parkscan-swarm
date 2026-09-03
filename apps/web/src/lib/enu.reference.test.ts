/**
 * Cross-language guard for the lot-metres <-> WGS84 projection.
 *
 * The projection is implemented twice, independently: here in TypeScript (`./enu.ts`)
 * and in Python (`packages/parkscan_core/src/parkscan_core/geo.py`). Nothing in either
 * codebase would fail if the two silently diverged — robots and detections would simply
 * be drawn in the wrong place on the map, consistently, with every test green.
 *
 * So both sides assert against one committed fixture: `sim/lot/enu_reference.json`.
 * The mirror of this test is `packages/parkscan_core/tests/test_enu_reference.py`.
 * If you change the projection, both tests fail until you regenerate the fixture.
 */

import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';

import { lngLatToLot, lotToLngLat, type LotOrigin } from './enu';

interface ReferencePoint {
  x_m: number;
  y_m: number;
  lon: number;
  lat: number;
}

interface Reference {
  origin: { lat: number; lon: number };
  tolerance_deg: number;
  points: ReferencePoint[];
}

const HERE = dirname(fileURLToPath(import.meta.url));
const REFERENCE_PATH = resolve(HERE, '..', '..', '..', '..', 'sim', 'lot', 'enu_reference.json');

const reference = JSON.parse(readFileSync(REFERENCE_PATH, 'utf8')) as Reference;
const origin: LotOrigin = { latDeg: reference.origin.lat, lonDeg: reference.origin.lon };

describe('cross-language projection fixture', () => {
  it('matches the TypeScript implementation at every reference point', () => {
    for (const point of reference.points) {
      const [lon, lat] = lotToLngLat({ x: point.x_m, y: point.y_m }, origin);
      expect(Math.abs(lon - point.lon)).toBeLessThanOrEqual(reference.tolerance_deg);
      expect(Math.abs(lat - point.lat)).toBeLessThanOrEqual(reference.tolerance_deg);
    }
  });

  it('round-trips every reference point back to lot metres', () => {
    for (const point of reference.points) {
      const back = lngLatToLot([point.lon, point.lat], origin);
      expect(Math.abs(back.x - point.x_m)).toBeLessThan(1e-6);
      expect(Math.abs(back.y - point.y_m)).toBeLessThan(1e-6);
    }
  });

  it('uses a synthetic origin, never a real address', () => {
    // A plausible land coordinate as the fallback would render simulated robots on top
    // of somewhere real when a lot file declares no origin. ADR-0004 forbids that.
    expect(reference.origin).toEqual({ lat: 0, lon: 0 });
  });
});
