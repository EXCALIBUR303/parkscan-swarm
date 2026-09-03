import { describe, expect, it } from 'vitest';

// Vite serves the file as a raw string, so the test reads exactly the bytes the browser
// would fetch — no separate fixture that could drift from the generator's output.
import lotGeoJsonText from '../../public/lot.geojson?raw';

import { DEFAULT_LOT_ORIGIN, lngLatToLot, lotToLngLat, originScale, type LotOrigin } from './enu';

/** ACCEPTANCE C10: lot metres -> WGS84 -> lot metres, within 1 mm. */
const ONE_MM = 1e-3;

function gridPoints(): { x: number; y: number }[] {
  const points: { x: number; y: number }[] = [];
  for (let x = -20; x <= 170; x += 10) {
    for (let y = -20; y <= 120; y += 10) {
      points.push({ x, y });
    }
  }
  // Awkward values as well as round ones.
  points.push({ x: 0, y: 0 }, { x: 149.999, y: 99.999 }, { x: 0.001, y: -0.001 });
  return points;
}

describe('ENU round trip', () => {
  it('is within 1 mm over a grid covering the lot (C10)', () => {
    let worst = 0;
    for (const origin of [
      DEFAULT_LOT_ORIGIN,
      { latDeg: 0, lonDeg: 0 },
      { latDeg: 60.2, lonDeg: 24.9 },
    ]) {
      for (const point of gridPoints()) {
        const back = lngLatToLot(lotToLngLat(point, origin), origin);
        worst = Math.max(worst, Math.abs(back.x - point.x), Math.abs(back.y - point.y));
      }
    }
    expect(worst).toBeLessThan(ONE_MM);
    // In practice this is IEEE-754 noise, not an approximation error.
    expect(worst).toBeLessThan(1e-6);
  });

  it('places the lot origin exactly at the geodetic origin', () => {
    expect(lotToLngLat({ x: 0, y: 0 }, DEFAULT_LOT_ORIGIN)).toEqual([
      DEFAULT_LOT_ORIGIN.lonDeg,
      DEFAULT_LOT_ORIGIN.latDeg,
    ]);
  });

  it('uses REP-103 axes: +x is east, +y is north', () => {
    const [eastLng, eastLat] = lotToLngLat({ x: 100, y: 0 });
    expect(eastLng).toBeGreaterThan(DEFAULT_LOT_ORIGIN.lonDeg);
    expect(eastLat).toBeCloseTo(DEFAULT_LOT_ORIGIN.latDeg, 12);

    const [northLng, northLat] = lotToLngLat({ x: 0, y: 100 });
    expect(northLat).toBeGreaterThan(DEFAULT_LOT_ORIGIN.latDeg);
    expect(northLng).toBeCloseTo(DEFAULT_LOT_ORIGIN.lonDeg, 12);
  });

  it('scales with the WGS84 radii of curvature at the origin latitude', () => {
    // At the equator the prime-vertical radius is the semi-major axis itself, so one
    // degree of longitude is a * pi/180 = 111319.49 m. The meridian is shorter because
    // the earth is flattened.
    const equator = originScale({ latDeg: 0, lonDeg: 0 });
    expect(equator.metresPerDegreeEast).toBeCloseTo(111319.4908, 3);
    expect(equator.metresPerDegreeNorth).toBeCloseTo(110574.2758, 3);
    expect(equator.metresPerDegreeNorth).toBeLessThan(equator.metresPerDegreeEast);

    // East scale shrinks with cos(lat); north scale grows slightly.
    const high = originScale({ latDeg: 60, lonDeg: 0 });
    expect(high.metresPerDegreeEast).toBeLessThan(equator.metresPerDegreeEast * 0.51);
    expect(high.metresPerDegreeNorth).toBeGreaterThan(equator.metresPerDegreeNorth);
  });

  it('refuses an origin where the tangent-plane model breaks down', () => {
    expect(() => originScale({ latDeg: 89.5, lonDeg: 0 })).toThrow(RangeError);
    expect(() => originScale({ latDeg: 10, lonDeg: 200 })).toThrow(RangeError);
  });
});

describe('agreement with parkscan_core.geo (cross-language)', () => {
  /**
   * The generator writes `lot.geojson` in WGS84 using
   * `packages/parkscan_core/src/parkscan_core/geo.py`. If this module's formula or
   * constants drift from that one, the dashboard draws robots in the wrong place while
   * every Python test still passes — so the check has to run against the real artefact,
   * not a fixture written by hand here.
   */
  it('recovers the generator-declared lot size from the generator-written geometry', () => {
    const raw: unknown = JSON.parse(lotGeoJsonText);
    const collection = raw as {
      properties: { origin: { lat: number; lon: number } };
      features: {
        properties: Record<string, unknown>;
        geometry: { type: string; coordinates: number[][][] };
      }[];
    };

    const origin: LotOrigin = {
      latDeg: collection.properties.origin.lat,
      lonDeg: collection.properties.origin.lon,
    };

    const outline = collection.features.find((f) => f.properties['layer'] === 'lot_outline');
    expect(outline, 'lot.geojson has no lot_outline feature').toBeDefined();

    const ring = outline!.geometry.coordinates[0]!;
    const corners = ring.map((position) => lngLatToLot([position[0]!, position[1]!], origin));
    const xs = corners.map((c) => c.x);
    const ys = corners.map((c) => c.y);

    const widthM = Math.max(...xs) - Math.min(...xs);
    const heightM = Math.max(...ys) - Math.min(...ys);

    expect(widthM).toBeCloseTo(outline!.properties['width_m'] as number, 3);
    expect(heightM).toBeCloseTo(outline!.properties['height_m'] as number, 3);
    // Same 1 mm budget as C10, applied across the language boundary.
    expect(Math.abs(widthM - (outline!.properties['width_m'] as number))).toBeLessThan(ONE_MM);
    expect(Math.abs(heightM - (outline!.properties['height_m'] as number))).toBeLessThan(ONE_MM);
  });
});
