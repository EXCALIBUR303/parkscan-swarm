import { afterEach, describe, expect, it, vi } from 'vitest';

import REAL_LOT_TEXT from '../../public/lot.geojson?raw';

import { DEFAULT_LOT_ORIGIN } from './enu';

import {
  boundsOf,
  detectLotCrs,
  detectLotDataSource,
  detectLotOrigin,
  featuresOfKind,
  loadLot,
  normaliseProperties,
  toWgs84,
  withCoverage,
  type LotCollection,
} from './lot';

function realLot(): LotCollection {
  return JSON.parse(REAL_LOT_TEXT) as LotCollection;
}

function metricLot(): LotCollection {
  return {
    type: 'FeatureCollection',
    features: [
      {
        type: 'Feature',
        properties: { layer: 'zone', kind: 'zone', id: 'zone_01' },
        geometry: {
          type: 'Polygon',
          coordinates: [
            [
              [0, 0],
              [50, 0],
              [50, 25],
              [0, 25],
              [0, 0],
            ],
          ],
        },
      },
    ],
  };
}

afterEach(() => {
  vi.unstubAllGlobals();
});

describe('CRS detection', () => {
  it('reads the real generator output as WGS84 degrees', () => {
    expect(detectLotCrs(realLot())).toBe('EPSG:4326');
  });

  it('reads a metric collection as lot metres', () => {
    expect(detectLotCrs(metricLot())).toBe('lot-metres');
  });

  it('prefers an explicit marker over the extent heuristic', () => {
    const marked = { ...metricLot(), 'parkscan:crs': 'EPSG:4326' } as LotCollection;
    expect(detectLotCrs(marked)).toBe('EPSG:4326');
  });
});

describe('metadata', () => {
  it('reads the origin the generator declared', () => {
    expect(detectLotOrigin(realLot())).toEqual({ latDeg: 0, lonDeg: 0 });
  });

  it('falls back to the default origin when none is declared', () => {
    // Asserted symbolically, not against a literal: the point of this test is that the
    // fallback is *the declared default*, whatever it is. A hardcoded number here would
    // have to be edited in lockstep with enu.ts, and would fail loudly for the wrong reason.
    expect(detectLotOrigin(metricLot())).toEqual(DEFAULT_LOT_ORIGIN);
  });

  it('never falls back to a real-world location', () => {
    // A plausible land coordinate as the fallback would draw simulated robots on top of
    // somewhere real whenever a lot file omits its origin. ADR-0004 forbids that, and this
    // test is what stops a future "nicer looking default" from reintroducing it.
    expect(detectLotOrigin(metricLot())).toEqual({ latDeg: 0, lonDeg: 0 });
  });

  it('treats an undeclared provenance as simulated', () => {
    expect(detectLotDataSource(realLot())).toBe('simulated');
    expect(detectLotDataSource(metricLot())).toBe('simulated');
  });
});

describe('normalisation', () => {
  it('maps the generator "layer" onto "kind"', () => {
    const normalised = normaliseProperties(realLot());
    const kinds = new Set(normalised.features.map((f) => f.properties.kind));
    expect(kinds).toEqual(
      new Set(['lot_outline', 'zone', 'parking_space', 'charging_bay', 'lane']),
    );
  });

  it('defaults parking-space occupancy to unknown, never to vacant', () => {
    const normalised = normaliseProperties(realLot());
    const spaces = featuresOfKind(normalised, 'parking_space');
    expect(spaces.features.length).toBeGreaterThan(0);
    for (const space of spaces.features) {
      expect(space.properties.state).toBe('unknown');
    }
  });
});

describe('projection', () => {
  it('turns lot metres into the same degrees the generator would have written', () => {
    const projected = toWgs84(
      normaliseProperties(metricLot()),
      { latDeg: 0, lonDeg: 0 },
      'lot-metres',
    );
    const ring = (projected.features[0]!.geometry as { coordinates: number[][][] }).coordinates[0]!;
    // 50 m east at the equator.
    expect(ring[1]![0]!).toBeCloseTo(50 / 111319.4908, 9);
    expect(ring[1]![1]!).toBeCloseTo(0, 12);
  });

  it('leaves a WGS84 collection untouched', () => {
    const collection = realLot();
    expect(toWgs84(collection, { latDeg: 0, lonDeg: 0 }, 'EPSG:4326')).toBe(collection);
  });

  it('computes bounds over every geometry type in the file', () => {
    const [[west, south], [east, north]] = boundsOf(realLot());
    expect(east).toBeGreaterThan(west);
    expect(north).toBeGreaterThan(south);
  });
});

describe('coverage merge', () => {
  it('defaults an unreported zone to 0, not to complete', () => {
    const zones = withCoverage(featuresOfKind(normaliseProperties(realLot()), 'zone'), {
      zone_01: 0.42,
    });
    const first = zones.features.find((f) => f.properties.id === 'zone_01');
    const other = zones.features.find((f) => f.properties.id !== 'zone_01');
    expect(first?.properties.coverage).toBe(0.42);
    expect(other?.properties.coverage).toBe(0);
  });
});

describe('loadLot', () => {
  function stubFetch(body: string, ok = true, status = 200): void {
    vi.stubGlobal(
      'fetch',
      vi.fn(() =>
        Promise.resolve({
          ok,
          status,
          statusText: ok ? 'OK' : 'Not Found',
          json: () => Promise.resolve(JSON.parse(body) as unknown),
        }),
      ),
    );
  }

  it('loads the committed lot and reports its contents', async () => {
    stubFetch(REAL_LOT_TEXT);
    const lot = await loadLot();
    expect(lot.crs).toBe('EPSG:4326');
    expect(lot.dataSource).toBe('simulated');
    expect(lot.counts.zones).toBe(12);
    expect(lot.counts.chargingBays).toBe(2);
    expect(lot.counts.parkingSpaces).toBeGreaterThan(0);
    expect(lot.counts.lanes).toBeGreaterThan(0);
  });

  it('fails loudly on a non-GeoJSON body instead of rendering an empty map', async () => {
    stubFetch('{"hello":"world"}');
    await expect(loadLot()).rejects.toThrow(/FeatureCollection/);
  });

  it('fails loudly on an empty FeatureCollection', async () => {
    stubFetch('{"type":"FeatureCollection","features":[]}');
    await expect(loadLot()).rejects.toThrow(/no features/);
  });

  it('reports a failed request', async () => {
    stubFetch('{}', false, 404);
    await expect(loadLot()).rejects.toThrow(/404/);
  });
});
