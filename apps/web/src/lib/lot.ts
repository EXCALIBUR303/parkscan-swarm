/**
 * Loading and projecting `lot.geojson`.
 *
 * This is the only place raw longitude/latitude enters the app. Everything above it —
 * the store, the panels, the robot markers — speaks lot metres, and conversion happens
 * here at the boundary (ADR-0004).
 *
 * The file is produced by `make lot` (`parkscan_core.lot`) from `sim/lot/lot.yaml` and
 * copied into `public/` by `scripts/sync-lot.mjs`. Its schema, as generated today:
 *
 *   - top level: `properties.origin = {lat, lon}`, `properties.size_m`, `properties.generator`
 *   - each feature: `properties.layer` in
 *     `lot_outline | zone | parking_space | charging_bay | lane`
 *   - `charging_bay` is a Point, `lane` a LineString, everything else a Polygon
 *   - coordinates are already WGS84 degrees
 *
 * A file authored in lot metres is also accepted, because `lot.yaml` is a metric spec and
 * a future generator may emit the metric form directly.
 */

import type { Feature, FeatureCollection, Geometry, Position } from 'geojson';

import { DEFAULT_LOT_ORIGIN, lotToLngLat, type LngLat, type LotOrigin } from './enu';
import { isBayState, isDataSource, type BayState, type DataSource } from './types';

/** What a feature in `lot.geojson` represents — the generator's `layer` values. */
export type LotFeatureKind = 'lot_outline' | 'zone' | 'parking_space' | 'charging_bay' | 'lane';

const LOT_FEATURE_KINDS: readonly LotFeatureKind[] = [
  'lot_outline',
  'zone',
  'parking_space',
  'charging_bay',
  'lane',
];

export interface LotFeatureProperties {
  /** Normalised from the generator's `layer` (or `kind`). */
  kind: LotFeatureKind;
  id: string;
  label?: string;
  /** `parking_space` only — occupancy as last observed. Absent in the source file. */
  state?: BayState;
  /** Injected at render time from `GET /api/v1/coverage`; 0..1. `zone` only. */
  coverage?: number;
  [key: string]: unknown;
}

export type LotFeature = Feature<Geometry, LotFeatureProperties>;
export type LotCollection = FeatureCollection<Geometry, LotFeatureProperties>;

/** Which coordinate reference system the source file is written in. */
export type LotCrs = 'lot-metres' | 'EPSG:4326';

/** `[[west, south], [east, north]]` — the shape MapLibre's `fitBounds` wants. */
export type LngLatBounds = readonly [sw: LngLat, ne: LngLat];

export interface LoadedLot {
  /** Source CRS, as declared by the file or inferred from its extent. */
  readonly crs: LotCrs;
  /** The origin the metric frame is anchored to. */
  readonly origin: LotOrigin;
  /** Always WGS84, whatever the source CRS was — this is what MapLibre consumes. */
  readonly collection: LotCollection;
  readonly bounds: LngLatBounds;
  /** Declared provenance. Drives the SIMULATED badge until the API is wired (H1). */
  readonly dataSource: DataSource;
  readonly counts: {
    readonly zones: number;
    readonly parkingSpaces: number;
    readonly chargingBays: number;
    readonly lanes: number;
  };
}

/* --------------------------------------------------------------- file metadata -- */

/**
 * Collection-level metadata, read from either the generator's `properties` object or
 * RFC 7946 §6.1 foreign members prefixed `parkscan:`. Both spellings are accepted so a
 * change on the Python side cannot silently break the map.
 */
function metaValue(collection: LotCollection, key: string): unknown {
  const raw = collection as unknown as Record<string, unknown>;
  const foreign = raw[`parkscan:${key}`];
  if (foreign !== undefined) return foreign;
  const props = raw['properties'];
  if (props !== null && typeof props === 'object') {
    return (props as Record<string, unknown>)[key];
  }
  return undefined;
}

/**
 * The file's CRS.
 *
 * An explicit `crs` marker wins. Failing that, extent decides, and it decides
 * unambiguously: a parking lot is metres-to-hundreds-of-metres across, while one degree
 * is ~111 km. So an extent below 1 can only be degrees, and an extent of 1 or more can
 * only be metres — there is no lot size for which both readings are plausible.
 */
export function detectLotCrs(collection: LotCollection): LotCrs {
  const declared = metaValue(collection, 'crs');
  if (declared === 'EPSG:4326' || declared === 'urn:ogc:def:crs:OGC::CRS84') return 'EPSG:4326';
  if (declared === 'lot-metres') return 'lot-metres';

  const [[west, south], [east, north]] = boundsOf(collection);
  return Math.max(east - west, north - south) >= 1 ? 'lot-metres' : 'EPSG:4326';
}

/** The origin declared by the file, falling back to the compiled-in default. */
export function detectLotOrigin(collection: LotCollection): LotOrigin {
  const declared = metaValue(collection, 'origin');
  if (declared !== null && typeof declared === 'object') {
    const candidate = declared as { lat?: unknown; lon?: unknown };
    if (typeof candidate.lat === 'number' && typeof candidate.lon === 'number') {
      return { latDeg: candidate.lat, lonDeg: candidate.lon };
    }
  }
  return DEFAULT_LOT_ORIGIN;
}

/** Provenance declared by the file. Anything unrecognised is treated as simulated. */
export function detectLotDataSource(collection: LotCollection): DataSource {
  const declared = metaValue(collection, 'data_source');
  return isDataSource(declared) ? declared : 'simulated';
}

/* ------------------------------------------------------------------- geometry -- */

type PositionFn = (position: Position) => Position;

function mapGeometry(geometry: Geometry, fn: PositionFn): Geometry {
  switch (geometry.type) {
    case 'Point':
      return { type: 'Point', coordinates: fn(geometry.coordinates) };
    case 'MultiPoint':
      return { type: 'MultiPoint', coordinates: geometry.coordinates.map(fn) };
    case 'LineString':
      return { type: 'LineString', coordinates: geometry.coordinates.map(fn) };
    case 'MultiLineString':
      return {
        type: 'MultiLineString',
        coordinates: geometry.coordinates.map((line) => line.map(fn)),
      };
    case 'Polygon':
      return { type: 'Polygon', coordinates: geometry.coordinates.map((ring) => ring.map(fn)) };
    case 'MultiPolygon':
      return {
        type: 'MultiPolygon',
        coordinates: geometry.coordinates.map((poly) => poly.map((ring) => ring.map(fn))),
      };
    case 'GeometryCollection':
      return {
        type: 'GeometryCollection',
        geometries: geometry.geometries.map((child) => mapGeometry(child, fn)),
      };
  }
}

function eachPosition(geometry: Geometry, visit: (position: Position) => void): void {
  mapGeometry(geometry, (position) => {
    visit(position);
    return position;
  });
}

/** Project a lot-metres collection to WGS84. A collection already in WGS84 is returned as-is. */
export function toWgs84(collection: LotCollection, origin: LotOrigin, crs: LotCrs): LotCollection {
  if (crs === 'EPSG:4326') return collection;
  return {
    ...collection,
    features: collection.features.map((feature) => ({
      ...feature,
      geometry: mapGeometry(feature.geometry, ([x = 0, y = 0]) => {
        const [lng, lat] = lotToLngLat({ x, y }, origin);
        return [lng, lat];
      }),
    })),
  };
}

/** Axis-aligned bounds of every position in the collection, in the collection's own units. */
export function boundsOf(collection: LotCollection): LngLatBounds {
  let west = Number.POSITIVE_INFINITY;
  let south = Number.POSITIVE_INFINITY;
  let east = Number.NEGATIVE_INFINITY;
  let north = Number.NEGATIVE_INFINITY;

  for (const feature of collection.features) {
    eachPosition(feature.geometry, ([lng, lat]) => {
      if (lng === undefined || lat === undefined) return;
      if (lng < west) west = lng;
      if (lng > east) east = lng;
      if (lat < south) south = lat;
      if (lat > north) north = lat;
    });
  }

  if (!Number.isFinite(west) || !Number.isFinite(south)) {
    throw new Error('lot.geojson contains no coordinates');
  }
  return [
    [west, south],
    [east, north],
  ];
}

/* ---------------------------------------------------------------- normalising -- */

function isLotFeatureKind(value: unknown): value is LotFeatureKind {
  return typeof value === 'string' && (LOT_FEATURE_KINDS as readonly string[]).includes(value);
}

/** Runtime shape check — a truncated or unrelated JSON file must fail loudly, not render empty. */
function assertLotCollection(value: unknown): asserts value is LotCollection {
  if (value === null || typeof value !== 'object') {
    throw new Error('lot.geojson is not a JSON object');
  }
  const candidate = value as { type?: unknown; features?: unknown };
  if (candidate.type !== 'FeatureCollection' || !Array.isArray(candidate.features)) {
    throw new Error('lot.geojson is not a GeoJSON FeatureCollection');
  }
  if (candidate.features.length === 0) {
    throw new Error('lot.geojson contains no features');
  }
}

/**
 * Normalise feature properties.
 *
 * The generator writes `layer`; `kind` is accepted as a synonym. An unrecognised layer
 * becomes `lane`, which renders as inert lot furniture rather than being silently
 * promoted into the zone choropleth or the parking-space count.
 */
export function normaliseProperties(collection: LotCollection): LotCollection {
  return {
    ...collection,
    features: collection.features.map((feature, index) => {
      const raw: Record<string, unknown> = { ...(feature.properties as unknown as object) };
      const layer: unknown = raw['layer'] ?? raw['kind'];
      const kind: LotFeatureKind = isLotFeatureKind(layer) ? layer : 'lane';
      const id = typeof raw['id'] === 'string' ? raw['id'] : `feature_${String(index)}`;
      const label = typeof raw['name'] === 'string' ? raw['name'] : undefined;
      const state: BayState | undefined =
        kind === 'parking_space'
          ? isBayState(raw['state'])
            ? raw['state']
            : 'unknown'
          : undefined;

      return {
        ...feature,
        id,
        properties: {
          ...raw,
          kind,
          id,
          ...(label === undefined ? {} : { label }),
          ...(state === undefined ? {} : { state }),
        },
      };
    }),
  };
}

/**
 * Fetch and prepare the lot.
 *
 * Served as a static asset from `public/lot.geojson`; `scripts/sync-lot.mjs` keeps that
 * copy in step with `sim/lot/lot.geojson` on every `npm run dev` and `npm run build`.
 */
export async function loadLot(url = '/lot.geojson', signal?: AbortSignal): Promise<LoadedLot> {
  const response = await fetch(url, signal ? { signal } : {});
  if (!response.ok) {
    throw new Error(`GET ${url} failed: ${String(response.status)} ${response.statusText}`);
  }
  const parsed: unknown = await response.json();
  assertLotCollection(parsed);

  const normalised = normaliseProperties(parsed);
  const crs = detectLotCrs(normalised);
  const origin = detectLotOrigin(normalised);
  const collection = toWgs84(normalised, origin, crs);

  return {
    crs,
    origin,
    collection,
    bounds: boundsOf(collection),
    dataSource: detectLotDataSource(normalised),
    counts: {
      zones: countKind(collection, 'zone'),
      parkingSpaces: countKind(collection, 'parking_space'),
      chargingBays: countKind(collection, 'charging_bay'),
      lanes: countKind(collection, 'lane'),
    },
  };
}

function countKind(collection: LotCollection, kind: LotFeatureKind): number {
  return collection.features.filter((f) => f.properties.kind === kind).length;
}

/**
 * Merge live coverage into the zone features.
 *
 * Kept out of the source data: the choropleth reads `['get', 'coverage']`, and a zone the
 * API has not reported on yet is 0 — never "assume complete".
 */
export function withCoverage(
  collection: LotCollection,
  coverageByZone: Readonly<Record<string, number>>,
): LotCollection {
  return {
    ...collection,
    features: collection.features.map((feature) =>
      feature.properties.kind === 'zone'
        ? {
            ...feature,
            properties: {
              ...feature.properties,
              coverage: coverageByZone[feature.properties.id] ?? 0,
            },
          }
        : feature,
    ),
  };
}

/** Features of one kind, as their own collection — one MapLibre source per kind. */
export function featuresOfKind(collection: LotCollection, kind: LotFeatureKind): LotCollection {
  return {
    type: 'FeatureCollection',
    features: collection.features.filter((f) => f.properties.kind === kind),
  };
}
