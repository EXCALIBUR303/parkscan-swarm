/**
 * Lot-metres <-> WGS84 for a single, fixed lot origin.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * MapLibre GL is Web-Mercator-only: it has no equivalent of Leaflet's `L.CRS.Simple`,
 * so a purely metric lot cannot be handed to it directly (ADR-0004). Everything above
 * this module works in lot metres; only the map boundary converts.
 *
 * THE APPROXIMATION
 * -----------------
 * A local tangent plane (ENU) at the lot origin, linearised with the WGS84 radii of
 * curvature at that origin:
 *
 *   e2 = f * (2 - f)                                (first eccentricity squared)
 *   w  = sqrt(1 - e2 * sin^2(lat0))
 *   N  = a / w                                      (prime vertical radius of curvature)
 *   M  = a * (1 - e2) / w^3                         (meridional radius of curvature)
 *
 *   metresPerDegreeNorth = M * pi/180
 *   metresPerDegreeEast  = N * cos(lat0) * pi/180
 *
 *   lon = lon0 + x / metresPerDegreeEast
 *   lat = lat0 + y / metresPerDegreeNorth
 *
 * Axes follow REP-103: x is East, y is North, both in metres, right-handed with z up.
 * The map is flat within the lot: no earth curvature term, no origin-relative rotation.
 *
 * ACCURACY
 * --------
 * The forward and inverse maps are the same linear scaling, so the round trip is exact
 * to double-precision (~1e-10 m over this lot) — far inside the 1 mm budget of
 * ACCEPTANCE C10. The *absolute* error against a rigorous geodesic grows with distance
 * from the origin; over a 150 m x 90 m lot it is well under a centimetre, which is an
 * order of magnitude below the simulator's own pose noise.
 *
 * MUST MATCH THE BACKEND
 * ----------------------
 * `packages/parkscan_core/src/parkscan_core/geo.py` must implement exactly this formula
 * with exactly these constants, or the dashboard and the database will disagree about
 * where a robot is. See notes for the integrator.
 */

/** WGS84 semi-major axis, metres. */
export const WGS84_A = 6378137.0;
/** WGS84 flattening. */
export const WGS84_F = 1 / 298.257223563;
/** WGS84 first eccentricity squared. */
export const WGS84_E2 = WGS84_F * (2 - WGS84_F);

const DEG = Math.PI / 180;

/** Geodetic anchor of the lot's local metric frame. */
export interface LotOrigin {
  /** Latitude of lot (0, 0), degrees. */
  readonly latDeg: number;
  /** Longitude of lot (0, 0), degrees. */
  readonly lonDeg: number;
}

/** A point in the lot's local metric frame. x = East, y = North, metres. */
export interface LotPoint {
  readonly x: number;
  readonly y: number;
}

/** GeoJSON coordinate order: [longitude, latitude], degrees. */
export type LngLat = readonly [lng: number, lat: number];

/**
 * Default synthetic origin. The lot does not exist in the real world — this anchor
 * exists only so PostGIS `geography` columns and MapLibre have somewhere to put it.
 *
 * Null Island (0, 0) on purpose, and this is not arbitrary:
 *   1. HONESTY. This is the fallback used when a lot file declares no origin. Any
 *      *plausible* land coordinate would silently render simulated robots on top of
 *      a real address, which ADR-0004 explicitly forbids. 0,0 is in the Gulf of
 *      Guinea and reads as synthetic to anyone who looks.
 *   2. CONSISTENCY. It must match `Origin` in packages/parkscan_core/src/parkscan_core/geo.py
 *      and the `origin` in sim/lot/lot.yaml, which are both 0,0. The two
 *      implementations are cross-checked numerically in CI.
 *   3. cos(lat0) == 1 at the equator, so east/west scaling carries no convergence
 *      distortion at all.
 */
export const DEFAULT_LOT_ORIGIN: LotOrigin = { latDeg: 0, lonDeg: 0 };

/** Metres-per-degree scale factors at a given origin latitude. */
export interface OriginScale {
  readonly metresPerDegreeNorth: number;
  readonly metresPerDegreeEast: number;
}

const scaleCache = new Map<number, OriginScale>();

/**
 * Beyond this the 1/cos(lat0) east scale blows up and the model is meaningless.
 * Mirrors `Origin.__post_init__` in `parkscan_core/geo.py`.
 */
const MAX_ABS_LAT_DEG = 89;

/**
 * Metres per degree of latitude/longitude at the origin, from the WGS84 radii of
 * curvature. Memoised on latitude: an origin never moves during a session.
 */
export function originScale(origin: LotOrigin): OriginScale {
  const cached = scaleCache.get(origin.latDeg);
  if (cached !== undefined) return cached;

  if (!Number.isFinite(origin.latDeg) || Math.abs(origin.latDeg) > MAX_ABS_LAT_DEG) {
    throw new RangeError(
      `lot origin latitude ${String(origin.latDeg)} is outside +/-${String(MAX_ABS_LAT_DEG)}deg; ` +
        'the local tangent-plane approximation is not usable there',
    );
  }
  if (!Number.isFinite(origin.lonDeg) || Math.abs(origin.lonDeg) > 180) {
    throw new RangeError(`lot origin longitude ${String(origin.lonDeg)} is out of range`);
  }

  const lat0 = origin.latDeg * DEG;
  const sinLat = Math.sin(lat0);
  const w = Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);
  const primeVertical = WGS84_A / w;
  const meridional = (WGS84_A * (1 - WGS84_E2)) / (w * w * w);

  const scale: OriginScale = {
    metresPerDegreeNorth: meridional * DEG,
    metresPerDegreeEast: primeVertical * Math.cos(lat0) * DEG,
  };
  scaleCache.set(origin.latDeg, scale);
  return scale;
}

/** Lot metres -> WGS84 [lng, lat]. */
export function lotToLngLat(point: LotPoint, origin: LotOrigin = DEFAULT_LOT_ORIGIN): LngLat {
  const { metresPerDegreeEast, metresPerDegreeNorth } = originScale(origin);
  return [
    origin.lonDeg + point.x / metresPerDegreeEast,
    origin.latDeg + point.y / metresPerDegreeNorth,
  ];
}

/** WGS84 [lng, lat] -> lot metres. */
export function lngLatToLot(lngLat: LngLat, origin: LotOrigin = DEFAULT_LOT_ORIGIN): LotPoint {
  const { metresPerDegreeEast, metresPerDegreeNorth } = originScale(origin);
  return {
    x: (lngLat[0] - origin.lonDeg) * metresPerDegreeEast,
    y: (lngLat[1] - origin.latDeg) * metresPerDegreeNorth,
  };
}

/** Great-circle-free planar distance between two lot points, metres. */
export function lotDistance(a: LotPoint, b: LotPoint): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}
