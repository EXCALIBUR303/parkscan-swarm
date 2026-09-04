import type { ExpressionSpecification, StyleSpecification } from 'maplibre-gl';
import { useCallback, useMemo, useState } from 'react';
import Map, { Layer, Marker, NavigationControl, Source } from 'react-map-gl/maplibre';

import { lotToLngLat, type LotOrigin } from '../lib/enu';
import { featuresOfKind, withCoverage, type LoadedLot } from '../lib/lot';
import { BAY_COLOUR, LOT_COLOUR, SEVERITY_COLOUR, coverageInterpolateStops } from '../lib/palette';
import { batteryPercent, describeRobot } from '../lib/robot';
import {
  useBatteryBand,
  useCoverageByZone,
  usePlottableDetections,
  useRobot,
  useRobotIds,
} from '../store';
import { MapLegend } from './MapLegend';

/**
 * The lot map.
 *
 * **There is deliberately no basemap.** The lot is synthetic, so an OSM raster or vector
 * basemap would draw simulated robots on top of a real, wrong street — actively
 * deceptive in a project whose whole premise is not overstating what exists (ADR-0004).
 * The style below has exactly one `background` layer and then the lot's own GeoJSON.
 *
 * Zone coverage is a **choropleth**, not a heatmap: an `interpolate` ramp over
 * `['get','coverage']` with the class breaks the legend prints, because coverage is a
 * countable ratio and a kernel-density smear would imply measurement never taken.
 *
 * Render split follows docs/ARCHITECTURE.md section 7 — the handful of robots are DOM
 * `<Marker>`s (focusable, real buttons, CSS-transitioned) and the ~180 static lot
 * features are GPU GeoJSON layers.
 *
 * MapLibre GL 6 is WebGL2 + ESM only and **cannot render in jsdom**, so component tests
 * mock `react-map-gl/maplibre`. Real map behaviour is verified in Playwright in Phase 5;
 * nothing here should be read as "the map is unit tested".
 */

/**
 * An empty style. No `sprite` and no `glyphs` are declared because there are no symbol
 * layers — the app must not depend on a remote font or sprite server to draw the lot.
 */
const BASE_STYLE: StyleSpecification = {
  version: 8,
  sources: {},
  layers: [
    {
      id: 'background',
      type: 'background',
      paint: { 'background-color': LOT_COLOUR.background },
    },
  ],
};

/**
 * MapLibre types `interpolate` as a fixed-arity tuple, so a stop list assembled from data
 * cannot be spelled without one cast. The stops come from `COVERAGE_BREAKS`, which the
 * legend also renders, so the ramp and its key cannot drift apart.
 */
const COVERAGE_FILL = [
  'interpolate',
  ['linear'],
  ['get', 'coverage'],
  ...coverageInterpolateStops(),
] as ExpressionSpecification;

const SPACE_FILL: ExpressionSpecification = [
  'match',
  ['get', 'state'],
  'vacant',
  BAY_COLOUR.vacant,
  'occupied',
  BAY_COLOUR.occupied,
  BAY_COLOUR.unknown,
];

export interface LotMapProps {
  lot: LoadedLot;
}

export function LotMap({ lot }: LotMapProps): React.JSX.Element {
  const coverageByZone = useCoverageByZone();
  const robotIds = useRobotIds();
  const plottable = usePlottableDetections();

  // Only geolocated detections become features. A rejected ray has no world position
  // (criterion C11) and is shown in the side panel instead — plotting it at a guessed
  // location would be a confident lie on a map people are meant to trust.
  const detectionFeatures = useMemo(
    () => ({
      type: 'FeatureCollection' as const,
      features: plottable.map((d) => ({
        type: 'Feature' as const,
        id: d.id,
        properties: {
          id: d.id,
          severity: d.severity ?? 'low',
          confidence: d.confidence,
          detSource: d.detSource,
        },
        geometry: {
          type: 'Point' as const,
          // Spread to a mutable tuple: lotToLngLat returns a readonly pair, and GeoJSON's
          // Position type is mutable.
          coordinates: [
            ...lotToLngLat({ x: d.position?.x ?? 0, y: d.position?.y ?? 0 }, lot.origin),
          ],
        },
      })),
    }),
    [plottable, lot.origin],
  );

  /*
   * Marker position is CSS-transitioned between telemetry frames, and MapLibre writes
   * the same `transform` while the user pans. Transitioning during a pan would make
   * every robot visibly lag the map, so the tween is switched off for the duration of
   * the gesture and back on when it ends. Two renders per gesture, not per frame.
   */
  const [mapMoving, setMapMoving] = useState(false);
  const onMoveStart = useCallback(() => {
    setMapMoving(true);
  }, []);
  const onMoveEnd = useCallback(() => {
    setMapMoving(false);
  }, []);

  const zones = useMemo(
    () => withCoverage(featuresOfKind(lot.collection, 'zone'), coverageByZone),
    [lot.collection, coverageByZone],
  );
  const outline = useMemo(() => featuresOfKind(lot.collection, 'lot_outline'), [lot.collection]);
  const lanes = useMemo(() => featuresOfKind(lot.collection, 'lane'), [lot.collection]);
  const spaces = useMemo(() => featuresOfKind(lot.collection, 'parking_space'), [lot.collection]);
  const bays = useMemo(() => featuresOfKind(lot.collection, 'charging_bay'), [lot.collection]);

  // MapLibre's `LngLatBoundsLike` is a mutable tuple; ours is readonly by design.
  const bounds = useMemo<[[number, number], [number, number]]>(
    () => [
      [lot.bounds[0][0], lot.bounds[0][1]],
      [lot.bounds[1][0], lot.bounds[1][1]],
    ],
    [lot.bounds],
  );

  return (
    <div className="lotmap" data-testid="lotmap">
      <Map
        initialViewState={{ bounds, fitBoundsOptions: { padding: 32 } }}
        mapStyle={BASE_STYLE}
        maxZoom={24}
        minZoom={12}
        attributionControl={false}
        onMoveStart={onMoveStart}
        onMoveEnd={onMoveEnd}
        // No basemap means no tile attribution to show; the lot is our own data.
        style={{ width: '100%', height: '100%' }}
      >
        <NavigationControl position="top-right" showCompass={false} />

        {/* Zone coverage choropleth, drawn first so everything else sits on top. */}
        <Source id="zones" type="geojson" data={zones} promoteId="id">
          <Layer
            id="zone-fill"
            type="fill"
            paint={{ 'fill-color': COVERAGE_FILL, 'fill-opacity': 0.85 }}
          />
          <Layer
            id="zone-outline"
            type="line"
            paint={{ 'line-color': LOT_COLOUR.zoneOutline, 'line-width': 1.2 }}
          />
        </Source>

        <Source id="lanes" type="geojson" data={lanes} promoteId="id">
          <Layer
            id="lane-line"
            type="line"
            paint={{ 'line-color': LOT_COLOUR.laneFill, 'line-width': 3, 'line-opacity': 0.9 }}
            layout={{ 'line-cap': 'round' }}
          />
        </Source>

        <Source id="spaces" type="geojson" data={spaces} promoteId="id">
          <Layer
            id="space-fill"
            type="fill"
            paint={{ 'fill-color': SPACE_FILL, 'fill-opacity': 0.9 }}
          />
          <Layer
            id="space-outline"
            type="line"
            paint={{ 'line-color': '#0B0E12', 'line-width': 0.5 }}
          />
        </Source>

        <Source id="bays" type="geojson" data={bays} promoteId="id">
          <Layer
            id="bay-point"
            type="circle"
            paint={{
              'circle-radius': 7,
              'circle-color': LOT_COLOUR.chargingBay,
              'circle-stroke-color': '#FFFFFF',
              'circle-stroke-width': 1.5,
            }}
          />
        </Source>

        <Source id="detections" type="geojson" data={detectionFeatures} promoteId="id">
          {/*
            Severity carries in BOTH colour and radius. Okabe-Ito hues stay distinguishable
            for the ~8% of men with a colour-vision deficiency, and the size ramp means the
            level survives a greyscale print or a screenshot run through a filter — a
            colour-only encoding would not.
          */}
          <Layer
            id="detection-point"
            type="circle"
            paint={{
              'circle-radius': [
                'match',
                ['get', 'severity'],
                'high',
                8,
                'medium',
                6,
                4,
              ] as unknown as number,
              'circle-color': [
                'match',
                ['get', 'severity'],
                'high',
                SEVERITY_COLOUR.high,
                'medium',
                SEVERITY_COLOUR.medium,
                SEVERITY_COLOUR.low,
              ] as unknown as string,
              'circle-stroke-color': '#FFFFFF',
              'circle-stroke-width': 1.5,
              'circle-opacity': 0.9,
            }}
          />
        </Source>

        <Source id="outline" type="geojson" data={outline} promoteId="id">
          <Layer
            id="lot-outline"
            type="line"
            paint={{ 'line-color': LOT_COLOUR.outline, 'line-width': 2 }}
          />
        </Source>

        {robotIds.map((id) => (
          <RobotMarker key={id} robotId={id} origin={lot.origin} tweening={!mapMoving} />
        ))}
      </Map>

      <MapLegend />
    </div>
  );
}

/**
 * One robot marker, subscribed to one robot.
 *
 * Each marker selects only its own slice of the store, so `robot_1` moving does not
 * re-render `robot_2`'s marker or card. The battery *band* comes from a second, narrower
 * selector so the colour class only changes when the band does — not on every percent.
 *
 * The button is a real `<button>` with a descriptive `aria-label`, because a rotated
 * glyph is not a control and a WebGL canvas cannot be reached by a keyboard at all.
 * Motion is CSS only, so `prefers-reduced-motion` (see styles.css) turns it off without
 * this component knowing anything about it.
 */
function RobotMarker({
  robotId,
  origin,
  tweening,
}: {
  robotId: string;
  origin: LotOrigin;
  tweening: boolean;
}): React.JSX.Element | null {
  const robot = useRobot(robotId);
  const band = useBatteryBand(robotId);
  if (robot === undefined || band === undefined) return null;

  // The one place lot metres become longitude/latitude (ADR-0004).
  const [lng, lat] = lotToLngLat(robot.position, origin);
  const headingDeg = (robot.headingRad * 180) / Math.PI;

  return (
    <Marker
      longitude={lng}
      latitude={lat}
      anchor="center"
      subpixelPositioning
      className={`robot-anchor${tweening ? '' : ' robot-anchor--fixed'}`}
    >
      <button
        type="button"
        className={`robot-marker robot-marker--${band}`}
        // Rotation is negated: screen y grows downward, lot y grows north.
        style={{ transform: `rotate(${String(-headingDeg)}deg)` }}
        aria-label={describeRobot(robot)}
        title={`${robotId} — ${String(batteryPercent(robot.batterySoc))}%`}
      >
        <span aria-hidden="true">&#9650;</span>
      </button>
    </Marker>
  );
}
