/**
 * Colour is data here, so it is defined once and imported — never spelled inline.
 *
 * Severity uses the Okabe-Ito colour-blind-safe palette and is **never** red-vs-green
 * (docs/ARCHITECTURE.md §7). Every severity is additionally paired with a distinct
 * shape so the meaning survives greyscale printing and monochrome vision.
 */

import type { BayState, Severity } from './types';

/** Okabe & Ito (2008), the eight-colour qualitative palette. */
export const OKABE_ITO = {
  black: '#000000',
  orange: '#E69F00',
  skyBlue: '#56B4E9',
  bluishGreen: '#009E73',
  yellow: '#F0E442',
  blue: '#0072B2',
  vermillion: '#D55E00',
  reddishPurple: '#CC79A7',
} as const;

/** Severity fill colours. Blue -> orange -> vermillion: monotone, not a traffic light. */
export const SEVERITY_COLOUR: Record<Severity, string> = {
  low: OKABE_ITO.blue,
  medium: OKABE_ITO.orange,
  high: OKABE_ITO.vermillion,
};

/** Redundant encoding: colour alone never carries the meaning. */
export type SeverityShape = 'circle' | 'triangle' | 'square';

export const SEVERITY_SHAPE: Record<Severity, SeverityShape> = {
  low: 'circle',
  medium: 'triangle',
  high: 'square',
};

export const SEVERITY_ORDER: readonly Severity[] = ['low', 'medium', 'high'];

/** Parking-space occupancy. Chosen to not collide with the severity ramp. */
export const BAY_COLOUR: Record<BayState, string> = {
  vacant: OKABE_ITO.skyBlue,
  occupied: OKABE_ITO.reddishPurple,
  unknown: '#B4B4B4',
};

export const BAY_ORDER: readonly BayState[] = ['vacant', 'occupied', 'unknown'];

/**
 * Sequential single-hue ramp for the zone-coverage choropleth.
 *
 * A choropleth, not a heatmap: coverage is a countable ratio (cells swept / cells total),
 * so it gets exact class breaks and a legend rather than a kernel-density smear whose
 * appearance is set by an aesthetic `heatmap-radius` knob (ADR-0004).
 *
 * Lightness is monotone, so the ramp still reads as an ordered scale in greyscale.
 */
export interface CoverageBreak {
  /** Coverage fraction, 0..1. */
  readonly value: number;
  readonly colour: string;
  /** Legend text — explicit numbers, never "low/high". */
  readonly label: string;
}

export const COVERAGE_BREAKS: readonly CoverageBreak[] = [
  { value: 0, colour: '#F2F2F2', label: '0%' },
  { value: 0.25, colour: '#C9DEEF', label: '25%' },
  { value: 0.5, colour: '#8FBBDD', label: '50%' },
  { value: 0.75, colour: '#4A93C4', label: '75%' },
  { value: 1, colour: OKABE_ITO.blue, label: '100%' },
];

/** Lot furniture. Not severity-carrying, so it stays outside the ramps above. */
export const LOT_COLOUR = {
  background: '#12161B',
  outline: '#7A8896',
  zoneOutline: '#5C6874',
  laneFill: '#1C222A',
  chargingBay: OKABE_ITO.bluishGreen,
  robot: OKABE_ITO.yellow,
} as const;

/**
 * MapLibre `interpolate` stop list for `['get', 'coverage']`, flattened.
 * Exported (rather than inlined in the layer) so the legend and the map cannot drift.
 */
export function coverageInterpolateStops(): (number | string)[] {
  return COVERAGE_BREAKS.flatMap((b) => [b.value, b.colour]);
}
