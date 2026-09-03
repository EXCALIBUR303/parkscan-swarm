import {
  BAY_COLOUR,
  BAY_ORDER,
  COVERAGE_BREAKS,
  LOT_COLOUR,
  SEVERITY_COLOUR,
  SEVERITY_ORDER,
  SEVERITY_SHAPE,
  type SeverityShape,
} from '../lib/palette';

/**
 * Map legend.
 *
 * Coverage gets **explicit numeric breaks** because it is a choropleth of a countable
 * ratio, not a heatmap of a smoothed density (ADR-0004): "50%" is a fact, "warmer" is not.
 *
 * Severity swatches pair each Okabe-Ito colour with a distinct shape, so the encoding
 * survives greyscale printing and monochrome vision. Severity pins are not on the map
 * until Phase 3 — the key is here because the palette is fixed now and the two must not
 * be allowed to drift.
 */

function ShapeSwatch({
  shape,
  colour,
}: {
  shape: SeverityShape;
  colour: string;
}): React.JSX.Element {
  return (
    <svg className="legend__swatch" viewBox="0 0 16 16" aria-hidden="true" focusable="false">
      {shape === 'circle' && <circle cx="8" cy="8" r="6" fill={colour} />}
      {shape === 'triangle' && <polygon points="8,2 15,14 1,14" fill={colour} />}
      {shape === 'square' && <rect x="2" y="2" width="12" height="12" fill={colour} />}
    </svg>
  );
}

function BoxSwatch({ colour, outline }: { colour: string; outline?: string }): React.JSX.Element {
  return (
    <span
      className="legend__swatch legend__swatch--box"
      style={{ backgroundColor: colour, borderColor: outline ?? 'rgba(255,255,255,0.35)' }}
      aria-hidden="true"
    />
  );
}

export function MapLegend(): React.JSX.Element {
  return (
    <div className="legend">
      <section className="legend__group" aria-labelledby="legend-coverage">
        <h3 className="legend__title" id="legend-coverage">
          Zone coverage
        </h3>
        <p className="legend__note">cells swept / cells total</p>
        <ul className="legend__items legend__items--ramp">
          {COVERAGE_BREAKS.map((brk) => (
            <li key={brk.label} className="legend__item">
              <BoxSwatch colour={brk.colour} />
              <span>{brk.label}</span>
            </li>
          ))}
        </ul>
      </section>

      <section className="legend__group" aria-labelledby="legend-spaces">
        <h3 className="legend__title" id="legend-spaces">
          Parking spaces
        </h3>
        <ul className="legend__items">
          {BAY_ORDER.map((state) => (
            <li key={state} className="legend__item">
              <BoxSwatch colour={BAY_COLOUR[state]} />
              <span>{state}</span>
            </li>
          ))}
          <li className="legend__item">
            <BoxSwatch colour={LOT_COLOUR.chargingBay} />
            <span>charging bay</span>
          </li>
        </ul>
      </section>

      <section className="legend__group" aria-labelledby="legend-severity">
        <h3 className="legend__title" id="legend-severity">
          Pothole severity
        </h3>
        <p className="legend__note">colour and shape both carry the level</p>
        <ul className="legend__items">
          {SEVERITY_ORDER.map((severity) => (
            <li key={severity} className="legend__item">
              <ShapeSwatch shape={SEVERITY_SHAPE[severity]} colour={SEVERITY_COLOUR[severity]} />
              <span>{severity}</span>
            </li>
          ))}
        </ul>
        <p className="legend__note">no detections until Phase 3</p>
      </section>
    </div>
  );
}
