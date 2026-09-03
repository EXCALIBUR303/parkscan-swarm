/**
 * Stand-in for `react-map-gl/maplibre` in jsdom tests.
 *
 * MapLibre GL 6 requires WebGL2 and is ESM-only; jsdom provides neither, so the real map
 * genuinely cannot render here. Rather than pretend otherwise, these components record
 * that the right sources and layers were declared with the right types, and actual map
 * behaviour (camera fit, choropleth paint, marker positioning) is deferred to Playwright
 * in Phase 5. Nothing in this file should be read as evidence that the map works.
 */

import type { ReactNode } from 'react';

interface AnyProps {
  children?: ReactNode;
  id?: string;
  type?: string;
  [key: string]: unknown;
}

export default function Map({ children }: AnyProps): React.JSX.Element {
  return <div data-testid="maplibre-map">{children}</div>;
}

export function Source({ children, id }: AnyProps): React.JSX.Element {
  return (
    <div data-testid={`source-${id ?? 'unknown'}`} data-source-id={id}>
      {children}
    </div>
  );
}

export function Layer({ id, type }: AnyProps): React.JSX.Element {
  return <div data-testid={`layer-${id ?? 'unknown'}`} data-layer-type={type} />;
}

export function Marker({ children }: AnyProps): React.JSX.Element {
  return <div data-testid="marker">{children}</div>;
}

export function NavigationControl(): React.JSX.Element {
  return <div data-testid="navigation-control" />;
}
