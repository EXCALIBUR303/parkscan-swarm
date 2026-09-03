import { setWorkerUrl } from 'maplibre-gl';
import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

// MapLibre's own control/marker CSS. Imported here rather than in LotMap so component
// tests never pull a stylesheet into jsdom.
import 'maplibre-gl/dist/maplibre-gl.css';

/*
 * MapLibre GL 6 loads its GeoJSON/vector worker by deriving a URL from its own
 * `import.meta.url`:
 *
 *     new URL('./maplibre-gl-worker.mjs', import.meta.url)
 *
 * That assumes the worker file sits next to the library file. Once a bundler rewrites
 * `maplibre-gl.mjs` into `assets/maplibre-gl-<hash>.js`, the sibling is not there, so the
 * request 404s — and MapLibre does *not* raise a map `error` event for it. The map stays
 * blank forever with a clean console, `isStyleLoaded()` stuck at false and every GeoJSON
 * source stuck mid-update. Reproduced here in both `vite dev` and `vite build` before
 * this line existed.
 *
 * `?worker&url` makes Vite bundle the worker entry (it imports `maplibre-gl-shared.mjs`,
 * so a bare `?url` copy would break on that import) and hand back the emitted URL, which
 * is correct in dev and in the production build alike. This must run before any map is
 * constructed.
 */
import maplibreWorkerUrl from 'maplibre-gl/dist/maplibre-gl-worker.mjs?worker&url';

import { App } from './App';
import './styles.css';

setWorkerUrl(maplibreWorkerUrl);

const container = document.getElementById('root');
if (container === null) {
  throw new Error('#root is missing from index.html');
}

createRoot(container).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
