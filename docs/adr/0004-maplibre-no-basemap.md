# ADR-0004 — MapLibre GL 6, no basemap, choropleth not heatmap

**Status:** Accepted (Phase 0)

## Decision
MapLibre GL JS 6 + react-map-gl 8. No basemap. Coverage rendered as a per-zone choropleth.

## Rationale
**Over Leaflet:** Leaflet stable is 1.9.4 (May 2023); 2.0 has been in alpha since May 2025 with an
officially "unknown" release date; `react-leaflet` 5.0.0 is pinned to Leaflet 1.x; `leaflet.heat`'s
last release was **2015-10-26**. MapLibre shipped 5 releases in the last three weeks. Accepted cost:
~341 kB gz vs ~111 kB.

**No basemap:** a simulated lot does not exist in OpenStreetMap, so an OSM layer would draw the
robots on top of a real, wrong street — actively deceptive in a project that forbids overstated
claims. The lot renders from a checked-in `lot.geojson`, the same data the coverage planner consumes.

**Choropleth over heatmap:** a heatmap is a kernel-density estimate whose look is set by
`heatmap-radius` — an aesthetic knob, not data. It smears across zone boundaries and implies
continuous measurement never taken. Zone coverage is discrete and countable (cells swept / total).

## Consequences
- MapLibre is Web-Mercator-only with no `L.CRS.Simple` equivalent, so a ~30-line `src/lib/enu.ts`
  must be written and round-trip tested. Measured distortion over the 150 m lot is **< 3 mm**.
- MapLibre v6 is WebGL2 + ESM-only and **cannot render in jsdom**. Map behaviour is therefore
  verified in Playwright / Vitest Browser Mode; jsdom tests mock the map. Budget for this.
- TypeScript is pinned to **6.0.3**: `typescript-eslint`'s peer range is `>=4.8.4 <6.1.0` and TS 7
  shipped without a stable programmatic API, so TS 7 cannot install alongside it.
