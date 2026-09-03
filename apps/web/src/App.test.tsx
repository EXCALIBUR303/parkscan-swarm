import { render, screen, within } from '@testing-library/react';
import { afterEach, describe, expect, it, vi } from 'vitest';

// MapLibre GL 6 needs WebGL2 and jsdom has none, so the map primitives are replaced.
// See src/test/reactMapGlMock.tsx — real map behaviour is a Phase 5 Playwright job.
vi.mock('react-map-gl/maplibre', async () => await import('./test/reactMapGlMock'));

import { App } from './App';
import LOT_TEXT from '../public/lot.geojson?raw';

function stubLotFetch(mode: 'ok' | 'fail' | 'pending'): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(() => {
      if (mode === 'pending') return new Promise(() => undefined);
      if (mode === 'fail') return Promise.reject(new Error('network down'));
      return Promise.resolve({
        ok: true,
        status: 200,
        statusText: 'OK',
        json: () => Promise.resolve(JSON.parse(LOT_TEXT) as unknown),
      });
    }),
  );
}

afterEach(() => {
  vi.unstubAllGlobals();
});

describe('App shell', () => {
  it('shows an explicit loading state, not a blank panel', () => {
    stubLotFetch('pending');
    render(<App />);
    expect(screen.getByTestId('lot-loading')).toHaveTextContent('Loading lot geometry');
    expect(screen.getAllByText(/Loading lot geometry/)).not.toHaveLength(0);
  });

  it('shows an explicit, actionable error state', async () => {
    stubLotFetch('fail');
    render(<App />);
    const error = await screen.findByTestId('lot-error');
    expect(error).toHaveTextContent('Could not load the lot');
    expect(error).toHaveTextContent('network down');
    expect(within(error).getByRole('button', { name: 'Retry' })).toBeInTheDocument();
    // The map region states the failure too rather than rendering an empty canvas.
    expect(screen.getByText('The lot could not be drawn.')).toBeInTheDocument();
  });

  it('renders the badge from data_source and carries it into the title (H1, H2)', async () => {
    stubLotFetch('ok');
    render(<App />);
    await screen.findByTestId('lot-summary');
    expect(screen.getByTestId('simulated-badge')).toHaveAttribute('data-data-source', 'simulated');
    expect(screen.getByText('SIMULATED DATA — no physical robots')).toBeInTheDocument();
    expect(document.title).toBe('SIMULATED — ParkScan Swarm');
  });

  it('summarises the loaded lot', async () => {
    stubLotFetch('ok');
    render(<App />);
    const summary = await screen.findByTestId('lot-summary');
    expect(within(summary).getByText('Zones')).toBeInTheDocument();
    expect(within(summary).getByText('12')).toBeInTheDocument();
    expect(within(summary).getByText('EPSG:4326')).toBeInTheDocument();
  });

  it('declares the lot layers with no basemap layer (ADR-0004)', async () => {
    stubLotFetch('ok');
    render(<App />);
    await screen.findByTestId('lotmap');

    for (const id of ['zones', 'lanes', 'spaces', 'bays', 'outline']) {
      expect(screen.getByTestId(`source-${id}`)).toBeInTheDocument();
    }
    // Coverage is a fill choropleth, not a heatmap layer.
    expect(screen.getByTestId('layer-zone-fill')).toHaveAttribute('data-layer-type', 'fill');
    expect(screen.queryByTestId('layer-zone-heatmap')).toBeNull();
    // No raster source means no tile basemap could have crept in.
    expect(screen.queryByTestId('source-basemap')).toBeNull();
  });

  it('shows the coverage legend with explicit numeric breaks', async () => {
    stubLotFetch('ok');
    render(<App />);
    await screen.findByTestId('lotmap');
    for (const label of ['0%', '25%', '50%', '75%', '100%']) {
      expect(screen.getByText(label)).toBeInTheDocument();
    }
  });

  it('says why the fleet is empty instead of showing a bare zero', async () => {
    stubLotFetch('ok');
    render(<App />);
    const empty = await screen.findByTestId('fleet-empty');
    expect(empty).toHaveTextContent('No robot telemetry yet.');
    // The detail must explain WHY it is empty, not just that it is. Asserting on
    // 'Phase' keeps the test honest about the cause without pinning exact prose.
    expect(empty).toHaveTextContent(/Phase\s*2/);
    expect(empty).toHaveTextContent(/no robots publishing/i);
  });

  it('has landmarks and a skip link that points at a real target', async () => {
    stubLotFetch('ok');
    render(<App />);
    await screen.findByTestId('lot-summary');

    expect(screen.getByRole('banner')).toBeInTheDocument();
    expect(screen.getByRole('main')).toBeInTheDocument();
    expect(screen.getByRole('complementary')).toBeInTheDocument();
    expect(screen.getByRole('contentinfo')).toBeInTheDocument();

    const skip = screen.getByRole('link', { name: 'Skip to data table' });
    expect(skip).toHaveAttribute('href', '#data-table');
    expect(document.querySelector('#data-table')).not.toBeNull();
  });

  it('renders the accessible data table with an empty state', async () => {
    stubLotFetch('ok');
    render(<App />);
    await screen.findByTestId('lot-summary');

    const table = screen.getByRole('table');
    expect(within(table).getByText('No robots reporting.')).toBeInTheDocument();
    expect(within(table).getByRole('columnheader', { name: 'Robot' })).toBeInTheDocument();
    expect(
      within(table).getByRole('columnheader', { name: 'Coordinator view' }),
    ).toBeInTheDocument();
  });
});
