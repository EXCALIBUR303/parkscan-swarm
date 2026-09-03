import { FleetTable } from './components/FleetTable';
import { LotMap } from './components/LotMap';
import { SidePanel } from './components/SidePanel';
import { SimulatedBadge } from './components/SimulatedBadge';
import { APP_NAME } from './lib/provenance';
import { useLot } from './lib/useLot';
import { useDataSource } from './store';

/**
 * Dashboard shell.
 *
 * Landmarks are real elements (`header`/`main`/`aside`/`footer`), not divs with ARIA
 * bolted on, and the "skip to data table" link targets a section that actually exists —
 * a skip link pointing at nothing is worse than none at all.
 *
 * `data_source` is read from the store, which `useLot` populated from the file's own
 * metadata. Nothing in this tree hardcodes the word "simulated" as the source of truth.
 */
export function App(): React.JSX.Element {
  const { status, lot, error, reload } = useLot();
  const dataSource = useDataSource();

  return (
    <div className="app">
      <a className="skip-link" href="#data-table">
        Skip to data table
      </a>

      <header className="header">
        <div className="header__titles">
          <h1 className="header__title">{APP_NAME}</h1>
          <p className="header__subtitle">
            Coordinated fleet of 3 robots, centrally allocated — not a swarm.
          </p>
        </div>
        <SimulatedBadge dataSource={dataSource} />
      </header>

      <main className="main">
        <section className="mapregion" aria-labelledby="map-heading">
          <h2 className="visually-hidden" id="map-heading">
            Lot map
          </h2>

          {status === 'ready' && lot !== null ? (
            <LotMap lot={lot} />
          ) : status === 'error' ? (
            <div className="mapregion__state state state--error" role="alert">
              <p>
                <strong>The lot could not be drawn.</strong>
              </p>
              <p className="state__detail">{error ?? 'Unknown error.'}</p>
            </div>
          ) : (
            <p className="mapregion__state state state--loading" aria-busy="true">
              Loading lot geometry…
            </p>
          )}
        </section>

        <SidePanel status={status} lot={lot} error={error} onRetry={reload} />
      </main>

      <FleetTable />

      <footer className="footer">
        <p>
          No basemap by design: the lot is synthetic, so a real street map underneath it would place
          simulated robots on a real, wrong street (ADR-0004).
        </p>
      </footer>
    </div>
  );
}
