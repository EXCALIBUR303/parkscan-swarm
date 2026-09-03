import type { LoadedLot } from '../lib/lot';
import type { LotStatus } from '../lib/useLot';
import { useConnection, useRobotIds } from '../store';

/**
 * Side panel.
 *
 * Every branch is an explicit state with words in it. There are no blank divs here: a
 * panel that renders nothing is indistinguishable from a panel that is broken, and this
 * project's failure modes (no telemetry yet, API not running, lot file missing) all look
 * identical if the empty case is silent.
 */

export interface SidePanelProps {
  status: LotStatus;
  lot: LoadedLot | null;
  error: string | null;
  onRetry: () => void;
}

export function SidePanel({ status, lot, error, onRetry }: SidePanelProps): React.JSX.Element {
  return (
    <aside className="side" aria-labelledby="side-heading">
      <h2 id="side-heading">Run</h2>

      {status === 'loading' && (
        <p className="state state--loading" aria-busy="true" data-testid="lot-loading">
          Loading lot geometry…
        </p>
      )}

      {status === 'error' && (
        <div className="state state--error" role="alert" data-testid="lot-error">
          <p>
            <strong>Could not load the lot.</strong>
          </p>
          <p className="state__detail">{error ?? 'Unknown error.'}</p>
          <p className="state__detail">
            <code>public/lot.geojson</code> is copied from <code>sim/lot/lot.geojson</code> by{' '}
            <code>npm run sync:lot</code>, which runs on <code>dev</code> and <code>build</code>.
          </p>
          <button type="button" className="button" onClick={onRetry}>
            Retry
          </button>
        </div>
      )}

      {status === 'ready' && lot !== null && <LotSummary lot={lot} />}

      <FleetSection />
    </aside>
  );
}

function LotSummary({ lot }: { lot: LoadedLot }): React.JSX.Element {
  return (
    <div data-testid="lot-summary">
      <dl className="kv">
        <dt>Zones</dt>
        <dd>{lot.counts.zones}</dd>
        <dt>Parking spaces</dt>
        <dd>{lot.counts.parkingSpaces}</dd>
        <dt>Charging bays</dt>
        <dd>{lot.counts.chargingBays}</dd>
        <dt>Lane segments</dt>
        <dd>{lot.counts.lanes}</dd>
        <dt>Source CRS</dt>
        <dd>{lot.crs}</dd>
        <dt>Lot origin</dt>
        <dd>
          {lot.origin.latDeg.toFixed(4)}, {lot.origin.lonDeg.toFixed(4)}
        </dd>
      </dl>
      <p className="note">
        The origin is a synthetic anchor so PostGIS and MapLibre have somewhere to put a lot that
        does not exist. It is not a real address.
      </p>
    </div>
  );
}

/**
 * Fleet slice.
 *
 * Empty in Phase 1 and it says exactly why, rather than showing a plausible-looking zero.
 */
function FleetSection(): React.JSX.Element {
  const robotIds = useRobotIds();
  const { status, error } = useConnection();

  return (
    <section className="side__section" aria-labelledby="fleet-heading">
      <h3 id="fleet-heading">Fleet</h3>

      {status === 'error' ? (
        <div className="state state--error" role="alert" data-testid="fleet-error">
          <p>
            <strong>Telemetry connection failed.</strong>
          </p>
          <p className="state__detail">{error ?? 'Unknown error.'}</p>
        </div>
      ) : status === 'connecting' || status === 'reconnecting' ? (
        <p className="state state--loading" aria-busy="true" data-testid="fleet-loading">
          Connecting to the telemetry stream…
        </p>
      ) : robotIds.length === 0 ? (
        <div className="state state--empty" data-testid="fleet-empty">
          <p>No robot telemetry yet.</p>
          <p className="state__detail">
            The dashboard is not connected to a live feed. The MQTT → API → WebSocket path is Phase
            2; this shell renders lot geometry only.
          </p>
        </div>
      ) : (
        <ul className="fleet" data-testid="fleet-list">
          {robotIds.map((id) => (
            <li key={id} className="fleet__id">
              {id}
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
