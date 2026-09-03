import type { LoadedLot } from '../lib/lot';
import { CONNECTION_LABEL, batteryPercent, capitalise, humaniseId } from '../lib/robot';
import type { LotStatus } from '../lib/useLot';
import {
  useBatteryBand,
  useConnection,
  useRobot,
  useRobotIds,
  useRunId,
  useStreamLoss,
} from '../store';

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

      <ConnectionIndicator />

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

/**
 * Liveness of `/ws/live`, always visible.
 *
 * Not folded into the fleet's empty state: "no robots" and "no connection" are different
 * facts with different fixes, and a dashboard that shows one when it means the other
 * wastes the operator's time. The dropped-frame counter is here too — the server's
 * per-client queue is bounded at 100 with drop-oldest and reports the loss
 * (docs/API_CONTRACT.md), so hiding that number would turn a visibly lossy stream into
 * an invisibly wrong one.
 */
function ConnectionIndicator(): React.JSX.Element {
  const { status, error } = useConnection();
  const { dropped, malformed } = useStreamLoss();
  const runId = useRunId();

  return (
    <div className={`conn conn--${status}`} data-testid="connection-status" data-status={status}>
      <span className="conn__dot" aria-hidden="true" />
      <span className="conn__label">
        Telemetry stream: <strong>{CONNECTION_LABEL[status]}</strong>
      </span>
      {runId !== null && <span className="conn__run">run {runId}</span>}
      {error !== null && <span className="conn__detail">{error}</span>}
      {dropped > 0 && (
        <span className="conn__detail" data-testid="dropped-frames">
          {dropped} frame{dropped === 1 ? '' : 's'} dropped by the server queue
        </span>
      )}
      {malformed > 0 && (
        <span className="conn__detail" data-testid="malformed-frames">
          {malformed} frame{malformed === 1 ? '' : 's'} this client could not decode
        </span>
      )}
    </div>
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
 * Subscribes only to the **id list**, and each card subscribes to its own robot, so a
 * pose arriving for `robot_1` re-renders one card rather than the panel.
 */
function FleetSection(): React.JSX.Element {
  const robotIds = useRobotIds();
  const { status, error } = useConnection();

  return (
    <section className="side__section" aria-labelledby="fleet-heading">
      <h3 id="fleet-heading">Fleet</h3>

      {robotIds.length > 0 ? (
        <ul className="fleet" data-testid="fleet-list">
          {robotIds.map((id) => (
            <RobotCard key={id} robotId={id} />
          ))}
        </ul>
      ) : status === 'error' ? (
        <div className="state state--error" role="alert" data-testid="fleet-error">
          <p>
            <strong>Telemetry connection failed.</strong>
          </p>
          <p className="state__detail">{error ?? 'Unknown error.'}</p>
          <p className="state__detail">
            The dashboard proxies <code>/ws</code> to the API on <code>127.0.0.1:8000</code>. If the
            API is not running, start it with <code>make api</code>.
          </p>
        </div>
      ) : status === 'connecting' || status === 'reconnecting' || status === 'idle' ? (
        <p className="state state--loading" aria-busy="true" data-testid="fleet-loading">
          Connecting to the telemetry stream…
        </p>
      ) : (
        <div className="state state--empty" data-testid="fleet-empty">
          <p>No robot telemetry yet.</p>
          <p className="state__detail">
            The stream is {CONNECTION_LABEL[status]} and nothing is publishing to{' '}
            <code>parkscan/v1/&#123;run&#125;/robot/+/telemetry</code>. Robots appear here the
            moment a sample arrives; this panel stays empty rather than showing a plausible-looking
            zero.
          </p>
        </div>
      )}
    </section>
  );
}

/**
 * One robot card.
 *
 * The heading is the same sentence the map marker uses for its accessible name, so the
 * two views cannot describe the same robot differently.
 */
function RobotCard({ robotId }: { robotId: string }): React.JSX.Element | null {
  const robot = useRobot(robotId);
  const band = useBatteryBand(robotId);
  if (robot === undefined || band === undefined) return null;

  return (
    <li className="fleet__card" data-testid={`robot-card-${robotId}`}>
      <h4 className="fleet__id">{capitalise(humaniseId(robot.id))}</h4>
      <dl className="kv kv--tight">
        <dt>Reported state</dt>
        <dd>{robot.state.replace(/_/g, ' ')}</dd>
        <dt>Coordinator view</dt>
        <dd>{robot.coordinatorView ?? 'not reported'}</dd>
        <dt>Battery</dt>
        <dd>
          <span className={`band band--${band}`}>{batteryPercent(robot.batterySoc)}%</span>{' '}
          <span className="band__word">({band})</span>
        </dd>
        <dt>Position</dt>
        <dd>
          {robot.position.x.toFixed(1)}, {robot.position.y.toFixed(1)} m
        </dd>
        <dt>Zone</dt>
        <dd>{robot.zoneId ?? '—'}</dd>
        <dt>Epoch</dt>
        <dd>{robot.epoch}</dd>
      </dl>
    </li>
  );
}
