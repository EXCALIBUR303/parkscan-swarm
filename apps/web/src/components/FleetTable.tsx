import { batteryPercent } from '../lib/robot';
import { useFleetAnnouncer } from '../lib/useFleetAnnouncer';
import { useBatteryBand, useRobot, useRobotIds } from '../store';

/**
 * The accessible equivalent of the map.
 *
 * WCAG 2.2 AA: a WebGL canvas is not readable by assistive technology, so the same
 * information is available as a real `<table>`. It is the target of the "skip to data
 * table" link, and it is where the fleet actually becomes inspectable without a mouse.
 *
 * Poses are shown in lot metres — the frame the rest of the system reasons in. Degrees
 * exist only inside the map.
 *
 * THE LIVE REGION
 * ---------------
 * One polite region, and it announces **state changes only**: a robot changed state, the
 * coordinator changed its judgement, a battery entered a low band, a robot started or
 * stopped reporting, or the stream dropped. It never announces a pose. At 3 robots x
 * 10 Hz a pose-driven live region emits thirty announcements a second, which does not
 * inform a screen-reader user — it locks them out of the page. See `lib/announce.ts`:
 * the announcer is subscribed to a signal that structurally cannot contain a coordinate,
 * and the messages inside one debounce window are coalesced into a single sentence.
 *
 * The table body itself still updates at full rate; only the *announcement* is throttled.
 */
export function FleetTable(): React.JSX.Element {
  const robotIds = useRobotIds();
  const announcement = useFleetAnnouncer();

  return (
    <section className="panel" id="data-table" aria-labelledby="data-table-heading" tabIndex={-1}>
      <h2 id="data-table-heading">Fleet data table</h2>

      <p
        className="visually-hidden"
        role="status"
        aria-live="polite"
        aria-atomic="true"
        data-testid="fleet-announcer"
      >
        {announcement}
      </p>

      <table className="table">
        <caption className="table__caption">
          Robot state, battery and pose in lot metres. Equivalent to the map above.
        </caption>
        <thead>
          <tr>
            <th scope="col">Robot</th>
            <th scope="col">Reported state</th>
            <th scope="col">Coordinator view</th>
            <th scope="col">Battery</th>
            <th scope="col">x (m)</th>
            <th scope="col">y (m)</th>
            <th scope="col">Zone</th>
            <th scope="col">Epoch</th>
          </tr>
        </thead>
        <tbody>
          {robotIds.length === 0 ? (
            <tr>
              <td colSpan={8} className="table__empty">
                No robots reporting.
              </td>
            </tr>
          ) : (
            robotIds.map((id) => <FleetRow key={id} robotId={id} />)
          )}
        </tbody>
      </table>
    </section>
  );
}

/**
 * One row, subscribed to one robot.
 *
 * Same reason as the marker and the card: `robot_1` moving must not re-render
 * `robot_2`'s row.
 */
function FleetRow({ robotId }: { robotId: string }): React.JSX.Element | null {
  const robot = useRobot(robotId);
  const band = useBatteryBand(robotId);
  if (robot === undefined || band === undefined) return null;

  return (
    <tr data-testid={`fleet-row-${robotId}`}>
      <th scope="row">{robot.id}</th>
      <td>{robot.state}</td>
      {/* `null` means the coordinator has published no judgement — not "online". */}
      <td>{robot.coordinatorView ?? 'not reported'}</td>
      <td className={`band band--${band}`}>{`${String(batteryPercent(robot.batterySoc))}%`}</td>
      <td>{robot.position.x.toFixed(2)}</td>
      <td>{robot.position.y.toFixed(2)}</td>
      <td>{robot.zoneId ?? '—'}</td>
      <td>{robot.epoch}</td>
    </tr>
  );
}
