import { useRobotList } from '../store';

/**
 * The accessible equivalent of the map.
 *
 * WCAG 2.2 AA: a WebGL canvas is not readable by assistive technology, so the same
 * information is available as a real `<table>`. It is the target of the "skip to data
 * table" link, and it is where the fleet actually becomes inspectable without a mouse.
 *
 * Poses are shown in lot metres — the frame the rest of the system reasons in. Degrees
 * exist only inside the map.
 */
export function FleetTable(): React.JSX.Element {
  const robots = useRobotList();

  return (
    <section className="panel" id="data-table" aria-labelledby="data-table-heading" tabIndex={-1}>
      <h2 id="data-table-heading">Fleet data table</h2>
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
          {robots.length === 0 ? (
            <tr>
              <td colSpan={8} className="table__empty">
                No robots reporting.
              </td>
            </tr>
          ) : (
            robots.map((robot) => (
              <tr key={robot.id}>
                <th scope="row">{robot.id}</th>
                <td>{robot.state}</td>
                <td>{robot.coordinatorView}</td>
                <td>{`${String(Math.round(robot.batterySoc * 100))}%`}</td>
                <td>{robot.position.x.toFixed(2)}</td>
                <td>{robot.position.y.toFixed(2)}</td>
                <td>{robot.zoneId ?? '—'}</td>
                <td>{robot.epoch}</td>
              </tr>
            ))
          )}
        </tbody>
      </table>
    </section>
  );
}
