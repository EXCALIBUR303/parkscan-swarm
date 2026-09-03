import { useEffect } from 'react';

import { APP_NAME, BADGE_COPY } from '../lib/provenance';
import type { DataSource } from '../lib/types';

/**
 * Provenance banner.
 *
 * ACCEPTANCE H1/H2: this renders **from** `data_source`, never from a hardcoded string,
 * and it is persistent and non-dismissable. There is deliberately no close button, no
 * timeout and no collapsed variant — if you are looking for one, the rule you are about
 * to break is the point of the project.
 *
 * The same value is written into `document.title`, so the tab itself carries the label
 * even when the page is screenshotted or embedded.
 */
export interface SimulatedBadgeProps {
  /** Comes from the API / lot metadata, never from a literal in this component. */
  dataSource: DataSource;
}

export function SimulatedBadge({ dataSource }: SimulatedBadgeProps): React.JSX.Element {
  const copy = BADGE_COPY[dataSource];

  useEffect(() => {
    document.title = `${copy.short} — ${APP_NAME}`;
  }, [copy.short]);

  return (
    <div
      className={`badge badge--${dataSource}`}
      data-data-source={dataSource}
      data-testid="simulated-badge"
      role="status"
      aria-live="polite"
    >
      <span className="badge__mark" aria-hidden="true">
        ●
      </span>
      <strong className="badge__text">{copy.full}</strong>
      <span className="badge__detail">{copy.detail}</span>
    </div>
  );
}
