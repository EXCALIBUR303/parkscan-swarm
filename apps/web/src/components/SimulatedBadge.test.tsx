import { fireEvent, render, screen, within } from '@testing-library/react';
import { describe, expect, it } from 'vitest';

import { BADGE_COPY } from '../lib/provenance';
import { SimulatedBadge } from './SimulatedBadge';

describe('SimulatedBadge', () => {
  it('renders the provenance banner for simulated data', () => {
    render(<SimulatedBadge dataSource="simulated" />);
    expect(screen.getByText('SIMULATED DATA — no physical robots')).toBeInTheDocument();
  });

  it('renders from the data, not from a hardcoded string (H1)', () => {
    // If the copy were baked in, these three renders would be identical.
    const { rerender } = render(<SimulatedBadge dataSource="simulated" />);
    expect(screen.getByTestId('simulated-badge')).toHaveAttribute('data-data-source', 'simulated');

    rerender(<SimulatedBadge dataSource="replay" />);
    expect(screen.getByText(BADGE_COPY.replay.full)).toBeInTheDocument();
    expect(screen.queryByText(BADGE_COPY.simulated.full)).not.toBeInTheDocument();

    rerender(<SimulatedBadge dataSource="live" />);
    expect(screen.getByText(BADGE_COPY.live.full)).toBeInTheDocument();
    expect(screen.getByTestId('simulated-badge')).toHaveAttribute('data-data-source', 'live');
  });

  it('puts the provenance in document.title (H2)', () => {
    render(<SimulatedBadge dataSource="simulated" />);
    expect(document.title).toBe('SIMULATED — ParkScan Swarm');
  });

  it('is a live region so it is announced, not silently painted (H2)', () => {
    render(<SimulatedBadge dataSource="simulated" />);
    const badge = screen.getByRole('status');
    expect(badge).toHaveAttribute('aria-live', 'polite');
    expect(badge).toHaveTextContent(BADGE_COPY.simulated.detail);
  });

  it('is non-dismissable: no control can remove it (H2)', () => {
    render(<SimulatedBadge dataSource="simulated" />);
    const badge = screen.getByTestId('simulated-badge');

    // No close button, no link, nothing focusable that could hide it.
    expect(within(badge).queryByRole('button')).toBeNull();
    expect(within(badge).queryByRole('link')).toBeNull();
    expect(badge.querySelectorAll('button, a, [role="button"]')).toHaveLength(0);

    // The usual dismissal gestures do nothing.
    fireEvent.click(badge);
    fireEvent.keyDown(badge, { key: 'Escape' });
    fireEvent.keyDown(document.body, { key: 'Escape' });
    expect(screen.getByTestId('simulated-badge')).toBeInTheDocument();
    expect(screen.getByText(BADGE_COPY.simulated.full)).toBeVisible();
  });
});
