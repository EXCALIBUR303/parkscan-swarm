import '@testing-library/jest-dom/vitest';

import { cleanup } from '@testing-library/react';
import { afterEach } from 'vitest';

import { useParkscanStore } from '../store';

afterEach(() => {
  cleanup();
  // The store is a module singleton; without this, state leaks between tests.
  useParkscanStore.getState().reset();
});
