#!/usr/bin/env node
/**
 * Copy the simulator's shared data files into `public/` so Vite serves them.
 *
 *   sim/lot/lot.geojson       -> /lot.geojson        (the lot the map draws)
 *   sim/lot/enu_reference.json -> /enu_reference.json (cross-language projection fixture)
 *
 * `sim/lot/` is the single source of truth — the same files the coverage planner and the
 * Python tests consume — but Vite can only serve static assets from inside `apps/web`, and
 * a symlink out of the package breaks `vite build` on a fresh CI checkout. So they are
 * copied, on `npm run dev` and `npm run build`.
 *
 * A missing source is a no-op (not an error): `public/` then keeps the committed
 * placeholder from `scripts/gen-placeholder-lot.mjs`. That is the only reason both exist.
 */

import { copyFileSync, existsSync, readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const SIM = resolve(HERE, '..', '..', '..', 'sim', 'lot');
const PUBLIC = resolve(HERE, '..', 'public');

/** Files mirrored from sim/lot into public/. */
const ASSETS = ['lot.geojson', 'enu_reference.json'];

let copied = 0;
for (const name of ASSETS) {
  const source = resolve(SIM, name);
  const dest = resolve(PUBLIC, name);

  if (!existsSync(source)) {
    console.log(`sync-lot: ${source} does not exist yet — keeping the committed placeholder.`);
    continue;
  }

  const before = existsSync(dest) ? readFileSync(dest, 'utf8') : null;
  copyFileSync(source, dest);
  const after = readFileSync(dest, 'utf8');

  if (before === after) {
    console.log(`sync-lot: public/${name} already matches ${source}`);
  } else {
    console.log(`sync-lot: copied ${source} -> public/${name}`);
    copied += 1;
  }
}

if (copied > 0) console.log(`sync-lot: ${copied} file(s) updated`);
