/**
 * The live telemetry socket: `ws://<origin>/ws/live`.
 *
 * THREE THINGS THIS FILE IS RESPONSIBLE FOR
 * -----------------------------------------
 * 1. **Staying connected.** `partysocket` gives reconnection with exponential backoff
 *    and an outgoing send buffer, so a `send()` issued while the socket is down is
 *    flushed before the next `open` event rather than thrown away. The subscribe frame
 *    is re-sent on *every* open, because the server allocates a fresh `ClientChannel`
 *    per connection and a subscription made before a drop does not survive it.
 *
 * 2. **Not melting React.** Telemetry arrives at 1-10 Hz per robot. Writing each frame
 *    straight into a store would commit three times per tick and re-render the tree on
 *    every one. Frames are pushed into a plain mutable array and handed to the store
 *    **once per `requestAnimationFrame`**, so a burst of N frames becomes exactly one
 *    commit (docs/ARCHITECTURE.md §7). `FrameBuffer` takes its clock by injection, which
 *    is what makes that coalescing directly testable rather than asserted.
 *
 * 3. **Not lying about what arrived.** The server's per-client queue is bounded at 100
 *    with drop-oldest, and it announces the loss as `{"type":"lag","dropped":N}` before
 *    the next real frame (docs/API_CONTRACT.md). That frame is counted and surfaced in
 *    the UI. Swallowing it would turn a visibly lossy stream into an invisibly wrong one.
 *    Frames that fail to parse are counted too, for the same reason.
 *
 * The wire format is the single envelope from docs/API_CONTRACT.md:
 *
 *     {"type":"telemetry|detection|zone|alert|assignment|run_state",
 *      "stamp":"<iso8601>","data_source":"simulated","payload":{...}}
 *
 * with two exceptions this decoder handles explicitly: the `lag` notice and the server's
 * `{"type":"error","detail":...}` reply to a malformed client frame, neither of which
 * carries the full envelope.
 */

import ReconnectingWebSocket from 'partysocket/ws';

import type { ConnectionStatus, RobotView } from './robot';
import {
  isCoordinatorView,
  isDataSource,
  isRobotState,
  type CoordinatorView,
  type DataSource,
} from './types';

export type { ConnectionStatus } from './robot';

/* --------------------------------------------------------------------- events -- */

/** One robot telemetry sample, already in the shape the store holds. */
export interface TelemetryEvent {
  readonly kind: 'telemetry';
  readonly robot: RobotView;
}

/** The server's greeting (and any later run-state change). */
export interface RunStateEvent {
  readonly kind: 'run_state';
  readonly runId: string | null;
  readonly dataSource: DataSource;
}

/** The bounded-queue overflow notice. `dropped` is frames lost since the last notice. */
export interface LagEvent {
  readonly kind: 'lag';
  readonly dropped: number;
}

/** The server rejected something we sent. Visible, never silent. */
export interface ServerErrorEvent {
  readonly kind: 'server_error';
  readonly detail: string;
}

/** A frame that did not decode. Counted so a schema drift shows up as a number. */
export interface MalformedEvent {
  readonly kind: 'malformed';
  readonly detail: string;
}

export interface DetectionView {
  id: string;
  robotId: string;
  kind: string;
  /** Provenance, straight from the server. The SIMULATED styling reads THIS. */
  detSource: string;
  detectorName: string;
  confidence: number;
  severity: 'low' | 'medium' | 'high' | null;
  /** Lot metres. Present only when the geolocation was accepted. */
  position: { x: number; y: number } | null;
  rangeM: number;
  geolocValid: boolean;
  receivedAt: number;
}

export interface DetectionEvent {
  kind: 'detection';
  detection: DetectionView;
}

export type LiveEvent =
  TelemetryEvent | RunStateEvent | DetectionEvent | LagEvent | ServerErrorEvent | MalformedEvent;

/* -------------------------------------------------------------------- parsing -- */

const SEVERITIES = new Set(['low', 'medium', 'high']);

/**
 * One detection off the wire.
 *
 * `local_x`/`local_y` are present only when the server accepted the geolocation. A
 * rejected ray has NO world position, and inventing one here — snapping it to the robot,
 * say — would put a confident, wrong pothole on the map. That is the exact failure
 * criterion C11 exists to prevent, so a rejected detection carries `position: null` and
 * is listed but never plotted.
 */
function parseDetection(payload: Record<string, unknown>, now: () => number): LiveEvent {
  const id = asString(payload['id']);
  const robotId = asString(payload['robot_id']);
  const kind = asString(payload['kind']);
  const detSource = asString(payload['det_source']);
  if (id === null || robotId === null || kind === null || detSource === null) {
    return { kind: 'malformed', detail: 'detection missing id/robot_id/kind/det_source' };
  }

  const x = asFiniteNumber(payload['local_x']);
  const y = asFiniteNumber(payload['local_y']);
  const geolocValid = payload['geoloc_valid'] === true;
  const severity = asString(payload['severity']);

  return {
    kind: 'detection',
    detection: {
      id,
      robotId,
      kind,
      detSource,
      detectorName: asString(payload['detector_name']) ?? 'unknown',
      confidence: asFiniteNumber(payload['confidence']) ?? 0,
      severity:
        severity !== null && SEVERITIES.has(severity)
          ? (severity as 'low' | 'medium' | 'high')
          : null,
      position: geolocValid && x !== null && y !== null ? { x, y } : null,
      rangeM: asFiniteNumber(payload['range_m']) ?? Number.NaN,
      geolocValid,
      receivedAt: now(),
    },
  };
}

function asRecord(value: unknown): Record<string, unknown> | null {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
    ? (value as Record<string, unknown>)
    : null;
}

function asFiniteNumber(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}

function asString(value: unknown): string | null {
  return typeof value === 'string' ? value : null;
}

/**
 * ISO 8601 -> epoch milliseconds.
 *
 * A stamp that will not parse falls back to the wall clock, and the fallback is
 * confined to this one display-only field: `stampMs` is shown as "last update" and is
 * never used for ordering, deduplication or any metric. Nothing in `make bench` reads it.
 */
function stampToMs(value: unknown, now: () => number): number {
  const text = asString(value);
  if (text !== null) {
    const parsed = Date.parse(text);
    if (!Number.isNaN(parsed)) return parsed;
  }
  return now();
}

function parseTelemetry(
  envelope: Record<string, unknown>,
  payload: Record<string, unknown>,
  now: () => number,
): LiveEvent {
  const id = asString(payload['robot_id']);
  const x = asFiniteNumber(payload['local_x']);
  const y = asFiniteNumber(payload['local_y']);
  const soc = asFiniteNumber(payload['battery_soc']);
  const state: unknown = payload['state'];

  if (id === null || x === null || y === null || soc === null || !isRobotState(state)) {
    return { kind: 'malformed', detail: 'telemetry payload is missing required fields' };
  }

  const coordinatorRaw: unknown = payload['coordinator_view'];
  const coordinatorView: CoordinatorView | null = isCoordinatorView(coordinatorRaw)
    ? coordinatorRaw
    : null;

  // Provenance travels with the sample. The payload's own value wins over the
  // envelope's, and an unrecognised value is not quietly downgraded to 'simulated' —
  // it makes the frame malformed, because a wrong badge is worse than no frame (H1).
  const payloadSource: unknown = payload['data_source'];
  const envelopeSource: unknown = envelope['data_source'];
  const declared = payloadSource === undefined ? envelopeSource : payloadSource;
  if (!isDataSource(declared)) {
    return { kind: 'malformed', detail: 'telemetry frame carries no usable data_source' };
  }

  const epoch = asFiniteNumber(payload['epoch']);
  const heading = asFiniteNumber(payload['heading_rad']);

  return {
    kind: 'telemetry',
    robot: {
      id,
      state,
      coordinatorView,
      batterySoc: soc,
      position: { x, y },
      headingRad: heading ?? 0,
      zoneId: asString(payload['zone_id']),
      epoch: epoch === null ? 0 : Math.trunc(epoch),
      stampMs: stampToMs(payload['stamp'] ?? envelope['stamp'], now),
      dataSource: declared,
    },
  };
}

/**
 * Decode one server frame.
 *
 * Never throws: a malformed frame becomes a `malformed` event so the count is visible.
 */
export function parseLiveMessage(raw: string, now: () => number = Date.now): LiveEvent {
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch {
    return { kind: 'malformed', detail: 'frame is not JSON' };
  }

  const envelope = asRecord(parsed);
  if (envelope === null) return { kind: 'malformed', detail: 'frame is not an object' };

  const type = asString(envelope['type']);
  if (type === null) return { kind: 'malformed', detail: 'frame has no type' };

  // The lag notice is deliberately not a full envelope (apps/api ws.py `_drain`).
  if (type === 'lag') {
    const dropped = asFiniteNumber(envelope['dropped']);
    return { kind: 'lag', dropped: dropped === null ? 0 : Math.trunc(dropped) };
  }

  if (type === 'error') {
    return { kind: 'server_error', detail: asString(envelope['detail']) ?? 'unspecified' };
  }

  const payload = asRecord(envelope['payload']);
  if (payload === null) {
    return { kind: 'malformed', detail: `${type} frame has no payload object` };
  }

  if (type === 'telemetry') return parseTelemetry(envelope, payload, now);

  if (type === 'run_state') {
    const source: unknown = envelope['data_source'];
    return {
      kind: 'run_state',
      runId: asString(payload['run_id']),
      dataSource: isDataSource(source) ? source : 'simulated',
    };
  }

  if (type === 'detection') return parseDetection(payload, now);

  // zone / alert / assignment arrive in Phase 4. Ignoring them is a decision, not an
  // oversight, so they are counted as malformed rather than dropped on the floor.
  return { kind: 'malformed', detail: `no handler for frame type ${type}` };
}

/* ---------------------------------------------------------------- frame buffer -- */

/**
 * The scheduler the buffer flushes on. Injected so tests drive it by hand instead of
 * waiting on a real animation frame.
 */
export interface FlushClock {
  readonly schedule: (callback: () => void) => number;
  readonly cancel: (handle: number) => void;
}

/** Used only where `requestAnimationFrame` is absent (non-visual test environments). */
const FALLBACK_FRAME_MS = 16;

export const animationFrameClock: FlushClock = {
  schedule: (callback) =>
    typeof requestAnimationFrame === 'function'
      ? requestAnimationFrame(() => {
          callback();
        })
      : (setTimeout(callback, FALLBACK_FRAME_MS) as unknown as number),
  cancel: (handle) => {
    if (typeof cancelAnimationFrame === 'function') cancelAnimationFrame(handle);
    else clearTimeout(handle);
  },
};

/**
 * Collects events and releases them in one batch per scheduled frame.
 *
 * This is the whole reason the dashboard stays smooth at 3 robots x 10 Hz: without it,
 * 30 store writes a second each trigger their own React commit.
 */
export class FrameBuffer {
  private readonly pending: LiveEvent[] = [];
  private handle: number | null = null;
  private disposed = false;

  constructor(
    private readonly onFlush: (batch: readonly LiveEvent[]) => void,
    private readonly clock: FlushClock = animationFrameClock,
  ) {}

  /** Events waiting for the next frame. */
  get size(): number {
    return this.pending.length;
  }

  push(event: LiveEvent): void {
    if (this.disposed) return;
    this.pending.push(event);
    if (this.handle !== null) return;
    this.handle = this.clock.schedule(() => {
      this.handle = null;
      this.flush();
    });
  }

  /** Release everything queued as a single batch. A no-op when nothing is queued. */
  flush(): void {
    if (this.pending.length === 0) return;
    const batch = this.pending.splice(0, this.pending.length);
    this.onFlush(batch);
  }

  /** Cancel the pending frame and drop anything not yet flushed. */
  dispose(): void {
    this.disposed = true;
    if (this.handle !== null) {
      this.clock.cancel(this.handle);
      this.handle = null;
    }
    this.pending.length = 0;
  }
}

/* --------------------------------------------------------------------- socket -- */

/**
 * Topics we actually consume today.
 *
 * The server treats "no subscribe" as "send everything"; naming only the frame types the
 * dashboard has a renderer for keeps the rest off the wire instead of receiving it and
 * counting it as unhandled. `detection` joined the list in Phase 3, when the map gained
 * severity pins and the panel gained a list. `zone`, `alert` and `assignment` stay off
 * until Phase 4 renders them.
 */
export const DEFAULT_TOPICS: readonly string[] = ['telemetry', 'run_state', 'detection'];

export interface LiveSocketOptions {
  /** Defaults to `/ws/live` on the page's own origin (the Vite dev proxy handles it). */
  readonly url?: string;
  readonly topics?: readonly string[];
  readonly clock?: FlushClock;
  /** WebSocket implementation, for tests. Defaults to the global one. */
  readonly webSocketImpl?: unknown;
  readonly minReconnectionDelayMs?: number;
  readonly maxReconnectionDelayMs?: number;
  readonly connectionTimeoutMs?: number;
  /** Injected so `stampMs` has no hidden dependency on the wall clock. */
  readonly now?: () => number;
}

export interface LiveSocketHandlers {
  /** One coalesced batch per animation frame. */
  readonly onBatch: (batch: readonly LiveEvent[]) => void;
  readonly onStatus: (status: ConnectionStatus, error: string | null) => void;
}

/** WebSocket close code for a normal, intentional close (RFC 6455 §7.4.1). */
const CLOSE_NORMAL = 1000;

const DEFAULTS = {
  minReconnectionDelayMs: 500,
  maxReconnectionDelayMs: 8000,
  connectionTimeoutMs: 5000,
} as const;

/**
 * Derive the WebSocket URL from the page's own origin.
 *
 * Same-origin on purpose: `vite.config.ts` proxies `/ws` to the API in dev, and in a
 * built deployment the dashboard is served next to it. Nothing here hardcodes a port.
 */
export function defaultLiveUrl(location: { protocol: string; host: string }): string {
  return `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws/live`;
}

/**
 * A connected `/ws/live` client.
 *
 * Owns the socket, the frame buffer and the status callbacks. `close()` is final: it
 * stops reconnection, cancels the pending frame and drops anything unflushed.
 */
export class LiveSocket {
  private readonly socket: ReconnectingWebSocket;
  private readonly buffer: FrameBuffer;
  private readonly topics: readonly string[];
  private closed = false;

  constructor(handlers: LiveSocketHandlers, options: LiveSocketOptions = {}) {
    const now = options.now ?? Date.now;
    this.topics = options.topics ?? DEFAULT_TOPICS;
    this.buffer = new FrameBuffer(handlers.onBatch, options.clock);

    const url =
      options.url ??
      (typeof window === 'undefined'
        ? 'ws://127.0.0.1:8000/ws/live'
        : defaultLiveUrl(window.location));

    handlers.onStatus('connecting', null);

    this.socket = new ReconnectingWebSocket(url, undefined, {
      WebSocket: options.webSocketImpl,
      minReconnectionDelay: options.minReconnectionDelayMs ?? DEFAULTS.minReconnectionDelayMs,
      maxReconnectionDelay: options.maxReconnectionDelayMs ?? DEFAULTS.maxReconnectionDelayMs,
      connectionTimeout: options.connectionTimeoutMs ?? DEFAULTS.connectionTimeoutMs,
    });

    this.socket.addEventListener('open', () => {
      // Re-sent on every open: the server allocates a fresh channel per connection, so
      // a subscription from before a drop does not carry over.
      this.send({ type: 'subscribe', topics: [...this.topics] });
      handlers.onStatus('open', null);
    });

    this.socket.addEventListener('message', (event) => {
      const data: unknown = event.data;
      if (typeof data !== 'string') {
        this.buffer.push({ kind: 'malformed', detail: 'binary frame' });
        return;
      }
      this.buffer.push(parseLiveMessage(data, now));
    });

    this.socket.addEventListener('error', (event) => {
      if (this.closed) return;
      handlers.onStatus('error', event.message === '' ? 'connection failed' : event.message);
    });

    this.socket.addEventListener('close', (event) => {
      if (this.closed) {
        handlers.onStatus('closed', null);
        return;
      }
      handlers.onStatus(
        this.socket.shouldReconnect ? 'reconnecting' : 'closed',
        event.reason === '' ? null : event.reason,
      );
    });
  }

  /**
   * Send one client frame.
   *
   * The socket is read-mostly by design (commands go through REST so they are logged
   * and testable), so in practice this only ever carries `subscribe`. When the socket is
   * down the message is buffered by partysocket and flushed before the next `open`.
   */
  send(message: Record<string, unknown>): void {
    this.socket.send(JSON.stringify(message));
  }

  /** Flush any buffered frames immediately, without waiting for the next frame. */
  flush(): void {
    this.buffer.flush();
  }

  get status(): number {
    return this.socket.readyState;
  }

  close(): void {
    this.closed = true;
    this.buffer.dispose();
    this.socket.close(CLOSE_NORMAL, 'dashboard unmounted');
  }
}
