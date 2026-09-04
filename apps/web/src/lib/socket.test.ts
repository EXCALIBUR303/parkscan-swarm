import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';

import {
  FrameBuffer,
  LiveSocket,
  defaultLiveUrl,
  parseLiveMessage,
  type FlushClock,
  type LiveEvent,
  DEFAULT_TOPICS,
} from './socket';
import type { ConnectionStatus } from './robot';
import { FakeWebSocket } from '../test/fakeWebSocket';

/** A frame clock the test steps by hand, so "once per frame" is an assertion. */
class ManualClock implements FlushClock {
  private readonly queue = new Map<number, () => void>();
  private nextHandle = 1;

  readonly schedule = (callback: () => void): number => {
    const handle = this.nextHandle++;
    this.queue.set(handle, callback);
    return handle;
  };

  readonly cancel = (handle: number): void => {
    this.queue.delete(handle);
  };

  get pending(): number {
    return this.queue.size;
  }

  /** Run every callback scheduled so far, as one frame would. */
  tick(): void {
    const callbacks = [...this.queue.values()];
    this.queue.clear();
    for (const callback of callbacks) callback();
  }
}

function telemetryFrame(overrides: Record<string, unknown> = {}): string {
  return JSON.stringify({
    type: 'telemetry',
    stamp: '2026-09-03T10:00:00Z',
    data_source: 'simulated',
    payload: {
      run_id: 'dev',
      robot_id: 'robot_1',
      stamp: '2026-09-03T10:00:00Z',
      local_x: 42,
      local_y: 17.5,
      heading_rad: 1.57,
      battery_soc: 0.63,
      state: 'inspecting',
      coordinator_view: 'online',
      zone_id: 'zone_05',
      epoch: 3,
      data_source: 'simulated',
      ...overrides,
    },
  });
}

const FIXED_NOW = (): number => 1_700_000_000_000;

describe('parseLiveMessage', () => {
  it('decodes a telemetry envelope into a robot view in lot metres', () => {
    const event = parseLiveMessage(telemetryFrame(), FIXED_NOW);
    expect(event.kind).toBe('telemetry');
    if (event.kind !== 'telemetry') return;

    expect(event.robot).toEqual({
      id: 'robot_1',
      state: 'inspecting',
      coordinatorView: 'online',
      batterySoc: 0.63,
      position: { x: 42, y: 17.5 },
      headingRad: 1.57,
      zoneId: 'zone_05',
      epoch: 3,
      stampMs: Date.parse('2026-09-03T10:00:00Z'),
      dataSource: 'simulated',
    });
  });

  it('takes provenance from the frame, never from a default (H1)', () => {
    const event = parseLiveMessage(telemetryFrame({ data_source: 'replay' }), FIXED_NOW);
    expect(event.kind === 'telemetry' && event.robot.dataSource).toBe('replay');
  });

  it('rejects a frame whose data_source is not a known value', () => {
    const event = parseLiveMessage(telemetryFrame({ data_source: 'totally_real' }), FIXED_NOW);
    expect(event.kind).toBe('malformed');
  });

  it('leaves coordinator_view null rather than inventing "online"', () => {
    const event = parseLiveMessage(telemetryFrame({ coordinator_view: null }), FIXED_NOW);
    expect(event.kind === 'telemetry' && event.robot.coordinatorView).toBeNull();
  });

  it('rejects an unknown robot state instead of widening the enum', () => {
    const event = parseLiveMessage(telemetryFrame({ state: 'vibing' }), FIXED_NOW);
    expect(event.kind).toBe('malformed');
  });

  it('decodes the bare lag notice, which is not a full envelope', () => {
    expect(parseLiveMessage('{"type":"lag","dropped":7}', FIXED_NOW)).toEqual({
      kind: 'lag',
      dropped: 7,
    });
  });

  it('decodes the run_state greeting the server sends on connect', () => {
    const raw = JSON.stringify({
      type: 'run_state',
      stamp: '2026-09-03T10:00:00Z',
      data_source: 'simulated',
      payload: { run_id: 'dev', connected: true },
    });
    expect(parseLiveMessage(raw, FIXED_NOW)).toEqual({
      kind: 'run_state',
      runId: 'dev',
      dataSource: 'simulated',
    });
  });

  it('surfaces the server error reply rather than dropping it', () => {
    const raw = '{"type":"error","detail":"only subscribe is accepted"}';
    expect(parseLiveMessage(raw, FIXED_NOW)).toEqual({
      kind: 'server_error',
      detail: 'only subscribe is accepted',
    });
  });

  it('never throws on garbage', () => {
    expect(parseLiveMessage('not json', FIXED_NOW).kind).toBe('malformed');
    expect(parseLiveMessage('[1,2,3]', FIXED_NOW).kind).toBe('malformed');
    expect(parseLiveMessage('{"type":"telemetry"}', FIXED_NOW).kind).toBe('malformed');
  });

  it('counts frame types it has no renderer for instead of pretending to handle them', () => {
    const raw = JSON.stringify({
      type: 'detection',
      stamp: '2026-09-03T10:00:00Z',
      data_source: 'simulated',
      payload: {},
    });
    expect(parseLiveMessage(raw, FIXED_NOW).kind).toBe('malformed');
  });
});

describe('FrameBuffer', () => {
  it('coalesces a burst into exactly one flush', () => {
    const clock = new ManualClock();
    const flushes: (readonly LiveEvent[])[] = [];
    const buffer = new FrameBuffer((batch) => flushes.push(batch), clock);

    for (let i = 0; i < 30; i += 1) {
      const event = parseLiveMessage(telemetryFrame({ local_x: i }), FIXED_NOW);
      buffer.push(event);
    }

    // Nothing has reached the consumer yet: 30 frames, zero commits.
    expect(flushes).toHaveLength(0);
    expect(buffer.size).toBe(30);
    // And only one frame was ever scheduled, not one per push.
    expect(clock.pending).toBe(1);

    clock.tick();

    expect(flushes).toHaveLength(1);
    expect(flushes[0]).toHaveLength(30);
    const last = flushes[0]?.at(-1);
    expect(last?.kind === 'telemetry' && last.robot.position.x).toBe(29);
  });

  it('schedules a new frame after flushing', () => {
    const clock = new ManualClock();
    const flushes: (readonly LiveEvent[])[] = [];
    const buffer = new FrameBuffer((batch) => flushes.push(batch), clock);

    buffer.push({ kind: 'lag', dropped: 1 });
    clock.tick();
    buffer.push({ kind: 'lag', dropped: 2 });
    clock.tick();

    expect(flushes.map((b) => b.length)).toEqual([1, 1]);
  });

  it('drops what is queued and cancels its frame on dispose', () => {
    const clock = new ManualClock();
    const flushes: (readonly LiveEvent[])[] = [];
    const buffer = new FrameBuffer((batch) => flushes.push(batch), clock);

    buffer.push({ kind: 'lag', dropped: 1 });
    buffer.dispose();

    expect(clock.pending).toBe(0);
    clock.tick();
    expect(flushes).toHaveLength(0);
  });
});

describe('defaultLiveUrl', () => {
  it('follows the page scheme so a TLS page never opens a plaintext socket', () => {
    expect(defaultLiveUrl({ protocol: 'http:', host: 'localhost:5173' })).toBe(
      'ws://localhost:5173/ws/live',
    );
    expect(defaultLiveUrl({ protocol: 'https:', host: 'example.test' })).toBe(
      'wss://example.test/ws/live',
    );
  });
});

describe('LiveSocket', () => {
  let clock: ManualClock;
  let statuses: ConnectionStatus[];
  let batches: (readonly LiveEvent[])[];
  let socket: LiveSocket | null;

  beforeEach(() => {
    FakeWebSocket.reset();
    clock = new ManualClock();
    statuses = [];
    batches = [];
    socket = null;
  });

  afterEach(() => {
    socket?.close();
  });

  function connect(): LiveSocket {
    socket = new LiveSocket(
      {
        onBatch: (batch) => batches.push(batch),
        onStatus: (status) => statuses.push(status),
      },
      {
        url: 'ws://test.invalid/ws/live',
        clock,
        webSocketImpl: FakeWebSocket,
        minReconnectionDelayMs: 1,
        connectionTimeoutMs: 1000,
        now: FIXED_NOW,
      },
    );
    return socket;
  }

  it('reports connecting immediately and open once the handshake completes', async () => {
    connect();
    expect(statuses).toEqual(['connecting']);

    await vi.waitFor(() => {
      expect(statuses).toContain('open');
    });
  });

  it('subscribes to the topics it can actually render, on every open', async () => {
    connect();
    await vi.waitFor(() => {
      expect(FakeWebSocket.latest().sent).toHaveLength(1);
    });

    // Asserted against DEFAULT_TOPICS rather than a literal list: the point of the test
    // is that the socket subscribes to exactly what it can render, and hardcoding the
    // list here would mean editing this test every time a renderer is added.
    expect(FakeWebSocket.latest().sentJson()[0]).toEqual({
      type: 'subscribe',
      topics: [...DEFAULT_TOPICS],
    });
    // The dashboard must never subscribe to a frame type it has no handler for — those
    // would arrive and be counted as malformed.
    expect(DEFAULT_TOPICS).toContain('telemetry');
    expect(DEFAULT_TOPICS).toContain('detection');

    // A dropped connection loses the server-side channel, so the subscription must be
    // re-sent rather than assumed to have survived.
    FakeWebSocket.latest().serverClose();
    await vi.waitFor(() => {
      expect(FakeWebSocket.instances).toHaveLength(2);
      expect(FakeWebSocket.latest().sent).toHaveLength(1);
    });
    // Asserted against DEFAULT_TOPICS rather than a literal list: the point of the test
    // is that the socket subscribes to exactly what it can render, and hardcoding the
    // list here would mean editing this test every time a renderer is added.
    expect(FakeWebSocket.latest().sentJson()[0]).toEqual({
      type: 'subscribe',
      topics: [...DEFAULT_TOPICS],
    });
    // The dashboard must never subscribe to a frame type it has no handler for — those
    // would arrive and be counted as malformed.
    expect(DEFAULT_TOPICS).toContain('telemetry');
    expect(DEFAULT_TOPICS).toContain('detection');
  });

  it('reports reconnecting — not error — when the server drops the connection', async () => {
    connect();
    await vi.waitFor(() => {
      expect(statuses).toContain('open');
    });

    FakeWebSocket.latest().serverClose();
    await vi.waitFor(() => {
      expect(statuses).toContain('reconnecting');
    });
    expect(statuses).not.toContain('error');

    await vi.waitFor(() => {
      expect(statuses.filter((s) => s === 'open')).toHaveLength(2);
    });
  });

  it('reports an error when the connection fails', async () => {
    connect();
    await vi.waitFor(() => {
      expect(statuses).toContain('open');
    });

    FakeWebSocket.latest().failWith('ECONNREFUSED');
    await vi.waitFor(() => {
      expect(statuses).toContain('error');
    });
  });

  it('delivers a burst of frames as one batch per frame tick', async () => {
    connect();
    await vi.waitFor(() => {
      expect(statuses).toContain('open');
    });

    const ws = FakeWebSocket.latest();
    for (let i = 0; i < 12; i += 1) ws.emit(telemetryFrame({ local_x: i }));
    ws.emit('{"type":"lag","dropped":4}');

    expect(batches).toHaveLength(0);
    clock.tick();

    expect(batches).toHaveLength(1);
    expect(batches[0]).toHaveLength(13);
    expect(batches[0]?.at(-1)).toEqual({ kind: 'lag', dropped: 4 });
  });

  it('buffers a send issued while the socket is down and flushes it on open', async () => {
    FakeWebSocket.autoOpen = false;
    const live = connect();
    live.send({ type: 'subscribe', topics: ['telemetry'] });

    await vi.waitFor(() => {
      expect(FakeWebSocket.instances).toHaveLength(1);
    });
    // Still handshaking: nothing has gone out on the wire yet.
    expect(FakeWebSocket.latest().sent).toHaveLength(0);

    FakeWebSocket.latest().acceptOpen();
    // The buffered frame plus the subscribe the open handler sends.
    expect(FakeWebSocket.latest().sent).toHaveLength(2);
    expect(FakeWebSocket.latest().sentJson()[0]).toEqual({
      type: 'subscribe',
      topics: ['telemetry'],
    });
  });

  it('stops reconnecting once closed', async () => {
    const live = connect();
    await vi.waitFor(() => {
      expect(statuses).toContain('open');
    });

    live.close();
    socket = null;
    expect(statuses.at(-1)).toBe('closed');

    const instanceCount = FakeWebSocket.instances.length;
    await new Promise((resolve) => setTimeout(resolve, 20));
    expect(FakeWebSocket.instances).toHaveLength(instanceCount);
  });
});
