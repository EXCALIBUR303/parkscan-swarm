/**
 * A WebSocket stand-in for jsdom tests.
 *
 * jsdom does ship a WebSocket, but it opens a real TCP connection, so using it here
 * would make the suite depend on a running backend and on network timing. This class
 * implements just the surface `partysocket` touches — `readyState`, `binaryType`,
 * `send`, `close`, and the four events — and adds explicit test controls so a test says
 * *when* the socket opens, what it delivers and how it fails.
 *
 * Install it either by passing it as `webSocketImpl`, or with
 * `vi.stubGlobal('WebSocket', FakeWebSocket)` (partysocket resolves the global lazily,
 * at connect time).
 */

export class FakeWebSocket extends EventTarget {
  static readonly CONNECTING = 0;
  static readonly OPEN = 1;
  static readonly CLOSING = 2;
  static readonly CLOSED = 3;

  /** Every socket constructed since the last `reset()`, oldest first. */
  static instances: FakeWebSocket[] = [];

  /** When true, a new socket opens itself on the next microtask. */
  static autoOpen = true;

  static reset(): void {
    FakeWebSocket.instances = [];
    FakeWebSocket.autoOpen = true;
  }

  /** The socket partysocket is currently using, i.e. the newest one. */
  static latest(): FakeWebSocket {
    const socket = FakeWebSocket.instances.at(-1);
    if (socket === undefined) throw new Error('no FakeWebSocket has been constructed yet');
    return socket;
  }

  readonly CONNECTING = FakeWebSocket.CONNECTING;
  readonly OPEN = FakeWebSocket.OPEN;
  readonly CLOSING = FakeWebSocket.CLOSING;
  readonly CLOSED = FakeWebSocket.CLOSED;

  readyState: number = FakeWebSocket.CONNECTING;
  binaryType = 'blob';

  /** Everything the client sent, in order. */
  readonly sent: string[] = [];

  constructor(readonly url: string) {
    super();
    FakeWebSocket.instances.push(this);
    if (FakeWebSocket.autoOpen) {
      queueMicrotask(() => {
        this.acceptOpen();
      });
    }
  }

  send(data: string): void {
    this.sent.push(data);
  }

  close(code = 1000, reason = ''): void {
    if (this.readyState === FakeWebSocket.CLOSED) return;
    this.readyState = FakeWebSocket.CLOSED;
    this.dispatchEvent(new CloseEvent('close', { code, reason, wasClean: true }));
  }

  /* ------------------------------------------------------------ test controls -- */

  /** Complete the handshake. */
  acceptOpen(): void {
    if (this.readyState !== FakeWebSocket.CONNECTING) return;
    this.readyState = FakeWebSocket.OPEN;
    this.dispatchEvent(new Event('open'));
  }

  /** Deliver one server frame. */
  emit(text: string): void {
    this.dispatchEvent(new MessageEvent('message', { data: text }));
  }

  /** Fail the connection the way a refused or reset socket does. */
  failWith(message: string): void {
    this.dispatchEvent(new ErrorEvent('error', { message, error: new Error(message) }));
  }

  /** Close from the server side, uncleanly — the case reconnection exists for. */
  serverClose(code = 1006, reason = 'going away'): void {
    if (this.readyState === FakeWebSocket.CLOSED) return;
    this.readyState = FakeWebSocket.CLOSED;
    this.dispatchEvent(new CloseEvent('close', { code, reason, wasClean: false }));
  }

  /** The frames sent so far, decoded. */
  sentJson(): unknown[] {
    return this.sent.map((text) => JSON.parse(text) as unknown);
  }
}
