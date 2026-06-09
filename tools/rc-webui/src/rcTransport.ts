export type RcMessageKind = "ok" | "err" | "state" | "config" | "binding" | "unknown";

export interface RcMessage {
  kind: RcMessageKind;
  line: string;
  fields: Record<string, string>;
  args: string[];
  receivedAt: Date;
}

export type TransportStatus =
  | "idle"
  | "connecting"
  | "connected"
  | "reconnecting"
  | "disconnecting"
  | "disconnected"
  | "unsupported"
  | "error";

export interface TransportEventMap {
  status: { status: TransportStatus; detail?: string };
  write: { line: string };
  line: { line: string };
  message: { message: RcMessage };
  error: { error: unknown };
}

type Listener<K extends keyof TransportEventMap> = (event: TransportEventMap[K]) => void;
type AnyListener = (event: TransportEventMap[keyof TransportEventMap]) => void;
type PendingRequest = {
  resolve: (message: RcMessage) => void;
  reject: (error: Error) => void;
  timer: number;
};

const KNOWN_KINDS = new Set<RcMessageKind>(["ok", "err", "state", "config", "binding"]);
const RECONNECT_DELAYS_MS = [250, 500, 1000, 2000, 5000];
const DEFAULT_TIMEOUT_MS = 2500;
const OPEN_HELLO_LINE = "rc.v1 hello client=rc-webui";

export function encodeRcValue(value: string): string {
  return encodeURIComponent(value);
}

export function decodeRcValue(value: string): string {
  try {
    return decodeURIComponent(value);
  } catch {
    return value;
  }
}

export function parseRcLine(line: string): RcMessage {
  const trimmed = line.trim();
  const tokens = trimmed.split(/\s+/).filter(Boolean);
  const first = tokens[0] ?? "";
  const hasRcPrefix = first === "rc.v1";
  const kindToken = hasRcPrefix ? tokens[1] ?? "" : first;
  const fieldTokens = hasRcPrefix ? tokens.slice(2) : tokens.slice(1);
  const kind = KNOWN_KINDS.has(kindToken as RcMessageKind) ? (kindToken as RcMessageKind) : "unknown";
  const fields: Record<string, string> = {};
  const args: string[] = [];

  for (const token of fieldTokens) {
    const equalsAt = token.indexOf("=");
    if (equalsAt <= 0) {
      args.push(decodeRcValue(token));
      continue;
    }

    const key = token.slice(0, equalsAt);
    const value = token.slice(equalsAt + 1);
    fields[key] = decodeRcValue(value);
  }

  return { kind, line, fields, args, receivedAt: new Date() };
}

export class RcSerialTransport {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
  private reconnectTimer = 0;
  private reconnectAttempt = 0;
  private seq = 1;
  private readonly pending = new Map<string, PendingRequest>();
  private shouldReconnect = false;
  private manuallyClosing = false;
  private readonly listeners = new Map<keyof TransportEventMap, Set<AnyListener>>();

  constructor(private readonly baudRate = 115200) {
    navigator.serial?.addEventListener("disconnect", () => {
      if (this.port && !this.manuallyClosing) {
        this.emit("status", { status: "disconnected", detail: "Serial device disconnected." });
        void this.cleanupPort();
        this.scheduleReconnect();
      }
    });
  }

  get activePort(): SerialPort | null {
    return this.port;
  }

  on<K extends keyof TransportEventMap>(type: K, listener: Listener<K>): () => void {
    const listeners = this.listeners.get(type) ?? new Set<AnyListener>();
    listeners.add(listener as AnyListener);
    this.listeners.set(type, listeners);
    return () => listeners.delete(listener as AnyListener);
  }

  async connect({ useRememberedPort = true }: { useRememberedPort?: boolean } = {}): Promise<void> {
    if (!navigator.serial) {
      this.emit("status", {
        status: "unsupported",
        detail: "Web Serial is only available in Chromium-based browsers on secure origins.",
      });
      return;
    }

    this.clearReconnect();
    this.shouldReconnect = true;
    this.manuallyClosing = false;
    this.emit("status", { status: "connecting" });

    try {
      const rememberedPorts = useRememberedPort ? await navigator.serial.getPorts() : [];
      this.port = rememberedPorts[0] ?? (await navigator.serial.requestPort());
      await this.openCurrentPort();
    } catch (error) {
      this.emit("error", { error });
      this.emit("status", { status: "error", detail: errorToString(error) });
      throw error;
    }
  }

  async disconnect(): Promise<void> {
    this.manuallyClosing = true;
    this.shouldReconnect = false;
    this.clearReconnect();
    this.rejectPending(new Error("Serial port disconnected."));
    this.emit("status", { status: "disconnecting" });
    await this.cleanupPort();
    this.emit("status", { status: "idle" });
  }

  async sendLine(line: string): Promise<void> {
    if (!this.writer) throw new Error("Serial port is not connected.");
    const normalized = line.endsWith("\n") ? line : `${line}\n`;
    this.emit("write", { line: normalized.trimEnd() });
    await this.writer.write(new TextEncoder().encode(normalized));
  }

  async request(
    command: string,
    fields: Record<string, string | number | boolean | null | undefined> = {},
    timeoutMs = DEFAULT_TIMEOUT_MS,
  ): Promise<RcMessage> {
    if (!this.writer) throw new Error("Serial port is not connected.");

    const seq = String(this.seq++);
    const encodedFields = Object.entries({ seq, ...fields })
      .filter(([, value]) => value !== undefined && value !== null)
      .map(([key, value]) => `${key}=${encodeRcValue(String(value))}`)
      .join(" ");
    const line = `rc.v1 ${command} ${encodedFields}`;

    const response = new Promise<RcMessage>((resolve, reject) => {
      const timer = window.setTimeout(() => {
        this.pending.delete(seq);
        reject(new Error(`${command} timed out`));
      }, timeoutMs);
      this.pending.set(seq, { resolve, reject, timer });
    });

    await this.sendLine(line);
    return response;
  }

  hello(): Promise<RcMessage> {
    return this.request("hello");
  }

  txHello(): Promise<RcMessage> {
    return this.request("tx_hello");
  }

  getConfig(): Promise<RcMessage> {
    return this.request("get_config");
  }

  setConfig(field: string, value: string | number | boolean): Promise<RcMessage> {
    return this.request("set_config", { field, value });
  }

  apply(): Promise<RcMessage> {
    return this.request("apply");
  }

  save(): Promise<RcMessage> {
    return this.request("save");
  }

  resetDefaults(target = "rc_config"): Promise<RcMessage> {
    return this.request("reset_defaults", { target });
  }

  getState(): Promise<RcMessage> {
    return this.request("state");
  }

  streamState(intervalMs = 100, enabled = true): Promise<RcMessage> {
    return this.request("stream_state", { interval_ms: intervalMs, enabled: enabled ? 1 : 0 });
  }

  calibrationStart(): Promise<RcMessage> {
    return this.request("cal_start");
  }

  calibrationSample(): Promise<RcMessage> {
    return this.request("cal_sample");
  }

  calibrationFinish(save: boolean): Promise<RcMessage> {
    return this.request("cal_finish", { save: save ? 1 : 0 }, 5000);
  }

  bindingGet(target: "tx" | "rx"): Promise<RcMessage> {
    return this.request("binding_get", { target });
  }

  bindingSet(target: "tx" | "rx", phrase: string): Promise<RcMessage> {
    return this.request("binding_set", { target, phrase }, 5000);
  }

  bindingVerify(target: "tx" | "rx", phrase: string): Promise<RcMessage> {
    return this.request("binding_verify", { target, phrase }, 5000);
  }

  bindingClear(target: "tx" | "rx"): Promise<RcMessage> {
    return this.request("binding_clear", { target }, 5000);
  }

  private async openCurrentPort(): Promise<void> {
    if (!this.port) throw new Error("No serial port selected.");

    await this.port.open({ baudRate: this.baudRate, bufferSize: 4096 });
    if (!this.port.readable || !this.port.writable) {
      throw new Error("Serial port did not expose readable and writable streams.");
    }

    this.reader = this.port.readable.getReader();
    this.writer = this.port.writable.getWriter();
    this.reconnectAttempt = 0;
    this.emit("status", { status: "connected", detail: describePort(this.port) });
    void this.readLoop();
    await this.sendLine(OPEN_HELLO_LINE);
  }

  private async readLoop(): Promise<void> {
    const decoder = new TextDecoder();
    let pending = "";

    try {
      while (this.reader) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (!value) continue;

        pending += decoder.decode(value, { stream: true });
        let newlineAt = pending.search(/\r?\n/);
        while (newlineAt >= 0) {
          const line = pending.slice(0, newlineAt).replace(/\r$/, "");
          pending = pending.slice(newlineAt + (pending[newlineAt] === "\r" ? 2 : 1));
          this.handleLine(line);
          newlineAt = pending.search(/\r?\n/);
        }
      }
    } catch (error) {
      if (!this.manuallyClosing) {
        this.rejectPending(new Error(`Serial port disconnected: ${errorToString(error)}`));
        this.emit("error", { error });
        this.emit("status", { status: "disconnected", detail: errorToString(error) });
      }
    } finally {
      if (!this.manuallyClosing) {
        this.rejectPending(new Error("Serial port disconnected."));
      }
      if (pending.trim()) this.handleLine(pending.trim());
      await this.cleanupPort();
      if (this.shouldReconnect && !this.manuallyClosing) this.scheduleReconnect();
    }
  }

  private handleLine(line: string): void {
    this.emit("line", { line });
    const message = parseRcLine(line);
    const seq = message.fields.seq;
    if (seq && this.pending.has(seq)) {
      const pending = this.pending.get(seq)!;
      window.clearTimeout(pending.timer);
      this.pending.delete(seq);
      if (message.kind === "err") {
        const code = message.fields.code ?? "error";
        const detail = message.fields.message ? `: ${message.fields.message}` : "";
        pending.reject(new Error(`${code}${detail}`));
      } else {
        pending.resolve(message);
      }
    }
    this.emit("message", { message });
  }

  private scheduleReconnect(): void {
    if (!this.port || this.reconnectTimer !== 0) return;

    const delay = RECONNECT_DELAYS_MS[Math.min(this.reconnectAttempt, RECONNECT_DELAYS_MS.length - 1)];
    this.reconnectAttempt += 1;
    this.emit("status", { status: "reconnecting", detail: `Retrying in ${delay} ms.` });
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = 0;
      void this.openCurrentPort().catch((error) => {
        this.emit("error", { error });
        this.scheduleReconnect();
      });
    }, delay);
  }

  private clearReconnect(): void {
    if (this.reconnectTimer !== 0) {
      window.clearTimeout(this.reconnectTimer);
      this.reconnectTimer = 0;
    }
  }

  private async cleanupPort(): Promise<void> {
    const reader = this.reader;
    const writer = this.writer;
    this.reader = null;
    this.writer = null;

    try {
      await reader?.cancel();
    } catch {
      // The stream may already be closed after a physical disconnect.
    }
    try {
      reader?.releaseLock();
    } catch {
      // Ignore already-released locks.
    }
    try {
      writer?.releaseLock();
    } catch {
      // Ignore already-released locks.
    }
    try {
      await this.port?.close();
    } catch {
      // Closing a removed USB device commonly rejects; reconnect will reopen later.
    }
  }

  private rejectPending(error: Error): void {
    for (const pending of this.pending.values()) {
      window.clearTimeout(pending.timer);
      pending.reject(error);
    }
    this.pending.clear();
  }

  private emit<K extends keyof TransportEventMap>(type: K, event: TransportEventMap[K]): void {
    this.listeners.get(type)?.forEach((listener) => listener(event));
  }
}

function describePort(port: SerialPort): string {
  const info = port.getInfo();
  const vendor = info.usbVendorId?.toString(16).padStart(4, "0");
  const product = info.usbProductId?.toString(16).padStart(4, "0");
  return vendor && product ? `USB ${vendor}:${product}` : "Serial port connected.";
}

function errorToString(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}
