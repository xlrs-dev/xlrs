import { useCallback, useRef, useState } from 'react';
import {
  buildFrame,
  CMD,
  FrameParser,
  parseResponse,
  parseDeviceInfo,
  parseConfigPayload,
  configToPayload,
  parseStateFrame,
  parseLinkStatus,
  type ParsedFrame,
  type ProtoResponse,
  type DeviceInfo,
  type StateFrame,
  type LinkStatus,
} from '../lib/protocol';
import type { RcConfig } from '../types/rc';

/** Web Serial API – not in default TypeScript libs; declare for type safety. */
declare global {
  interface SerialPort {
    open(options: { baudRate: number }): Promise<void>;
    close(): Promise<void>;
    getInfo(): { usbVendorId?: number; usbProductId?: number };
    readonly readable: ReadableStream<Uint8Array> | null;
    readonly writable: WritableStream<Uint8Array> | null;
  }
  interface Navigator {
    serial?: {
      requestPort: () => Promise<SerialPort>;
      getPorts: () => Promise<SerialPort[]>;
    };
  }
}

/** Label for a SerialPort for display (VID:PID or "Serial port") */
export function getPortLabel(port: SerialPort): string {
  const info = port.getInfo();
  if (info.usbVendorId != null && info.usbProductId != null) {
    const vid = info.usbVendorId.toString(16).padStart(4, '0');
    const pid = info.usbProductId.toString(16).padStart(4, '0');
    return `USB ${vid}:${pid}`;
  }
  return 'Serial port';
}

export interface UseSerialResult {
  connected: boolean;
  portName: string | null;
  deviceInfo: DeviceInfo | null;
  error: string | null;
  /** Ports the site has permission to use (from getPorts()). Refresh on load and after connect. */
  availablePorts: SerialPort[];
  refreshPorts: () => Promise<void>;
  /** Connect to a port from the available list (e.g. from dropdown). */
  connectToPort: (port: SerialPort) => Promise<void>;
  /** Open browser picker to grant access to a new device, then connect. */
  connectNew: () => Promise<void>;
  connect: () => Promise<void>;
  disconnect: () => void;
  request: (cmd: number, payload?: Uint8Array | null) => Promise<ProtoResponse>;
  getDeviceInfo: () => Promise<DeviceInfo | null>;
  getConfig: () => Promise<RcConfig | null>;
  setConfigDraft: (cfg: RcConfig) => Promise<boolean>;
  applyConfig: () => Promise<boolean>;
  saveConfig: () => Promise<boolean>;
  calStart: () => Promise<boolean>;
  calSample: () => Promise<boolean>;
  calFinish: () => Promise<boolean>;
  setRxBindingPhrase: (phrase: string) => Promise<boolean>;
  setTxBindingPhrase: (phrase: string) => Promise<boolean>;
  getLinkStatus: () => Promise<LinkStatus | null>;
  enterPairingMode: () => Promise<boolean>;
  streamStart: (onState: (s: StateFrame) => void) => void;
  streamStop: () => void;
}

let seq = 0;
function nextSeq(): number {
  return (++seq) & 0xff;
}

export function useSerial(): UseSerialResult {
  const [connected, setConnected] = useState(false);
  const [portName, setPortName] = useState<string | null>(null);
  const [deviceInfo, setDeviceInfo] = useState<DeviceInfo | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [availablePorts, setAvailablePorts] = useState<SerialPort[]>([]);

  const portRef = useRef<SerialPort | null>(null);
  const readerRef = useRef<ReadableStreamDefaultReader<Uint8Array> | null>(null);
  const parserRef = useRef(new FrameParser());
  const pendingRef = useRef<Map<number, { resolve: (r: ProtoResponse) => void }>>(new Map());
  const stateCallbackRef = useRef<((s: StateFrame) => void) | null>(null);
  const readLoopRef = useRef<boolean>(false);

  const disconnect = useCallback(() => {
    readLoopRef.current = false;
    stateCallbackRef.current = null;
    if (readerRef.current) {
      readerRef.current.cancel().catch(() => {});
      readerRef.current = null;
    }
    if (portRef.current) {
      portRef.current.close().catch(() => {});
      portRef.current = null;
    }
    setConnected(false);
    setPortName(null);
    setDeviceInfo(null);
    pendingRef.current.forEach((p) => p.resolve({ status: 255, schema: 0, data: null }));
    pendingRef.current.clear();
  }, []);

  const send = useCallback(
    async (frame: Uint8Array) => {
      const port = portRef.current;
      if (!port?.writable) throw new Error('Not connected');
      const writer = port.writable.getWriter();
      try {
        await writer.write(frame);
      } finally {
        writer.releaseLock();
      }
    },
    []
  );

  const request = useCallback(
    async (cmd: number, payload?: Uint8Array | null): Promise<ProtoResponse> => {
      const s = nextSeq();
      const frame = buildFrame(cmd, s, payload ?? null);
      await send(frame);
      return new Promise<ProtoResponse>((resolve) => {
        pendingRef.current.set((cmd << 8) | s, { resolve });
        setTimeout(() => {
          if (pendingRef.current.has((cmd << 8) | s)) {
            pendingRef.current.delete((cmd << 8) | s);
            resolve({ status: 255, schema: 0, data: null });
          }
        }, 3000);
      });
    },
    [send]
  );

  const processFrame = useCallback((f: ParsedFrame) => {
    if ((f.cmd & 0x80) !== 0) {
      const reqCmd = f.cmd & 0x7f;
      const key = (reqCmd << 8) | f.seq;
      const p = pendingRef.current.get(key);
      if (p) {
        pendingRef.current.delete(key);
        const resp = f.payload ? parseResponse(f.payload) : { status: 255, schema: 0, data: null };
        p.resolve(resp);
      }
    } else if (f.cmd === CMD.STATE_FRAME && f.payload) {
      const state = parseStateFrame(f.payload);
      if (state && stateCallbackRef.current) stateCallbackRef.current(state);
    }
  }, []);

  const readLoop = useCallback(async (port: SerialPort) => {
    if (!port.readable) return;
    const reader = port.readable.getReader();
    readerRef.current = reader;
    readLoopRef.current = true;
    const parser = parserRef.current;
    try {
      while (readLoopRef.current) {
        const { value, done } = await reader.read();
        if (done) break;
        for (let i = 0; i < value.length; i++) {
          const frame = parser.push(value[i]!);
          if (frame) processFrame(frame);
        }
      }
    } catch (e) {
      if (readLoopRef.current) setError(String(e));
    } finally {
      readerRef.current = null;
      parser.reset();
      disconnect();
    }
  }, [processFrame, disconnect]);

  const openPort = useCallback(
    async (port: SerialPort) => {
      await port.open({ baudRate: 115200 });
      portRef.current = port;
      setPortName(getPortLabel(port));
      setConnected(true);
      setError(null);
      readLoop(port);
      try {
        const info = await request(CMD.GET_DEVICE_INFO);
        if (info.data) {
          const dev = parseDeviceInfo(info.data, info.status, info.schema);
          if (dev) setDeviceInfo(dev);
        }
      } catch {
        setDeviceInfo(null);
      }
    },
    [request, readLoop]
  );

  const refreshPorts = useCallback(async () => {
    if (!navigator.serial) return;
    try {
      const ports = await navigator.serial.getPorts();
      setAvailablePorts(ports);
    } catch {
      setAvailablePorts([]);
    }
  }, []);

  const connectToPort = useCallback(
    async (port: SerialPort) => {
      setError(null);
      if (!navigator.serial) {
        setError('Web Serial not supported. Use Chrome or Edge.');
        return;
      }
      try {
        await openPort(port);
        await refreshPorts();
      } catch (e) {
        setError(e instanceof Error ? e.message : String(e));
        disconnect();
      }
    },
    [openPort, refreshPorts, disconnect]
  );

  const connectNew = useCallback(async () => {
    setError(null);
    if (!navigator.serial) {
      setError('Web Serial not supported. Use Chrome or Edge.');
      return;
    }
    try {
      const port = await navigator.serial.requestPort();
      await openPort(port);
      await refreshPorts();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
      disconnect();
    }
  }, [openPort, refreshPorts, disconnect]);

  const connect = useCallback(async () => {
    await connectNew();
  }, [connectNew]);

  const getDeviceInfo = useCallback(async () => {
    const resp = await request(CMD.GET_DEVICE_INFO);
    if (resp.status !== 0 || !resp.data) return null;
    return parseDeviceInfo(resp.data, resp.status, resp.schema);
  }, [request]);

  const getConfig = useCallback(async (): Promise<RcConfig | null> => {
    const resp = await request(CMD.GET_CONFIG);
    if (resp.status !== 0 || !resp.data) return null;
    return parseConfigPayload(resp.data);
  }, [request]);

  const setConfigDraft = useCallback(
    async (cfg: RcConfig): Promise<boolean> => {
      const resp = await request(CMD.SET_CONFIG_DRAFT, configToPayload(cfg));
      return resp.status === 0;
    },
    [request]
  );

  const applyConfig = useCallback(async (): Promise<boolean> => {
    const resp = await request(CMD.APPLY_CONFIG);
    return resp.status === 0;
  }, [request]);

  const saveConfig = useCallback(async (): Promise<boolean> => {
    const resp = await request(CMD.SAVE_CONFIG);
    return resp.status === 0;
  }, [request]);

  const calStart = useCallback(async (): Promise<boolean> => {
    const resp = await request(CMD.START_CALIBRATION);
    return resp.status === 0;
  }, [request]);

  const calSample = useCallback(async (): Promise<boolean> => {
    const resp = await request(CMD.CALIBRATION_SAMPLE);
    return resp.status === 0;
  }, [request]);

  const calFinish = useCallback(async (): Promise<boolean> => {
    const resp = await request(CMD.FINISH_CALIBRATION);
    return resp.status === 0;
  }, [request]);

  const setRxBindingPhrase = useCallback(
    async (phrase: string): Promise<boolean> => {
      const payload = new TextEncoder().encode(phrase.trim());
      if (!payload.length || payload.length > 32) return false;
      const resp = await request(CMD.SET_BINDING_PHRASE_RX, payload);
      return resp.status === 0;
    },
    [request]
  );

  const setTxBindingPhrase = useCallback(
    async (phrase: string): Promise<boolean> => {
      const payload = new TextEncoder().encode(phrase.trim());
      if (!payload.length || payload.length > 32) return false;
      const resp = await request(CMD.SET_BINDING_PHRASE_TX, payload);
      return resp.status === 0;
    },
    [request]
  );

  const getLinkStatus = useCallback(async (): Promise<LinkStatus | null> => {
    const resp = await request(CMD.GET_LINK_STATUS);
    if (resp.status !== 0) return null;
    return parseLinkStatus(resp.data);
  }, [request]);

  const enterPairingMode = useCallback(async (): Promise<boolean> => {
    const resp = await request(CMD.ENTER_PAIRING_MODE);
    return resp.status === 0;
  }, [request]);

  const streamStart = useCallback(
    (onState: (s: StateFrame) => void) => {
      stateCallbackRef.current = onState;
      request(CMD.STREAM_STATE_START)
        .then((resp) => {
          if (resp.status !== 0) {
            setError(`Live stream start failed (status ${resp.status}). Check RC firmware.`);
          }
        })
        .catch(() => {
          setError('No response to live stream start. Is rc-crsf / rc-rp2350 firmware running?');
        });
    },
    [request]
  );

  const streamStop = useCallback(() => {
    stateCallbackRef.current = null;
    request(CMD.STREAM_STATE_STOP).catch(() => {});
  }, [request]);

  return {
    connected,
    portName,
    deviceInfo,
    error,
    availablePorts,
    refreshPorts,
    connectToPort,
    connectNew,
    connect,
    disconnect,
    request,
    getDeviceInfo,
    getConfig,
    setConfigDraft,
    applyConfig,
    saveConfig,
    calStart,
    calSample,
    calFinish,
    setRxBindingPhrase,
    setTxBindingPhrase,
    getLinkStatus,
    enterPairingMode,
    streamStart,
    streamStop,
  };
}
