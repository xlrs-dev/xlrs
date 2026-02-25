/**
 * RC Config protocol (binary) - matches firmware RCConfigProtocol + RCConfig.
 * Frame: SYNC 0xA5, LEN, CMD, SEQ, PAYLOAD..., CRC8 (LEN = 2 + payload length, CRC over CMD+SEQ+PAYLOAD).
 */

import type { RcConfig } from '../types/rc';
import { NUM_AXES, NUM_CHANNELS } from '../types/rc';

const SYNC = 0xa5;
const MAX_PAYLOAD = 128;

export const CMD = {
  GET_DEVICE_INFO: 0x01,
  GET_CONFIG: 0x02,
  SET_CONFIG_DRAFT: 0x03,
  APPLY_CONFIG: 0x04,
  SAVE_CONFIG: 0x05,
  START_CALIBRATION: 0x10,
  CALIBRATION_SAMPLE: 0x11,
  FINISH_CALIBRATION: 0x12,
  STREAM_STATE_START: 0x20,
  STREAM_STATE_STOP: 0x21,
  STATE_FRAME: 0x30,
  SET_BINDING_PHRASE_RX: 0x40,
  SET_BINDING_PHRASE_TX: 0x41,
  GET_LINK_STATUS: 0x42,
  ENTER_PAIRING_MODE: 0x43,
} as const;

export const STATUS = {
  OK: 0,
  ERR_INVALID_CMD: 1,
  ERR_PAYLOAD: 2,
  ERR_CRC: 3,
  ERR_BUSY: 4,
  ERR_SAVE: 5,
  ERR_FORWARD: 6,
} as const;

const CRC8_POLY = 0xd5;
const crc8Table = new Uint8Array(256);
for (let i = 0; i < 256; i++) {
  let c = i;
  for (let s = 0; s < 8; s++) c = ((c << 1) ^ (c & 0x80 ? CRC8_POLY : 0)) & 0xff;
  crc8Table[i] = c;
}

function crc8(data: Uint8Array): number {
  let crc = 0;
  for (let i = 0; i < data.length; i++) crc = crc8Table[crc ^ data[i]];
  return crc;
}

/** Build frame bytes: SYNC, LEN, CMD, SEQ, PAYLOAD..., CRC */
export function buildFrame(cmd: number, seq: number, payload: Uint8Array | null): Uint8Array {
  const pl = payload ? payload.length : 0;
  const len = 2 + pl;
  if (len > MAX_PAYLOAD) throw new Error('payload too long');
  const buf = new Uint8Array(1 + 1 + len + 1);
  buf[0] = SYNC;
  buf[1] = len;
  buf[2] = cmd;
  buf[3] = seq;
  if (pl && payload) buf.set(payload, 4);
  // CRC byte is placed immediately after CMD+SEQ+PAYLOAD body.
  buf[2 + len] = crc8(buf.subarray(2, 2 + len));
  return buf;
}

export interface ParsedFrame {
  cmd: number;
  seq: number;
  payload: Uint8Array | null;
}

export class FrameParser {
  private state: 'sync' | 'len' | 'body' = 'sync';
  private len = 0;
  private buf: number[] = [];

  reset(): void {
    this.state = 'sync';
    this.len = 0;
    this.buf = [];
  }

  push(byte: number): ParsedFrame | null {
    switch (this.state) {
      case 'sync':
        if (byte === SYNC) this.state = 'len';
        return null;
      case 'len':
        if (byte > MAX_PAYLOAD) {
          this.state = 'sync';
          return null;
        }
        this.len = byte;
        this.buf = [];
        this.state = 'body';
        return null;
      case 'body': {
        this.buf.push(byte);
        if (this.buf.length < this.len + 1) return null; // len bytes body + 1 crc
        const body = new Uint8Array(this.buf);
        const payload = body.byteLength > 2 ? body.subarray(2, this.len) : new Uint8Array(0);
        const recvCrc = body[this.len];
        const calcCrc = crc8(body.subarray(0, this.len));
        this.state = 'sync';
        if (recvCrc !== calcCrc) return null;
        return {
          cmd: body[0],
          seq: body[1],
          payload: payload.length > 0 ? payload : null,
        };
      }
      default:
        this.state = 'sync';
        return null;
    }
  }
}

const CONFIG_PAYLOAD_SIZE =
  4 + 4 + 4 * 2 * 3 + 4 * 4 * 3 + 8 * 2 * 2 + 1;

function configToBytes(cfg: RcConfig): Uint8Array {
  const buf = new ArrayBuffer(CONFIG_PAYLOAD_SIZE);
  const dv = new DataView(buf);
  let off = 0;
  for (let i = 0; i < NUM_AXES; i++) dv.setUint8(off++, cfg.channel_function[i] ?? i);
  for (let i = 0; i < NUM_AXES; i++) dv.setUint8(off++, cfg.invert[i] ? 1 : 0);
  for (let i = 0; i < NUM_AXES; i++) {
    dv.setUint16(off, cfg.calib_min[i] ?? 2917, true);
    off += 2;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    dv.setUint16(off, cfg.calib_max[i] ?? 23420, true);
    off += 2;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    dv.setUint16(off, cfg.calib_center[i] ?? 13199, true);
    off += 2;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    dv.setFloat32(off, cfg.deadzone[i] ?? 0.05, true);
    off += 4;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    dv.setFloat32(off, cfg.rate[i] ?? 0.7, true);
    off += 4;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    dv.setFloat32(off, cfg.expo[i] ?? 0.3, true);
    off += 4;
  }
  for (let i = 0; i < NUM_CHANNELS; i++) {
    dv.setUint16(off, cfg.cutoff_min[i] ?? 1000, true);
    off += 2;
  }
  for (let i = 0; i < NUM_CHANNELS; i++) {
    dv.setUint16(off, cfg.cutoff_max[i] ?? 2000, true);
    off += 2;
  }
  dv.setUint8(off, cfg.high_pass_filter ? 1 : 0);
  return new Uint8Array(buf);
}

function bytesToConfig(bytes: Uint8Array): RcConfig | null {
  if (bytes.length < CONFIG_PAYLOAD_SIZE) return null;
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const cfg: RcConfig = {
    channel_function: [],
    invert: [],
    calib_min: [],
    calib_max: [],
    calib_center: [],
    deadzone: [],
    rate: [],
    expo: [],
    cutoff_min: [],
    cutoff_max: [],
    high_pass_filter: false,
  };
  let off = 0;
  for (let i = 0; i < NUM_AXES; i++) cfg.channel_function[i] = dv.getUint8(off++);
  for (let i = 0; i < NUM_AXES; i++) cfg.invert[i] = dv.getUint8(off++) !== 0;
  for (let i = 0; i < NUM_AXES; i++) {
    cfg.calib_min[i] = dv.getUint16(off, true);
    off += 2;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    cfg.calib_max[i] = dv.getUint16(off, true);
    off += 2;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    cfg.calib_center[i] = dv.getUint16(off, true);
    off += 2;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    cfg.deadzone[i] = dv.getFloat32(off, true);
    off += 4;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    cfg.rate[i] = dv.getFloat32(off, true);
    off += 4;
  }
  for (let i = 0; i < NUM_AXES; i++) {
    cfg.expo[i] = dv.getFloat32(off, true);
    off += 4;
  }
  for (let i = 0; i < NUM_CHANNELS; i++) {
    cfg.cutoff_min[i] = dv.getUint16(off, true);
    off += 2;
  }
  for (let i = 0; i < NUM_CHANNELS; i++) {
    cfg.cutoff_max[i] = dv.getUint16(off, true);
    off += 2;
  }
  cfg.high_pass_filter = dv.getUint8(off) !== 0;
  return cfg;
}

export function configToPayload(cfg: RcConfig): Uint8Array {
  return configToBytes(cfg);
}

export function parseConfigPayload(payload: Uint8Array): RcConfig | null {
  return bytesToConfig(payload);
}

export interface DeviceInfo {
  status: number;
  schema: number;
  name: string;
  version: string;
}

/**
 * Parse GET_DEVICE_INFO response into structured info.
 * The protocol response already strips status/schema into ProtoResponse,
 * so this parser expects only the data section: name\\0version\\0.
 */
export function parseDeviceInfo(
  data: Uint8Array,
  status: number = STATUS.OK,
  schema: number = 0
): DeviceInfo | null {
  if (!data || data.length < 2) return null;
  let i = 0;
  const nameEnd = data.indexOf(0, i);
  const name = nameEnd >= 0 ? new TextDecoder().decode(data.subarray(i, nameEnd)) : '';
  i = nameEnd >= 0 ? nameEnd + 1 : data.length;
  const verEnd = data.indexOf(0, i);
  const version = verEnd >= 0 ? new TextDecoder().decode(data.subarray(i, verEnd)) : '';
  return { status, schema, name, version };
}

export interface ProtoResponse {
  status: number;
  schema: number;
  data: Uint8Array | null;
}

export function parseResponse(payload: Uint8Array): ProtoResponse {
  if (!payload || payload.length < 3)
    return { status: 255, schema: 0, data: null };
  return {
    status: payload[0]!,
    schema: payload[1]! | (payload[2]! << 8),
    data: payload.length > 3 ? payload.subarray(3) : null,
  };
}

export interface StateFrame {
  adc: number[];
  ch: number[];
  toggles: boolean[];
}

export interface LinkStatus {
  txConnected: boolean;
  txPaired: boolean;
  txState: number;
}

export function parseStateFrame(payload: Uint8Array): StateFrame | null {
  if (!payload || payload.length < 8 + 16 + 4) return null;
  const dv = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const adc: number[] = [];
  for (let i = 0; i < 4; i++) adc.push(dv.getInt16(i * 2, true));
  const ch: number[] = [];
  for (let i = 0; i < 8; i++) ch.push(dv.getUint16(8 + i * 2, true));
  const toggles: boolean[] = [];
  for (let i = 0; i < 4; i++) toggles.push(dv.getUint8(24 + i) !== 0);
  return { adc, ch, toggles };
}

export function parseLinkStatus(payload: Uint8Array | null): LinkStatus | null {
  if (!payload || payload.length < 3) return null;
  return {
    txConnected: payload[0] !== 0,
    txPaired: payload[1] !== 0,
    txState: payload[2] ?? 0xff,
  };
}

export { CONFIG_PAYLOAD_SIZE };
