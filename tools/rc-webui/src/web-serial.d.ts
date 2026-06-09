interface SerialPortInfo {
  usbVendorId?: number;
  usbProductId?: number;
}

interface SerialPort extends EventTarget {
  readonly readable: ReadableStream<Uint8Array> | null;
  readonly writable: WritableStream<Uint8Array> | null;
  getInfo(): SerialPortInfo;
  open(options: { baudRate: number; bufferSize?: number }): Promise<void>;
  close(): Promise<void>;
}

interface Serial extends EventTarget {
  getPorts(): Promise<SerialPort[]>;
  requestPort(options?: { filters?: SerialPortFilter[] }): Promise<SerialPort>;
  addEventListener(type: "connect" | "disconnect", listener: (event: Event) => void): void;
}

interface SerialPortFilter {
  usbVendorId?: number;
  usbProductId?: number;
}

interface Navigator {
  readonly serial?: Serial;
}
