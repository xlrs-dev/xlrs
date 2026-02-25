import { useEffect, useRef } from 'react';
import { useSerialContext } from '../context/SerialContext';
import { NUM_CHANNELS } from '../types/rc';

const ADC_MAX = 32767;
const STREAM_RETRY_MS = 2000;

export function LivePanel() {
  const { serial, liveState, setLiveState } = useSerialContext();
  const lastReceivedRef = useRef<number>(0);
  const retryRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    if (!serial.connected) return;

    const onState = (s: { adc: number[]; ch: number[]; toggles: boolean[] }) => {
      lastReceivedRef.current = Date.now();
      setLiveState(s);
    };

    serial.streamStart(onState);
    lastReceivedRef.current = Date.now();

    retryRef.current = window.setInterval(() => {
      if (Date.now() - lastReceivedRef.current > STREAM_RETRY_MS) {
        serial.streamStart(onState);
      }
    }, STREAM_RETRY_MS);

    return () => {
      if (retryRef.current) {
        clearInterval(retryRef.current);
        retryRef.current = null;
      }
      serial.streamStop();
      setLiveState(null);
    };
  }, [serial.connected, serial.streamStart, serial.streamStop, setLiveState]);

  if (!serial.connected) {
    return (
      <p className="hint">
        Connect USB first, then open this tab to see live sticks and channels.
      </p>
    );
  }

  const s = liveState;
  const adc = s?.adc ?? [0, 0, 0, 0];
  const ch = s?.ch ?? Array(NUM_CHANNELS).fill(1500);
  const toggles = s?.toggles ?? [false, false, false, false];

  return (
    <div className="panel-content">
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        Live data over USB. Requires Chrome or Edge (Web Serial).
      </p>
      <div className="viz-row">
        <div className="stick-viz">
          <h3 className="text-base font-semibold text-[var(--color-text)] mb-2">Sticks (ADC)</h3>
          <div className="stick-grid">
            {adc.map((v: number, i: number) => (
              <div key={i} className="stick-cell">
                <span>Axis {i}</span>
                <div className="bar-wrap">
                  <div
                    className="bar"
                    style={{
                      width: `${Math.max(0, Math.min(100, ((v + ADC_MAX) / (2 * ADC_MAX)) * 100))}%`,
                    }}
                  />
                </div>
                <span>{v}</span>
              </div>
            ))}
          </div>
        </div>
        <div className="ch-viz">
          <h3 className="text-base font-semibold text-[var(--color-text)] mb-2">Channels (µs)</h3>
          <div className="ch-grid">
            {ch.map((v: number, i: number) => (
              <div key={i} className="ch-cell">
                <span>Ch{i}</span>
                <div className="bar-wrap">
                  <div
                    className="bar ch-bar"
                    style={{ width: `${((v - 1000) / 1000) * 100}%` }}
                  />
                </div>
                <span>{v}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
      <div className="toggles-row">
        <h3 className="text-base font-semibold text-[var(--color-text)] mt-4 mb-2">Switches</h3>
        <div className="toggles">
          {toggles.map((on: boolean, i: number) => (
            <span key={i} className={`toggle-badge ${on ? 'on' : 'off'}`}>
              SW{i + 1} {on ? 'ON' : 'OFF'}
            </span>
          ))}
        </div>
      </div>
    </div>
  );
}
