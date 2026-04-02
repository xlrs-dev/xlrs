import { useEffect, useRef, useState } from 'react';
import { useSerialContext } from '../context/SerialContext';
import { NUM_CHANNELS } from '../types/rc';

const ADC_MAX = 32767;
const STREAM_RETRY_MS = 2000;

export function LivePanel() {
  const { serial, liveState, setLiveState, configDraft, setConfigDraft } = useSerialContext();
  const lastReceivedRef = useRef<number>(0);
  const retryRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const [hpfStatus, setHpfStatus] = useState<string>('');
  const [hpfPending, setHpfPending] = useState(false);
  const [lpfStatus, setLpfStatus] = useState<string>('');
  const [lpfPending, setLpfPending] = useState(false);

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

  const STICK_LPF_OPTIONS = [
    { v: 0, label: 'Off (oversample only)' },
    { v: 1, label: 'Light' },
    { v: 2, label: 'Medium (default)' },
    { v: 3, label: 'Strong' },
  ] as const;

  const setStickLowPass = async (level: number) => {
    const s = Math.max(0, Math.min(3, level));
    const newCfg = { ...configDraft, stick_low_pass: s };
    setConfigDraft(newCfg);
    setLpfPending(true);
    setLpfStatus('');
    try {
      const drafted = await serial.setConfigDraft(newCfg);
      if (!drafted) {
        setLpfStatus('Send failed');
        setLpfPending(false);
        return;
      }
      const applied = await serial.applyConfig();
      setLpfStatus(applied ? 'Applied live. Save elsewhere to persist EEPROM.' : 'Apply failed');
    } catch {
      setLpfStatus('Error');
    } finally {
      setLpfPending(false);
    }
  };

  const toggleHighPass = async () => {
    const newVal = !configDraft.high_pass_filter;
    const newCfg = { ...configDraft, high_pass_filter: newVal };
    setConfigDraft(newCfg);
    setHpfPending(true);
    setHpfStatus('');
    try {
      const drafted = await serial.setConfigDraft(newCfg);
      if (!drafted) {
        setHpfStatus('Send failed');
        setHpfPending(false);
        return;
      }
      const applied = await serial.applyConfig();
      setHpfStatus(applied ? 'Applied live. Save elsewhere to persist EEPROM.' : 'Apply failed');
    } catch {
      setHpfStatus('Error');
    } finally {
      setHpfPending(false);
    }
  };

  return (
    <div className="panel-content">
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        Live data over USB. Requires Chrome or Edge (Web Serial).
      </p>
      <div className="flex flex-wrap items-center gap-6 mb-4">
        <label className="flex items-center gap-2 cursor-pointer">
          <input
            type="checkbox"
            checked={configDraft.high_pass_filter ?? false}
            onChange={toggleHighPass}
            disabled={hpfPending}
            className="rounded border-[var(--color-border)]"
          />
          <span className="text-sm text-[var(--color-text)]">High-pass (slow center drift)</span>
        </label>
        {hpfPending && (
          <span className="text-xs text-[var(--color-text-muted)]">Saving…</span>
        )}
        {!hpfPending && hpfStatus && (
          <span className={`text-xs ${hpfStatus.startsWith('Apply') || hpfStatus === 'Error' ? 'text-[var(--color-error)]' : 'text-[var(--color-success)]'}`}>
            {hpfStatus}
          </span>
        )}
        {!hpfPending && !hpfStatus && (
          <span className="text-xs text-[var(--color-text-muted)]">USB path only when Core1 is off.</span>
        )}

        <label className="flex items-center gap-2 text-sm text-[var(--color-text)]">
          <span>Stick low-pass (noise)</span>
          <select
            className="py-1.5 px-2 rounded border border-[var(--color-input-border)] bg-[var(--color-input-bg)] text-inherit min-w-[200px]"
            value={configDraft.stick_low_pass ?? 2}
            disabled={lpfPending}
            onChange={(e) => setStickLowPass(Number(e.target.value))}
          >
            {STICK_LPF_OPTIONS.map((o) => (
              <option key={o.v} value={o.v}>
                {o.label}
              </option>
            ))}
          </select>
        </label>
        {lpfPending && <span className="text-xs text-[var(--color-text-muted)]">Saving…</span>}
        {!lpfPending && lpfStatus && (
          <span
            className={`text-xs ${lpfStatus.startsWith('Apply') || lpfStatus === 'Error' ? 'text-[var(--color-error)]' : 'text-[var(--color-success)]'}`}
          >
            {lpfStatus}
          </span>
        )}
      </div>
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
