import { useState } from 'react';
import { useSerialContext } from '../context/SerialContext';
import { LABELS, AXIS_LABELS, CHANNEL_LABELS, CHANNEL_TRIM_MAX_ABS, NUM_CHANNELS } from '../types/rc';

type StatusType = 'idle' | 'success' | 'error';

export function MappingPanel() {
  const { configDraft, setConfigDraft, serial, loadConfigFromDevice } = useSerialContext();
  const [status, setStatus] = useState('');
  const [statusType, setStatusType] = useState<StatusType>('idle');

  const showStatus = (msg: string, type: StatusType) => {
    setStatus(msg);
    setStatusType(type);
  };

  const setAxisFunction = (axis: number, value: number) => {
    setConfigDraft((prev) => {
      const next = { ...prev, channel_function: [...(prev.channel_function ?? [0, 1, 2, 3])] };
      next.channel_function[axis] = value;
      return next;
    });
  };

  const setAxisInvert = (axis: number, value: boolean) => {
    setConfigDraft((prev) => {
      const next = { ...prev, invert: [...(prev.invert ?? [false, false, false, false])] };
      next.invert[axis] = value;
      return next;
    });
  };

  const setChannelTrim = (ch: number, value: number) => {
    const v = Math.max(-CHANNEL_TRIM_MAX_ABS, Math.min(CHANNEL_TRIM_MAX_ABS, Math.round(value)));
    setConfigDraft((prev) => {
      const next = { ...prev, channel_trim: [...(prev.channel_trim ?? Array(NUM_CHANNELS).fill(0))] };
      next.channel_trim[ch] = v;
      return next;
    });
  };

  const handleLoad = async () => {
    showStatus('Loading…', 'idle');
    await loadConfigFromDevice();
    showStatus('Config loaded from device.', 'success');
  };

  const applyDraftToDevice = async (): Promise<boolean> => {
    const ok = await serial.setConfigDraft(configDraft);
    if (!ok) return false;
    return serial.applyConfig();
  };

  const handleApply = async () => {
    showStatus('Applying…', 'idle');
    const applied = await applyDraftToDevice();
    showStatus(applied ? 'Applied to device.' : 'Apply failed.', applied ? 'success' : 'error');
  };

  const handleSave = async () => {
    showStatus('Pushing & saving to EEPROM…', 'idle');
    const sent = await serial.setConfigDraft(configDraft);
    if (!sent) {
      showStatus('Failed to send config.', 'error');
      return;
    }
    const saved = await serial.saveConfig();
    showStatus(
      saved ? 'Saved to EEPROM (live config updated).' : 'Save failed.',
      saved ? 'success' : 'error'
    );
  };

  if (!serial.connected) {
    return (
      <p className="hint">
        Connect USB first. Then load config to pull current mapping.
      </p>
    );
  }

  return (
    <div className="panel-content">
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        Assign each physical axis to a function (Aileron, Elevator, Rudder, Throttle). Invert
        reverses direction. Load first to pull current config, then Apply to use changes, Save to persist.
      </p>
      <div className="mapping-grid">
        {AXIS_LABELS.map((_, axis) => (
          <div
            key={axis}
            className="card flex flex-wrap items-center gap-4 py-3"
            style={{ marginBottom: '0.5rem' }}
          >
            <span className="font-medium text-sm min-w-[72px]">{AXIS_LABELS[axis]}</span>
            <label className="flex items-center gap-2 text-sm">
              Function
              <select
                value={configDraft.channel_function?.[axis] ?? axis}
                onChange={(e) => setAxisFunction(axis, Number(e.target.value))}
                className="min-w-[120px] py-2 px-2 rounded border border-[var(--color-input-border)] bg-[var(--color-input-bg)] text-inherit"
              >
                {LABELS.map((label, i) => (
                  <option key={i} value={i}>
                    {label}
                  </option>
                ))}
              </select>
            </label>
            <label className="checkbox-label text-sm cursor-pointer">
              <input
                type="checkbox"
                checked={configDraft.invert?.[axis] ?? false}
                onChange={(e) => setAxisInvert(axis, e.target.checked)}
                className="mr-1"
              />
              Invert
            </label>
          </div>
        ))}
      </div>

      <div className="mt-8">
        <h4 className="text-sm font-semibold text-[var(--color-text)] mb-2">Trim (µs)</h4>
        <p className="text-[var(--color-text-muted)] text-sm mb-3">
          Constant bias per output channel (subtrim). Applied after stick mapping, before cutoffs. Range ±
          {CHANNEL_TRIM_MAX_ABS} µs.
        </p>
        <div className="grid gap-3 sm:grid-cols-2">
          {CHANNEL_LABELS.map((label, ch) => (
            <label key={ch} className="flex flex-col gap-1 text-sm">
              <span className="text-[var(--color-text-muted)]">
                {label}: {configDraft.channel_trim?.[ch] ?? 0} µs
              </span>
              <input
                type="range"
                min={-CHANNEL_TRIM_MAX_ABS}
                max={CHANNEL_TRIM_MAX_ABS}
                step={1}
                value={configDraft.channel_trim?.[ch] ?? 0}
                onChange={(e) => setChannelTrim(ch, Number(e.target.value))}
                className="w-full"
              />
            </label>
          ))}
        </div>
      </div>

      <div className="flex flex-wrap gap-2 mt-4">
        <button type="button" className="btn-secondary" onClick={handleLoad}>
          Load from device
        </button>
        <button type="button" className="btn-secondary" onClick={handleApply}>
          Apply to device
        </button>
        <button type="button" className="btn-primary" onClick={handleSave}>
          Save to EEPROM
        </button>
      </div>
      {status && (
        <p
          role="status"
          aria-live="polite"
          className={`mt-4 text-sm ${
            statusType === 'success'
              ? 'text-[var(--color-success)]'
              : statusType === 'error'
                ? 'text-[var(--color-error)]'
                : 'text-[var(--color-text-muted)]'
          }`}
        >
          {status}
        </p>
      )}
    </div>
  );
}
