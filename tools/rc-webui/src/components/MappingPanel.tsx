import { useSerialContext } from '../context/SerialContext';
import { LABELS, AXIS_LABELS } from '../types/rc';

export function MappingPanel() {
  const { configDraft, setConfigDraft, serial } = useSerialContext();

  const setAxisFunction = (axis: number, value: number) => {
    setConfigDraft((prev) => {
      const next = { ...prev, channel_function: [...prev.channel_function] };
      next.channel_function[axis] = value;
      return next;
    });
  };

  const setAxisInvert = (axis: number, value: boolean) => {
    setConfigDraft((prev) => {
      const next = { ...prev, invert: [...prev.invert] };
      next.invert[axis] = value;
      return next;
    });
  };

  const pushDraft = async () => {
    await serial.setConfigDraft(configDraft);
    await serial.applyConfig();
  };

  if (!serial.connected) {
    return (
      <p className="hint">
        Connect USB first. Load config from Save / Apply tab to pull current mapping.
      </p>
    );
  }

  return (
    <div className="panel-content">
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        Assign each physical axis to a function (Aileron, Elevator, Rudder, Throttle). Invert
        reverses direction.
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
                value={configDraft.channel_function[axis]}
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
                checked={configDraft.invert[axis]}
                onChange={(e) => setAxisInvert(axis, e.target.checked)}
                className="mr-1"
              />
              Invert
            </label>
          </div>
        ))}
      </div>
      <div className="mt-4">
        <button type="button" className="btn-primary" onClick={pushDraft}>
          Apply to device
        </button>
      </div>
    </div>
  );
}
