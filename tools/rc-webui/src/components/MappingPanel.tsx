import { useSerialContext } from '../context/SerialContext';
import { NUM_AXES, LABELS, AXIS_LABELS } from '../types/rc';

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
    return <p className="hint">Connect USB first. Load config to pull current mapping.</p>;
  }

  return (
    <div className="panel-content">
      <p>Assign each physical axis to a function (A/E/R/T). Invert reverses direction.</p>
      <div className="mapping-grid">
        {AXIS_LABELS.map((_, axis) => (
          <div key={axis} className="mapping-row">
            <label>{AXIS_LABELS[axis]}</label>
            <select
              value={configDraft.channel_function[axis]}
              onChange={(e) => setAxisFunction(axis, Number(e.target.value))}
            >
              {LABELS.map((label, i) => (
                <option key={i} value={i}>
                  {label}
                </option>
              ))}
            </select>
            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={configDraft.invert[axis]}
                onChange={(e) => setAxisInvert(axis, e.target.checked)}
              />
              Invert
            </label>
          </div>
        ))}
      </div>
      <button type="button" className="btn primary" onClick={pushDraft}>
        Apply to device
      </button>
    </div>
  );
}
