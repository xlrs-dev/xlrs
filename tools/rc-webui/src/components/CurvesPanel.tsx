import { useSerialContext } from '../context/SerialContext';
import { NUM_AXES, NUM_CHANNELS, AXIS_LABELS } from '../types/rc';

export function CurvesPanel() {
  const { configDraft, setConfigDraft, serial } = useSerialContext();

  const setDeadzone = (i: number, v: number) => {
    setConfigDraft((prev) => ({
      ...prev,
      deadzone: prev.deadzone.map((x, j) => (j === i ? v : x)),
    }));
  };
  const setRate = (i: number, v: number) => {
    setConfigDraft((prev) => ({
      ...prev,
      rate: prev.rate.map((x, j) => (j === i ? v : x)),
    }));
  };
  const setExpo = (i: number, v: number) => {
    setConfigDraft((prev) => ({
      ...prev,
      expo: prev.expo.map((x, j) => (j === i ? v : x)),
    }));
  };
  const setCutoffMin = (i: number, v: number) => {
    setConfigDraft((prev) => ({
      ...prev,
      cutoff_min: prev.cutoff_min.map((x, j) => (j === i ? v : x)),
    }));
  };
  const setCutoffMax = (i: number, v: number) => {
    setConfigDraft((prev) => ({
      ...prev,
      cutoff_max: prev.cutoff_max.map((x, j) => (j === i ? v : x)),
    }));
  };

  const pushDraft = async () => {
    await serial.setConfigDraft(configDraft);
    await serial.applyConfig();
  };

  if (!serial.connected) {
    return <p className="hint">Connect USB first.</p>;
  }

  return (
    <div className="panel-content">
      <p>Per-axis deadzone (0–0.2), rate (0.5–1), expo (0–0.5). Cutoffs: channel min/max µs.</p>
      <div className="curves-grid">
        <h4>Axes</h4>
        {AXIS_LABELS.map((_, i) => (
          <div key={i} className="curve-row">
            <span>{AXIS_LABELS[i]}</span>
            <label>DZ <input type="number" min={0} max={0.5} step={0.01} value={configDraft.deadzone[i]} onChange={(e) => setDeadzone(i, Number(e.target.value))} /></label>
            <label>Rate <input type="number" min={0.3} max={1} step={0.05} value={configDraft.rate[i]} onChange={(e) => setRate(i, Number(e.target.value))} /></label>
            <label>Expo <input type="number" min={0} max={1} step={0.05} value={configDraft.expo[i]} onChange={(e) => setExpo(i, Number(e.target.value))} /></label>
          </div>
        ))}
        <h4>Channel cutoffs (µs)</h4>
        {Array.from({ length: NUM_CHANNELS }, (_, i) => (
          <div key={i} className="curve-row">
            <span>Ch{i}</span>
            <label>Min <input type="number" min={1000} max={2000} value={configDraft.cutoff_min[i]} onChange={(e) => setCutoffMin(i, Number(e.target.value))} /></label>
            <label>Max <input type="number" min={1000} max={2000} value={configDraft.cutoff_max[i]} onChange={(e) => setCutoffMax(i, Number(e.target.value))} /></label>
          </div>
        ))}
      </div>
      <button type="button" className="btn primary" onClick={pushDraft}>
        Apply to device
      </button>
    </div>
  );
}
