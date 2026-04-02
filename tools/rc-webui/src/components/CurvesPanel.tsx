import { useSerialContext } from '../context/SerialContext';
import { NUM_CHANNELS, AXIS_LABELS } from '../types/rc';
import * as Slider from '@radix-ui/react-slider';

function SliderField({
  label,
  value,
  min,
  max,
  step,
  onValueChange,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onValueChange: (v: number) => void;
}) {
  return (
    <div className="curves-slider-field">
      <span className="curves-slider-label">{label}</span>
      <Slider.Root
        className="curves-slider-root"
        value={[value]}
        onValueChange={(vals) => onValueChange(vals[0] ?? value)}
        min={min}
        max={max}
        step={step}
        aria-label={label}
      >
        <Slider.Track className="curves-slider-track">
          <Slider.Range className="curves-slider-range" />
        </Slider.Track>
        <Slider.Thumb className="curves-slider-thumb" />
      </Slider.Root>
      <span className="curves-slider-value">{value.toFixed(2)}</span>
    </div>
  );
}

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
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        Per-axis deadzone (0–0.2), rate (0.5–1), expo (0–0.5). Cutoffs: channel min/max µs.
      </p>
      <div className="curves-grid">
        <h4 className="text-[var(--color-text)] font-semibold mt-4 mb-1">Per-axis</h4>
        {AXIS_LABELS.map((_, i) => (
          <div
            key={i}
            className="curves-axis-row"
          >
            <span className="curves-axis-name">{AXIS_LABELS[i]}</span>
            <SliderField
              label="DZ"
              value={configDraft.deadzone[i]!}
              min={0}
              max={0.5}
              step={0.01}
              onValueChange={(v) => setDeadzone(i, v)}
            />
            <SliderField
              label="Rate"
              value={configDraft.rate[i]!}
              min={0.3}
              max={1}
              step={0.05}
              onValueChange={(v) => setRate(i, v)}
            />
            <SliderField
              label="Expo"
              value={configDraft.expo[i]!}
              min={0}
              max={1}
              step={0.05}
              onValueChange={(v) => setExpo(i, v)}
            />
          </div>
        ))}
        <h4 className="text-[var(--color-text)] font-semibold mt-6 mb-1">Channel cutoffs (µs)</h4>
        <div className="flex flex-col gap-2">
          {Array.from({ length: NUM_CHANNELS }, (_, i) => (
            <div key={i} className="curve-row flex flex-wrap items-center gap-3">
              <span className="min-w-[48px] font-medium text-sm">Ch{i}</span>
              <label className="inline-flex items-center gap-2 text-sm">
                Min
                <input
                  type="number"
                  min={1000}
                  max={2000}
                  value={configDraft.cutoff_min[i]}
                  onChange={(e) => setCutoffMin(i, Number(e.target.value))}
                  className="w-20 py-1.5 px-2 rounded border border-[var(--color-input-border)] bg-[var(--color-input-bg)] text-inherit"
                />
              </label>
              <label className="inline-flex items-center gap-2 text-sm">
                Max
                <input
                  type="number"
                  min={1000}
                  max={2000}
                  value={configDraft.cutoff_max[i]}
                  onChange={(e) => setCutoffMax(i, Number(e.target.value))}
                  className="w-20 py-1.5 px-2 rounded border border-[var(--color-input-border)] bg-[var(--color-input-bg)] text-inherit"
                />
              </label>
            </div>
          ))}
        </div>
      </div>
      <div className="mt-4">
        <button type="button" className="btn-primary" onClick={pushDraft}>
          Apply to device
        </button>
      </div>
    </div>
  );
}
