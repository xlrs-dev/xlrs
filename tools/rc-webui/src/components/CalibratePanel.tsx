import { useState, useEffect, useRef } from 'react';
import { useSerialContext } from '../context/SerialContext';

const ADC_MAX = 32767;
const STREAM_RETRY_MS = 2000;

const STEPS = [
  { key: 'start', label: 'Start', description: 'Start calibration' },
  { key: 'sample', label: 'Sample', description: 'Sample stick positions' },
  { key: 'finish', label: 'Finish & save', description: 'Save to device' },
] as const;

export function CalibratePanel() {
  const { serial, liveState, setLiveState } = useSerialContext();
  const [status, setStatus] = useState<string>('');
  const [statusType, setStatusType] = useState<'idle' | 'success' | 'error'>('idle');
  const [currentStep, setCurrentStep] = useState<number>(0);
  const lastReceivedRef = useRef<number>(0);
  const retryRef = useRef<ReturnType<typeof setInterval> | null>(null);

  // Keep state stream active while on Calibrate tab so we can show live ADC
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

  const handleStart = async () => {
    setStatus('Starting…');
    setStatusType('idle');
    setCurrentStep(1);
    const ok = await serial.calStart();
    setStatus(
      ok ? 'Move sticks through full range, then click Sample a few times.' : 'Failed to start.'
    );
    setStatusType(ok ? 'success' : 'error');
  };

  const handleSample = async () => {
    setStatus('Sampling…');
    setStatusType('idle');
    setCurrentStep(2);
    const ok = await serial.calSample();
    setStatus(ok ? 'Sampled. Click Finish when done.' : 'Sample failed.');
    setStatusType(ok ? 'success' : 'error');
  };

  const handleFinish = async () => {
    setStatus('Finishing…');
    setStatusType('idle');
    setCurrentStep(3);
    const ok = await serial.calFinish();
    setStatus(ok ? 'Calibration saved to device.' : 'Finish failed.');
    setStatusType(ok ? 'success' : 'error');
  };

  if (!serial.connected) {
    return <p className="hint">Connect USB first.</p>;
  }

  const s = liveState;
  const adc = s?.adc ?? [0, 0, 0, 0];

  return (
    <div className="panel-content">
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        Move all sticks through their full range, then click Finish. Center is (min+max)/2.
      </p>

      {serial.connected && (
        <div className="stick-viz mb-6 p-3 rounded-[var(--radius-card)] border border-[var(--color-border)] bg-[var(--color-card)]">
          <h3 className="text-sm font-semibold text-[var(--color-text)] mb-2">Live ADC (sticks)</h3>
          <div className="stick-grid grid grid-cols-2 sm:grid-cols-4 gap-2">
            {adc.map((v: number, i: number) => (
              <div key={i} className="stick-cell flex flex-col gap-1">
                <span className="text-xs text-[var(--color-text-muted)]">Axis {i}</span>
                <div className="bar-wrap h-2 rounded bg-[var(--color-input-bg)] overflow-hidden">
                  <div
                    className="bar h-full bg-[var(--color-primary)] rounded"
                    style={{
                      width: `${Math.max(0, Math.min(100, ((v + ADC_MAX) / (2 * ADC_MAX)) * 100))}%`,
                    }}
                  />
                </div>
                <span className="text-xs font-mono text-[var(--color-text)]">{v}</span>
              </div>
            ))}
          </div>
        </div>
      )}

      <div className="flex flex-wrap items-center gap-4 mb-6">
        {STEPS.map((step, i) => (
          <div key={step.key} className="flex items-center gap-2">
            <span
              className={`flex h-8 w-8 items-center justify-center rounded-full text-sm font-medium ${
                currentStep > i + 1
                  ? 'bg-[var(--color-success)] text-white'
                  : currentStep === i + 1
                    ? 'bg-[var(--color-primary)] text-white'
                    : 'bg-[var(--color-input-bg)] text-[var(--color-text-muted)]'
              }`}
            >
              {currentStep > i + 1 ? '✓' : i + 1}
            </span>
            <span className="text-sm text-[var(--color-text-muted)]">{step.label}</span>
            {i < STEPS.length - 1 && (
              <span className="text-[var(--color-border)] mx-1">→</span>
            )}
          </div>
        ))}
      </div>

      <div className="btn-row">
        <button type="button" className="btn-secondary" onClick={handleStart}>
          Start calibration
        </button>
        <button type="button" className="btn-secondary" onClick={handleSample}>
          Sample
        </button>
        <button type="button" className="btn-primary" onClick={handleFinish}>
          Finish & save
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
