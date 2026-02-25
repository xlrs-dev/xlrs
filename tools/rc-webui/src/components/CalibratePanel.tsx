import { useState } from 'react';
import { useSerialContext } from '../context/SerialContext';

const STEPS = [
  { key: 'start', label: 'Start', description: 'Start calibration' },
  { key: 'sample', label: 'Sample', description: 'Sample stick positions' },
  { key: 'finish', label: 'Finish & save', description: 'Save to device' },
] as const;

export function CalibratePanel() {
  const { serial } = useSerialContext();
  const [status, setStatus] = useState<string>('');
  const [statusType, setStatusType] = useState<'idle' | 'success' | 'error'>('idle');
  const [currentStep, setCurrentStep] = useState<number>(0);

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

  return (
    <div className="panel-content">
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        Move all sticks through their full range, then click Finish. Center is (min+max)/2.
      </p>

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
