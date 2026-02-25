import { useState } from 'react';
import { useSerialContext } from '../context/SerialContext';

export function CalibratePanel() {
  const { serial } = useSerialContext();
  const [status, setStatus] = useState<string>('');

  const handleStart = async () => {
    setStatus('Starting…');
    const ok = await serial.calStart();
    setStatus(ok ? 'Move sticks through full range, then click Sample a few times.' : 'Failed to start.');
  };

  const handleSample = async () => {
    const ok = await serial.calSample();
    setStatus(ok ? 'Sampled.' : 'Sample failed.');
  };

  const handleFinish = async () => {
    setStatus('Finishing…');
    const ok = await serial.calFinish();
    setStatus(ok ? 'Calibration saved to device.' : 'Finish failed.');
  };

  if (!serial.connected) {
    return <p className="hint">Connect USB first.</p>;
  }

  return (
    <div className="panel-content">
      <p>Move all sticks through their full range, then click Finish. Center is (min+max)/2.</p>
      <div className="btn-row">
        <button type="button" className="btn" onClick={handleStart}>
          Start calibration
        </button>
        <button type="button" className="btn" onClick={handleSample}>
          Sample
        </button>
        <button type="button" className="btn primary" onClick={handleFinish}>
          Finish & save
        </button>
      </div>
      <p className="status">{status}</p>
    </div>
  );
}
