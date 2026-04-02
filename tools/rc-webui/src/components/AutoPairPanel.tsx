import { useCallback, useEffect, useState } from 'react';
import { useSerialContext } from '../context/SerialContext';
import { getPortLabel, useSerial } from '../hooks/useSerial';
import type { LinkStatus } from '../lib/protocol';

type StepState = 'pending' | 'running' | 'done' | 'error';

const PREFLIGHT_POLL_MS = 2000;

interface StepItem {
  id: string;
  title: string;
  detail: string;
  state: StepState;
}

const INITIAL_STEPS: StepItem[] = [
  { id: 'rc', title: 'Verify RC connection', detail: 'Checking RC USB connection.', state: 'pending' },
  { id: 'tx', title: 'Verify TX via RC', detail: 'Checking TX is connected and reachable from RC.', state: 'pending' },
  { id: 'rx', title: 'Verify RX connection', detail: 'Checking RX USB connection.', state: 'pending' },
  { id: 'gen', title: 'Generate phrase', detail: 'Creating a new random binding phrase.', state: 'pending' },
  { id: 'set-rx', title: 'Apply phrase to RX', detail: 'Sending phrase to RX over USB.', state: 'pending' },
  { id: 'set-tx', title: 'Apply phrase to TX', detail: 'Sending phrase to TX through RC.', state: 'pending' },
  { id: 'pair-rx', title: 'Enter RX pairing mode', detail: 'Explicitly putting RX into pairing mode.', state: 'pending' },
  { id: 'pair-tx', title: 'Enter TX pairing mode', detail: 'Explicitly putting TX into pairing mode.', state: 'pending' },
  { id: 'wait', title: 'Wait for pairing', detail: 'Polling RC for TX pairing status.', state: 'pending' },
];

function randomBindingPhrase(): string {
  const chars = 'ABCDEFGHJKLMNPQRSTUVWXYZ23456789';
  const part = (len: number) =>
    Array.from({ length: len }, () => chars[Math.floor(Math.random() * chars.length)]).join('');
  return `QP-${part(4)}-${part(4)}-${part(4)}`;
}

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export function AutoPairPanel() {
  const { serial: rcSerial } = useSerialContext();
  const rxSerial = useSerial();

  const [steps, setSteps] = useState<StepItem[]>(INITIAL_STEPS);
  const [status, setStatus] = useState('');
  const [generatedPhrase, setGeneratedPhrase] = useState('');
  const [busy, setBusy] = useState(false);
  const [linkStatus, setLinkStatus] = useState<LinkStatus | null>(null);

  const rcOk = rcSerial.connected;
  const rxDeviceName = rxSerial.deviceInfo?.name ?? '';
  const rxLooksLikeTarget =
    rxDeviceName.toUpperCase().includes('RX') || rxDeviceName.toUpperCase().includes('SX128X');
  const rxOk = rxSerial.connected && rxLooksLikeTarget;
  const elrsWrongMode = linkStatus?.txType === 2 && linkStatus?.elrsRole === 'rx';
  const txOk = (linkStatus?.txConnected ?? false) && !elrsWrongMode;

  const refreshLinkStatus = useCallback(async () => {
    if (!rcSerial.connected) {
      setLinkStatus(null);
      return;
    }
    const link = await rcSerial.getLinkStatus();
    setLinkStatus(link ?? null);
  }, [rcSerial.connected, rcSerial.getLinkStatus]);

  useEffect(() => {
    if (navigator.serial) rxSerial.refreshPorts();
  }, [rxSerial.refreshPorts]);

  useEffect(() => {
    refreshLinkStatus();
    if (!rcSerial.connected) return;
    const id = setInterval(refreshLinkStatus, PREFLIGHT_POLL_MS);
    return () => clearInterval(id);
  }, [rcSerial.connected, refreshLinkStatus]);

  const preflightReady = rcOk && txOk && rxOk;
  const currentStepIndex =
    steps.findIndex((s) => s.state === 'running') >= 0
      ? steps.findIndex((s) => s.state === 'running') + 1
      : steps.filter((s) => s.state === 'done').length + 1;

  const setStep = (id: string, state: StepState, detail?: string) => {
    setSteps((prev) =>
      prev.map((s) => (s.id === id ? { ...s, state, detail: detail ?? s.detail } : s))
    );
  };

  const resetSteps = () => {
    setSteps(INITIAL_STEPS.map((s) => ({ ...s, state: 'pending' })));
  };

  const runAutoPair = async () => {
    if (busy) return;
    setBusy(true);
    setStatus('');
    resetSteps();

    try {
      setStep('rc', 'running');
      if (!rcSerial.connected) {
        setStep('rc', 'error', 'RC is not connected over USB.');
        setStatus('Connect RC first.');
        return;
      }
      setStep('rc', 'done', 'RC is connected.');

      setStep('tx', 'running');
      const initialLink = await rcSerial.getLinkStatus();
      if (!initialLink?.txConnected) {
        setStep('tx', 'error', 'TX is not connected/available from RC.');
        setStatus('TX is not available from RC. Power TX and check RC-TX UART link.');
        return;
      }
      setStep('tx', 'done', `TX reachable (state ${initialLink.txState}).`);

      setStep('rx', 'running');
      if (!rxSerial.connected) {
        setStep('rx', 'error', 'RX is not connected over USB.');
        setStatus('Connect RX USB in this panel before running auto pair.');
        return;
      }
      if (!rxLooksLikeTarget) {
        setStep('rx', 'error', `Connected device looks wrong (${rxDeviceName || 'unknown device'}).`);
        setStatus('RX port appears to be the wrong device. Connect the RX board USB and try again.');
        return;
      }
      setStep('rx', 'done', 'RX is connected.');

      setStep('gen', 'running');
      const phrase = randomBindingPhrase();
      setGeneratedPhrase(phrase);
      setStep('gen', 'done', `Generated phrase: ${phrase}`);

      setStep('set-rx', 'running');
      const rxOk = await rxSerial.setRxBindingPhrase(phrase);
      if (!rxOk) {
        setStep('set-rx', 'error', 'RX phrase update failed.');
        setStatus('Failed to apply phrase on RX.');
        return;
      }
      setStep('set-rx', 'done', 'RX phrase applied.');

      setStep('set-tx', 'running');
      const txOk = await rcSerial.setTxBindingPhrase(phrase);
      if (!txOk) {
        setStep('set-tx', 'error', 'TX phrase update failed via RC.');
        setStatus('Failed to apply phrase on TX via RC.');
        return;
      }
      setStep('set-tx', 'done', 'TX phrase applied.');

      setStep('pair-rx', 'running');
      const pairRxOk = await rxSerial.enterPairingMode();
      if (!pairRxOk) {
        setStep('pair-rx', 'error', 'Failed to put RX into pairing mode.');
        setStatus('RX did not enter pairing mode.');
        return;
      }
      setStep('pair-rx', 'done', 'RX pairing mode confirmed.');

      setStep('pair-tx', 'running');
      const pairTxOk = await rcSerial.enterPairingMode();
      if (!pairTxOk) {
        setStep('pair-tx', 'error', 'Failed to put TX into pairing mode via RC.');
        setStatus('TX did not enter pairing mode.');
        return;
      }
      setStep('pair-tx', 'done', 'TX pairing mode command sent.');

      setStep('wait', 'running');
      const start = Date.now();
      let paired = false;
      while (Date.now() - start < 60000) {
        const link = await rcSerial.getLinkStatus();
        if (link?.txPaired) {
          paired = true;
          break;
        }
        await sleep(1000);
      }

      if (!paired) {
        setStep('wait', 'error', 'Timed out waiting for pairing.');
        setStatus('Pairing did not complete within 60s.');
        return;
      }

      setStep('wait', 'done', 'Pairing completed successfully.');
      setStatus(`Success: TX/RX paired with phrase "${phrase}".`);
    } finally {
      setBusy(false);
    }
  };

  const handleRxPortChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const value = e.target.value;
    if (value === '') return;
    if (value === 'new') {
      rxSerial.connectNew();
      e.target.value = '';
      return;
    }
    const i = Number(value);
    if (!Number.isNaN(i) && rxSerial.availablePorts[i]) {
      rxSerial.connectToPort(rxSerial.availablePorts[i]!);
      e.target.value = '';
    }
  };

  const txStateLabel =
    linkStatus?.txState !== undefined && linkStatus.txState !== 0xff
      ? ['Disconnected', 'Pairing', 'Connecting', 'Connected', 'Lost'][linkStatus.txState] ?? `State ${linkStatus.txState}`
      : '';
  const txTypeLabel =
    linkStatus?.txType === 1 ? 'Custom' : linkStatus?.txType === 2 ? 'ELRS' : linkStatus?.txType === 0 ? 'Unknown' : '';
  const txTypeDetail = txTypeLabel
    ? linkStatus?.txType === 2 && linkStatus?.elrsDeviceName
      ? `TX: ELRS (${linkStatus.elrsDeviceName})`
      : `TX: ${txTypeLabel}`
    : '';

  const handleReDetectTx = useCallback(async () => {
    await rcSerial.reDetectTx();
    await refreshLinkStatus();
  }, [rcSerial.reDetectTx, refreshLinkStatus]);

  return (
    <div className="panel-content">
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        One-click pairing: verify RC→TX link, generate a new phrase, apply to RX+TX, then wait for
        pairing and report result.
      </p>

      <div className="preflight-card">
        <h4 className="text-[var(--color-text)] font-semibold">Preflight checks</h4>
        <p className="hint">All three must be green before running auto pair.</p>
        <ul className="preflight-list" aria-label="Preflight status">
          <li className={rcOk ? 'ok' : 'fail'}>
            <span className="preflight-dot" aria-hidden />
            <span>RC connected</span>
            {rcOk && <span className="preflight-detail">USB connected</span>}
            {!rcOk && <span className="preflight-detail">Connect RC in the header</span>}
          </li>
          <li className={txOk ? 'ok' : 'fail'}>
            <span className="preflight-dot" aria-hidden />
            <span>TX reachable via RC</span>
            {txOk && (
              <span className="preflight-detail">
                {txTypeDetail && `${txTypeDetail} — `}
                {linkStatus?.txPaired ? 'Paired' : 'Reachable'}
                {txStateLabel ? ` — ${txStateLabel}` : ''}
              </span>
            )}
            {elrsWrongMode && (
              <span className="preflight-detail text-[var(--color-error)]">
                ELRS RX (wrong mode) — need transmitter module
              </span>
            )}
            {!txOk && !elrsWrongMode && (
              <span className="preflight-detail">
                {rcOk ? 'Power TX and connect to RC' : 'RC must be connected first'}
              </span>
            )}
          </li>
          <li className={rxOk ? 'ok' : 'fail'}>
            <span className="preflight-dot" aria-hidden />
            <span>RX connected</span>
            {rxOk && (
              <span className="preflight-detail">
                USB connected ({rxDeviceName || 'RX'})
              </span>
            )}
            {!rxSerial.connected && <span className="preflight-detail">Connect RX below</span>}
            {rxSerial.connected && !rxLooksLikeTarget && (
              <span className="preflight-detail">
                Wrong device selected ({rxDeviceName || 'unknown'}). Pick the RX board.
              </span>
            )}
          </li>
        </ul>
        <div className="flex flex-wrap gap-2 mt-2">
          <button type="button" className="btn-secondary preflight-refresh" onClick={refreshLinkStatus}>
            Refresh TX status
          </button>
          <button type="button" className="btn-secondary" onClick={handleReDetectTx}>
            Re-detect TX
          </button>
        </div>
      </div>

      <div className="card auto-pair-card mt-4">
        <h4 className="text-[var(--color-text)] font-semibold">RX USB connection</h4>
        <div className="flex flex-wrap items-center gap-3 mt-2">
          {rxSerial.connected ? (
            <>
              <span className="text-sm">Connected: {rxSerial.portName ?? 'RX serial'}</span>
              <button type="button" className="btn-secondary" onClick={rxSerial.disconnect}>
                Disconnect RX
              </button>
            </>
          ) : (
            <>
              <select
                className="input-select"
                value=""
                onChange={handleRxPortChange}
                aria-label="Select RX USB device"
              >
                <option value="">Select RX device…</option>
                {rxSerial.availablePorts.map((port, i) => (
                  <option key={i} value={i}>
                    {getPortLabel(port)}
                  </option>
                ))}
                <option value="new">Add new device…</option>
              </select>
              <button type="button" className="btn-secondary" onClick={rxSerial.connectNew}>
                Add RX device
              </button>
            </>
          )}
          <button type="button" className="btn-secondary" onClick={rxSerial.refreshPorts}>
            Refresh ports
          </button>
        </div>
      </div>

      <div className="btn-row mt-6">
        <button
          type="button"
          className="btn-primary"
          onClick={runAutoPair}
          disabled={busy || !preflightReady || elrsWrongMode}
          title={
            elrsWrongMode
              ? 'Connected device is an ELRS receiver; use an ELRS transmitter module.'
              : !preflightReady
                ? 'Complete all preflight checks first'
                : undefined
          }
        >
          {busy
            ? `Running… Step ${currentStepIndex} of ${steps.length}`
            : preflightReady
              ? 'Auto pair now'
              : 'Complete preflight first'}
        </button>
      </div>

      <p className="text-sm text-[var(--color-text-muted)] mt-2">
        {busy ? `Step ${currentStepIndex} of ${steps.length}` : 'Steps run in order when you click Auto pair now.'}
      </p>

      <div className="step-list">
        {steps.map((s) => (
          <div key={s.id} className={`step-item ${s.state}`}>
            <div className="step-title">
              <span className={`step-dot ${s.state}`} />
              {s.title}
            </div>
            <div className="step-detail">{s.detail}</div>
          </div>
        ))}
      </div>

      {generatedPhrase && (
        <p className="mt-4 text-sm text-[var(--color-text-muted)]">
          Last generated phrase: <span className="font-mono">{generatedPhrase}</span>
        </p>
      )}
      {status && (
        <p
          className={`mt-2 text-sm ${
            status.toLowerCase().startsWith('success') ? 'text-[var(--color-success)]' : 'text-[var(--color-text)]'
          }`}
        >
          {status}
        </p>
      )}
    </div>
  );
}
