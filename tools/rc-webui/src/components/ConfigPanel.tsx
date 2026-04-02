import { useState, useRef } from 'react';
import { useSerialContext } from '../context/SerialContext';
import { defaultRcConfig } from '../types/rc';

type StatusType = 'idle' | 'success' | 'error';

const PROXY_CONFIRM_MSG =
  'RC will forward raw bytes between USB and the TX module. This WebUI will disconnect. Use ELRS Buddy or similar to configure your TX directly. Reset RC to exit proxy mode. Continue?';

export function ConfigPanel() {
  const { serial, configDraft, setConfigDraft, loadConfigFromDevice } = useSerialContext();
  const [status, setStatus] = useState('');
  const [statusType, setStatusType] = useState<StatusType>('idle');
  const [bindingPhrase, setBindingPhrase] = useState('');
  const [elrsPhraseView, setElrsPhraseView] = useState<string | null>(null);
  const [proxyConfirmOpen, setProxyConfirmOpen] = useState(false);
  const [proxyBusy, setProxyBusy] = useState(false);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const showStatus = (msg: string, type: StatusType) => {
    setStatus(msg);
    setStatusType(type);
  };

  const handleApply = async () => {
    showStatus('Applying…', 'idle');
    const ok = await serial.setConfigDraft(configDraft);
    if (ok) {
      const applied = await serial.applyConfig();
      showStatus(applied ? 'Applied to device.' : 'Apply failed.', applied ? 'success' : 'error');
    } else {
      showStatus('Set draft failed.', 'error');
    }
  };

  const handleSave = async () => {
    showStatus('Pushing config & saving to EEPROM…', 'idle');
    const sent = await serial.setConfigDraft(configDraft);
    if (!sent) {
      showStatus('Failed to send config to device.', 'error');
      return;
    }
    const ok = await serial.saveConfig();
    showStatus(ok ? 'Saved to EEPROM (live config updated).' : 'Save failed.', ok ? 'success' : 'error');
  };

  const handleLoad = async () => {
    showStatus('Loading…', 'idle');
    await loadConfigFromDevice();
    showStatus('Config loaded from device.', 'success');
  };

  const handleExport = () => {
    const blob = new Blob([JSON.stringify(configDraft, null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'rc-config.json';
    a.click();
    URL.revokeObjectURL(a.href);
    showStatus('Exported.', 'success');
  };

  const handleImport = () => {
    fileInputRef.current?.click();
  };

  const onFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const r = new FileReader();
    r.onload = () => {
      try {
        const cfg = JSON.parse(r.result as string);
        setConfigDraft({ ...defaultRcConfig(), ...cfg });
        showStatus('Imported. Apply to device to use.', 'success');
      } catch {
        showStatus('Invalid JSON.', 'error');
      }
    };
    r.readAsText(file);
    e.target.value = '';
  };

  const validateBindingPhrase = (): string | null => {
    const trimmed = bindingPhrase.trim();
    if (!trimmed) return 'Enter a binding phrase first.';
    const len = new TextEncoder().encode(trimmed).length;
    if (len > 32) return 'Binding phrase must be 32 bytes or less.';
    return null;
  };

  const handleSetRxBindingPhrase = async () => {
    const err = validateBindingPhrase();
    if (err) {
      showStatus(err, 'error');
      return;
    }
    showStatus('Sending RX binding phrase update...', 'idle');
    const ok = await serial.setRxBindingPhrase(bindingPhrase);
    showStatus(
      ok ? 'RX binding phrase update sent.' : 'RX binding phrase update failed.',
      ok ? 'success' : 'error'
    );
  };

  const handleSetTxBindingPhrase = async () => {
    const err = validateBindingPhrase();
    if (err) {
      showStatus(err, 'error');
      return;
    }
    showStatus('Sending TX binding phrase update...', 'idle');
    const ok = await serial.setTxBindingPhrase(bindingPhrase);
    showStatus(
      ok ? 'TX binding phrase updated.' : 'TX binding phrase update failed.',
      ok ? 'success' : 'error'
    );
  };

  const handleEnterProxyClick = () => setProxyConfirmOpen(true);
  const handleProxyConfirmCancel = () => setProxyConfirmOpen(false);
  const handleProxyConfirmOk = async () => {
    setProxyBusy(true);
    try {
      const ok = await serial.enterUsbUartProxy();
      if (ok) {
        showStatus('Proxy mode enabled. ELRS Buddy will open below—click Connect to configure your TX.', 'success');
        setProxyConfirmOpen(false);
      } else {
        showStatus('Failed to enter proxy mode.', 'error');
      }
    } finally {
      setProxyBusy(false);
    }
  };

  const handleViewElrsPhrase = async () => {
    setElrsPhraseView(null);
    const link = await serial.getLinkStatus();
    if (link?.txType !== 2) {
      showStatus('TX is not ELRS. View phrase only works when RC is connected to an ELRS module.', 'error');
      return;
    }
    if (link?.elrsRole === 'rx') {
      showStatus('Connected device is an ELRS receiver (wrong mode). Use an ELRS transmitter module.', 'error');
      return;
    }
    showStatus('Reading phrase from ELRS…', 'idle');
    const phrase = await serial.getElrsBindingPhrase();
    if (phrase !== null) {
      setElrsPhraseView(phrase);
      showStatus('Phrase read from ELRS.', 'success');
    } else {
      showStatus('Could not read phrase from ELRS (timeout or not supported).', 'error');
    }
  };

  if (!serial.connected) {
    return <p className="hint">Connect USB first.</p>;
  }

  return (
    <div className="panel-content">
      <p className="text-[var(--color-text-muted)] text-sm mb-4">
        Apply = write draft to runtime. Save = write runtime to EEPROM (persists).
      </p>

      <div className="flex flex-wrap gap-6">
        <div>
          <h4 className="text-sm font-semibold text-[var(--color-text)] mb-2">Device</h4>
          <div className="btn-row flex flex-wrap gap-2">
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
        </div>
        <div>
          <h4 className="text-sm font-semibold text-[var(--color-text)] mb-2">File</h4>
          <div className="btn-row flex flex-wrap gap-2">
            <button type="button" className="btn-secondary" onClick={handleExport}>
              Export JSON
            </button>
            <button type="button" className="btn-secondary" onClick={handleImport}>
              Import JSON
            </button>
          </div>
        </div>
      </div>

      <input
        ref={fileInputRef}
        type="file"
        accept=".json"
        hidden
        onChange={onFileChange}
      />

      <div className="binding-phrase-card mt-6">
        <h4>Binding phrase</h4>
        <p className="hint text-sm mt-1">
          Set RX first, then TX, then run pairing again. Max 32 bytes.
        </p>
        <div className="binding-phrase-row">
          <input
            className="binding-phrase-input"
            type="text"
            value={bindingPhrase}
            onChange={(e) => setBindingPhrase(e.target.value)}
            placeholder="Enter new binding phrase"
          />
          <button type="button" className="btn-secondary" onClick={handleSetRxBindingPhrase}>
            Set RX phrase
          </button>
          <button type="button" className="btn-secondary" onClick={handleSetTxBindingPhrase}>
            Set TX phrase
          </button>
        </div>
        <p className="hint text-sm mt-2">
          When TX is ELRS, phrase is set on the ELRS module via CRSF.
        </p>
        <div className="binding-phrase-row mt-2">
          <button type="button" className="btn-secondary" onClick={handleViewElrsPhrase}>
            View phrase (from ELRS)
          </button>
          {elrsPhraseView !== null && (
            <span className="font-mono text-sm ml-2" title="Current phrase on ELRS TX">
              {elrsPhraseView || '(empty)'}
            </span>
          )}
        </div>
      </div>

      <div className="card mt-6 border-[var(--color-warning)]">
        <h4 className="text-[var(--color-text)] font-semibold">USB-UART Proxy</h4>
        <p className="hint text-sm mt-1">
          Forward raw bytes between USB and the TX module. Use ELRS Buddy or similar to configure your TX directly. Reset RC to exit.
        </p>
        <div className="mt-3">
          <button
            type="button"
            className="btn-secondary"
            onClick={handleEnterProxyClick}
            disabled={proxyBusy}
          >
            {proxyBusy ? 'Entering…' : 'Enter USB-UART Proxy Mode'}
          </button>
        </div>
      </div>

      {proxyConfirmOpen && (
        <div
          className="fixed inset-0 z-50 flex items-center justify-center bg-black/50"
          role="dialog"
          aria-modal="true"
          aria-labelledby="proxy-confirm-title"
        >
          <div className="bg-[var(--color-card)] rounded-lg p-4 max-w-md mx-4 shadow-lg border border-[var(--color-border)]">
            <h3 id="proxy-confirm-title" className="font-semibold text-[var(--color-text)] mb-2">
              Enter USB-UART Proxy Mode?
            </h3>
            <p className="text-sm text-[var(--color-text-muted)] mb-4">{PROXY_CONFIRM_MSG}</p>
            <div className="flex justify-end gap-2">
              <button type="button" className="btn-secondary" onClick={handleProxyConfirmCancel} disabled={proxyBusy}>
                Cancel
              </button>
              <button type="button" className="btn-primary" onClick={handleProxyConfirmOk} disabled={proxyBusy}>
                {proxyBusy ? 'Entering…' : 'Enter Proxy Mode'}
              </button>
            </div>
          </div>
        </div>
      )}

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
