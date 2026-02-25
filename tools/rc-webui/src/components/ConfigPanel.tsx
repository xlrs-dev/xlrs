import { useState, useRef } from 'react';
import { useSerialContext } from '../context/SerialContext';
import { defaultRcConfig } from '../types/rc';

export function ConfigPanel() {
  const { serial, configDraft, setConfigDraft, loadConfigFromDevice } = useSerialContext();
  const [status, setStatus] = useState('');
  const [bindingPhrase, setBindingPhrase] = useState('');
  const fileInputRef = useRef<HTMLInputElement>(null);

  const handleApply = async () => {
    setStatus('Applying…');
    const ok = await serial.setConfigDraft(configDraft);
    if (ok) {
      const applied = await serial.applyConfig();
      setStatus(applied ? 'Applied to device.' : 'Apply failed.');
    } else {
      setStatus('Set draft failed.');
    }
  };

  const handleSave = async () => {
    setStatus('Saving to EEPROM…');
    const ok = await serial.saveConfig();
    setStatus(ok ? 'Saved to EEPROM.' : 'Save failed.');
  };

  const handleLoad = async () => {
    setStatus('Loading…');
    await loadConfigFromDevice();
    setStatus('Config loaded from device.');
  };

  const handleExport = () => {
    const blob = new Blob([JSON.stringify(configDraft, null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'rc-config.json';
    a.click();
    URL.revokeObjectURL(a.href);
    setStatus('Exported.');
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
        setStatus('Imported. Apply to device to use.');
      } catch {
        setStatus('Invalid JSON.');
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
      setStatus(err);
      return;
    }
    setStatus('Sending RX binding phrase update...');
    const ok = await serial.setRxBindingPhrase(bindingPhrase);
    setStatus(ok ? 'RX binding phrase update sent.' : 'RX binding phrase update failed.');
  };

  const handleSetTxBindingPhrase = async () => {
    const err = validateBindingPhrase();
    if (err) {
      setStatus(err);
      return;
    }
    setStatus('Sending TX binding phrase update...');
    const ok = await serial.setTxBindingPhrase(bindingPhrase);
    setStatus(ok ? 'TX binding phrase updated.' : 'TX binding phrase update failed.');
  };

  if (!serial.connected) {
    return <p className="hint">Connect USB first.</p>;
  }

  return (
    <div className="panel-content">
      <p>Apply = write draft to runtime. Save = write runtime to EEPROM (persists).</p>
      <div className="btn-row">
        <button type="button" className="btn" onClick={handleLoad}>
          Load from device
        </button>
        <button type="button" className="btn" onClick={handleApply}>
          Apply to device
        </button>
        <button type="button" className="btn primary" onClick={handleSave}>
          Save to EEPROM
        </button>
        <button type="button" className="btn" onClick={handleExport}>
          Export JSON
        </button>
        <button type="button" className="btn" onClick={handleImport}>
          Import JSON
        </button>
      </div>
      <input
        ref={fileInputRef}
        type="file"
        accept=".json"
        hidden
        onChange={onFileChange}
      />
      <div className="binding-phrase-card">
        <h4>Binding phrase</h4>
        <p className="hint">
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
          <button type="button" className="btn" onClick={handleSetRxBindingPhrase}>
            Set RX phrase
          </button>
          <button type="button" className="btn" onClick={handleSetTxBindingPhrase}>
            Set TX phrase
          </button>
        </div>
      </div>
      <p className="status">{status}</p>
    </div>
  );
}
