import { useState, useEffect } from 'react';
import { SerialProvider, useSerialContext } from './context/SerialContext';
import { LivePanel } from './components/LivePanel';
import { CalibratePanel } from './components/CalibratePanel';
import { MappingPanel } from './components/MappingPanel';
import { CurvesPanel } from './components/CurvesPanel';
import { ConfigPanel } from './components/ConfigPanel';
import { AutoPairPanel } from './components/AutoPairPanel';
import { getPortLabel } from './hooks/useSerial';
import './App.css';

const TABS = [
  { id: 'live', label: 'Live', Panel: LivePanel },
  { id: 'calibrate', label: 'Calibrate', Panel: CalibratePanel },
  { id: 'mapping', label: 'Mapping', Panel: MappingPanel },
  { id: 'curves', label: 'Curves', Panel: CurvesPanel },
  { id: 'config', label: 'Save / Apply', Panel: ConfigPanel },
  { id: 'autopair', label: 'Auto Pair', Panel: AutoPairPanel },
] as const;

function AppContent() {
  const [tab, setTab] = useState<string>('live');
  const { serial } = useSerialContext();
  const current = TABS.find((t) => t.id === tab) ?? TABS[0];
  const Panel = current.Panel;

  useEffect(() => {
    if (navigator.serial) serial.refreshPorts();
  }, [serial.refreshPorts]);

  const handlePortChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const value = e.target.value;
    if (value === '') return;
    if (value === 'new') {
      serial.connectNew();
      e.target.value = '';
      return;
    }
    const i = Number(value);
    if (!Number.isNaN(i) && serial.availablePorts[i]) {
      serial.connectToPort(serial.availablePorts[i]!);
      e.target.value = '';
    }
  };

  return (
    <div className="app">
      <header className="header">
        <h1>RC Config</h1>
        <div className="connection-card">
          <div className="serial-row">
            {serial.connected ? (
              <>
                <div className="connection-status">
                  <span className="status-dot connected" aria-hidden />
                  <span className="status">
                    {serial.portName ?? 'Connected'}
                    {serial.deviceInfo && (
                      <span className="device-info"> — {serial.deviceInfo.name} {serial.deviceInfo.version}</span>
                    )}
                  </span>
                </div>
                <button type="button" className="btn" onClick={serial.disconnect}>
                  Disconnect
                </button>
              </>
            ) : (
              <>
                <label htmlFor="port-select" className="sr-only">Device</label>
                <select
                  id="port-select"
                  className="port-select"
                  value=""
                  onChange={handlePortChange}
                  aria-label="Select USB serial device"
                >
                  <option value="">Select device…</option>
                  {serial.availablePorts.map((port, i) => (
                    <option key={i} value={i}>
                      {getPortLabel(port)}
                    </option>
                  ))}
                  <option value="new">Add new device…</option>
                </select>
                <button type="button" className="btn primary" onClick={serial.connectNew}>
                  Add new device
                </button>
                <span className="status muted">Not connected</span>
              </>
            )}
          </div>
          <p className="connection-hint">
            {serial.connected
              ? 'Connected over USB serial. Use the tabs below to calibrate, map axes, and save config.'
              : 'Choose a device you’ve used before, or “Add new device” to pick from the system list. Requires Chrome or Edge.'}
          </p>
          {serial.error && <p className="error">{serial.error}</p>}
        </div>
      </header>

      <main className="main">
        <nav className="tabs" role="tablist" aria-label="Configuration sections">
          {TABS.map((t) => (
            <button
              key={t.id}
              type="button"
              role="tab"
              aria-selected={tab === t.id}
              className={`tab ${tab === t.id ? 'active' : ''}`}
              onClick={() => setTab(t.id)}
            >
              {t.label}
            </button>
          ))}
        </nav>

        <section className="panel" role="tabpanel">
          <Panel />
        </section>
      </main>
    </div>
  );
}

function App() {
  return (
    <SerialProvider>
      <AppContent />
    </SerialProvider>
  );
}

export default App;
