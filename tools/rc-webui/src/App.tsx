import { useState, useEffect } from 'react';
import { SerialProvider, useSerialContext } from './context/SerialContext';
import { LivePanel } from './components/LivePanel';
import { CalibratePanel } from './components/CalibratePanel';
import { MappingPanel } from './components/MappingPanel';
import { CurvesPanel } from './components/CurvesPanel';
import { ConfigPanel } from './components/ConfigPanel';
import { AutoPairPanel } from './components/AutoPairPanel';
import { getPortLabel } from './hooks/useSerial';
import * as Tabs from '@radix-ui/react-tabs';
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

  if (serial.proxyModeActive) {
    return (
      <div className="app w-full min-h-screen flex flex-col">
        <div className="rounded-[var(--radius-card)] border border-[var(--color-border)] bg-[var(--color-card)] m-4 p-4 shrink-0">
          <p className="text-sm text-[var(--color-text)]">
            <strong>USB-UART Proxy mode active.</strong> RC is forwarding raw bytes. Use ELRS Buddy below to configure your TX. Reset RC to exit proxy mode.
            {' '}
            <a href="https://fourflies.mooo.com/elrsbuddy/" target="_blank" rel="noopener noreferrer" className="text-[var(--color-primary)] underline">
              Open ELRS Buddy in new tab
            </a>
          </p>
        </div>
        <iframe
          src="https://fourflies.mooo.com/elrsbuddy/"
          title="ELRS Buddy"
          className="flex-1 w-full min-h-[500px] border border-[var(--color-border)] rounded-[var(--radius-card)] mx-4 mb-4"
          sandbox="allow-scripts allow-same-origin allow-forms"
          allow="serial"
        />
      </div>
    );
  }

  return (
    <div className="app max-w-[920px] mx-auto px-5 py-6 w-full">
      <header className="mb-6">
        <h1 className="font-heading text-xl font-semibold tracking-tight m-0 mb-3 text-[var(--color-text)]">
          RC Config
        </h1>
        <div
          className="rounded-[var(--radius-card)] border border-[var(--color-border)] bg-[var(--color-card)]"
          style={{ padding: '1rem 1.25rem' }}
        >
          <div className="flex flex-col sm:flex-row sm:flex-wrap items-stretch sm:items-center gap-3">
            {serial.connected ? (
              <>
                <div className="flex items-center gap-2">
                  <span
                    className="w-2 h-2 rounded-full bg-[var(--color-success)] shrink-0"
                    style={{ boxShadow: '0 0 0 2px var(--color-success-glow)' }}
                    aria-hidden
                  />
                  <span className="text-[0.95rem] text-[var(--color-text)]">
                    {serial.portName ?? 'Connected'}
                    {serial.deviceInfo && (
                      <span className="text-[var(--color-text-muted)] font-normal">
                        {' '}
                        — {serial.deviceInfo.name} {serial.deviceInfo.version}
                      </span>
                    )}
                  </span>
                </div>
                <button
                  type="button"
                  className="btn-secondary"
                  onClick={() => void serial.disconnect()}
                >
                  Disconnect
                </button>
              </>
            ) : (
              <>
                <label htmlFor="port-select" className="sr-only">
                  Device
                </label>
                <select
                  id="port-select"
                  className="input-select w-full sm:min-w-[180px] sm:w-auto"
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
                <button type="button" className="btn-primary" onClick={serial.connectNew}>
                  Add new device
                </button>
                <span className="text-[0.95rem] text-[var(--color-text-muted)]">Not connected</span>
              </>
            )}
          </div>
          <p className="mt-3 text-[0.85rem] text-[var(--color-text-muted)] leading-snug">
            {serial.connected
              ? 'Connected over USB serial. Use the tabs below to calibrate, map axes, and save config.'
              : 'Choose a device you’ve used before, or “Add new device” to pick from the system list. Requires Chrome or Edge.'}
          </p>
          {serial.error && (
            <p className="mt-3 text-[0.9rem] text-[var(--color-error)] w-full">{serial.error}</p>
          )}
        </div>
      </header>

      <Tabs.Root className="w-full" value={tab} onValueChange={(value) => setTab(value)}>
        <Tabs.List
          className="flex gap-0.5 mb-5 pb-0 border-b border-[var(--color-border)] overflow-x-auto flex-nowrap"
          role="tablist"
          aria-label="Configuration sections"
        >
          {TABS.map((t) => (
            <Tabs.Trigger
              key={t.id}
              value={t.id}
              className="tabs-trigger"
            >
              {t.label}
            </Tabs.Trigger>
          ))}
        </Tabs.List>

        {TABS.map((t) => (
          <Tabs.Content
            key={t.id}
            value={t.id}
            className="min-h-[220px] py-1 focus:outline-none"
            role="tabpanel"
          >
            <t.Panel />
          </Tabs.Content>
        ))}
      </Tabs.Root>
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
