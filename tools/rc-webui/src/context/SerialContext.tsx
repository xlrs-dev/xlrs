import { createContext, useContext, useMemo, useState, useCallback } from 'react';
import { useSerial } from '../hooks/useSerial';
import type { RcConfig, StateFrame } from '../types/rc';
import { defaultRcConfig } from '../types/rc';

interface SerialContextValue {
  serial: ReturnType<typeof useSerial>;
  configDraft: RcConfig;
  setConfigDraft: (c: RcConfig | ((prev: RcConfig) => RcConfig)) => void;
  liveState: StateFrame | null;
  setLiveState: (s: StateFrame | null) => void;
  loadConfigFromDevice: () => Promise<void>;
}

const SerialContext = createContext<SerialContextValue | null>(null);

export function SerialProvider({ children }: { children: React.ReactNode }) {
  const serial = useSerial();
  const [configDraft, setConfigDraftState] = useState<RcConfig>(defaultRcConfig);
  const [liveState, setLiveState] = useState<StateFrame | null>(null);

  const setConfigDraft = useCallback((c: RcConfig | ((prev: RcConfig) => RcConfig)) => {
    setConfigDraftState((prev) => (typeof c === 'function' ? c(prev) : c));
  }, []);

  const loadConfigFromDevice = useCallback(async () => {
    const cfg = await serial.getConfig();
    if (cfg) setConfigDraftState(cfg);
  }, [serial.getConfig, setConfigDraftState]);

  const value = useMemo<SerialContextValue>(
    () => ({
      serial,
      configDraft,
      setConfigDraft,
      liveState,
      setLiveState,
      loadConfigFromDevice,
    }),
    [serial, configDraft, setConfigDraft, liveState, loadConfigFromDevice]
  );

  return <SerialContext.Provider value={value}>{children}</SerialContext.Provider>;
}

export function useSerialContext(): SerialContextValue {
  const ctx = useContext(SerialContext);
  if (!ctx) throw new Error('useSerialContext must be used within SerialProvider');
  return ctx;
}
