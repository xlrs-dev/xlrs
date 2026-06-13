import "./style.css";
import { parseRcLine, RcMessage, RcSerialTransport, TransportStatus } from "./rcTransport";

type DeviceKey = "rc" | "rx";
type BindingTarget = "tx" | "rx";
type StatusLevel = "info" | "ok" | "warn" | "bad";

interface RcConfig {
  version: number;
  calMin: number[];
  calCenter: number[];
  calMax: number[];
  invert: number[];
  deadzone: number[];
  functions: number[];
  trim: number[];
  cutoffMin: number[];
  cutoffMax: number[];
  filter: FilterConfig;
}

interface FilterConfig {
  oversample: number;
  lowPass: number;
  temporalBlend: boolean;
  highPass: number;
}

interface LiveState {
  adc: number[];
  adcFiltered: number[];
  ch: number[];
  toggles: number[];
  lq: string;
  rssi: string;
  snr: string;
  txBattMv: string;
  rxBattMv: string;
  txPresent: boolean;
  safetyHold: boolean;
  haveChannels: boolean;
}

interface BindingInfo {
  target: string;
  uid: string;
  uidCheck: string;
  persisted: boolean;
  requiresReboot: boolean;
  phrase: string;
}

interface DeviceSession {
  key: DeviceKey;
  label: string;
  transport: RcSerialTransport;
  status: TransportStatus;
  statusDetail: string;
  info: Record<string, string>;
  caps: Set<string>;
  txInfo: Record<string, string>;
  config: RcConfig;
  state: LiveState | null;
  binding: BindingInfo | null;
  streamEnabled: boolean;
  pollTimer: number;
  polling: boolean;
  dirty: boolean;
  dirtyFields: Set<string>;
  logLines: string[];
}

interface UiState {
  activeScreen: "calibration" | "channels" | "filters" | "binding";
  phrase: string;
  statusText: string;
  statusLevel: StatusLevel;
  rc: DeviceSession;
  rx: DeviceSession;
}

const AXES = ["Aileron", "Elevator", "Rudder", "Throttle"];
const CHANNELS = ["CH1", "CH2", "CH3", "CH4", "CH5", "CH6", "CH7", "CH8"];
const FUNCTION_OPTIONS = ["Aileron", "Elevator", "Rudder", "Throttle"];
const MAX_LOG_LINES = 80;
const CRSF_RAW_MIN = 172;
const CRSF_RAW_MID = 992;
const CRSF_RAW_MAX = 1811;

const defaultConfig: RcConfig = {
  version: 0,
  calMin: [0, 0, 0, 0],
  calCenter: [2048, 2048, 2048, 2048],
  calMax: [4095, 4095, 4095, 4095],
  invert: [0, 0, 0, 0],
  deadzone: [8, 8, 8, 8],
  functions: [0, 1, 2, 3],
  trim: [0, 0, 0, 0, 0, 0, 0, 0],
  cutoffMin: [172, 172, 172, 172, 172, 172, 172, 172],
  cutoffMax: [1811, 1811, 1811, 1811, 1811, 1811, 1811, 1811],
  filter: {
    oversample: 4,
    lowPass: 0,
    temporalBlend: false,
    highPass: 0,
  },
};

const ui: UiState = {
  activeScreen: "calibration",
  phrase: "",
  statusText: "Connect RC USB for handset config/calibration and TX binding, or RX USB for direct RX binding.",
  statusLevel: "info",
  rc: createSession("rc", "RC USB"),
  rx: createSession("rx", "RX USB"),
};

const appRoot = document.querySelector<HTMLDivElement>("#app")!;
let renderTimer = 0;
let liveStateFrame = 0;
const liveStateDirty = new Set<DeviceKey>();

if (!navigator.serial) {
  ui.statusText = "Use Chrome or Edge over localhost/HTTPS to enable Web Serial.";
  ui.statusLevel = "bad";
}

document.addEventListener("click", (event) => {
  const target = (event.target as Element).closest<HTMLElement>("[data-action]");
  if (!target) return;
  void handleAction(target).catch((error: unknown) => {
    setStatus(errorToString(error), "bad");
    render();
  });
});

document.addEventListener("input", (event) => {
  const target = event.target as HTMLInputElement | HTMLSelectElement;
  if (target.matches("[data-binding-phrase]")) {
    ui.phrase = target.value;
    updateBindingButtonState();
    return;
  }

  const configPath = target.dataset.configPath;
  if (configPath) {
    updateConfig(configPath, readInputValue(target));
    syncConfigControls(configPath, target);
    return;
  }

  const filterKey = target.dataset.filterKey as keyof FilterConfig | undefined;
  if (filterKey) {
    updateFilter(filterKey, readInputValue(target));
    syncMirroredControls(`[data-filter-key="${cssEscape(filterKey)}"]`, target);
  }
});

document.addEventListener("submit", (event) => {
  const form = (event.target as Element).closest<HTMLFormElement>("[data-send-device]");
  if (!form) return;
  event.preventDefault();
  const session = ui[form.dataset.sendDevice as DeviceKey];
  void sendRaw(form, session).catch((error: unknown) => {
    setStatus(errorToString(error), "bad");
    render();
  });
});

document.addEventListener("change", (event) => {
  const target = event.target as HTMLInputElement | HTMLSelectElement;

  const configPath = target.dataset.configPath;
  if (configPath) {
    updateConfig(configPath, readInputValue(target));
    syncConfigControls(configPath, target);
    void persistConfigPath(configPath).catch((error: unknown) => {
      setStatus(errorToString(error), "bad");
      scheduleRender();
    });
    return;
  }

  const filterKey = target.dataset.filterKey as keyof FilterConfig | undefined;
  if (filterKey) {
    updateFilter(filterKey, readInputValue(target));
    syncMirroredControls(`[data-filter-key="${cssEscape(filterKey)}"]`, target);
    void persistFilterChange(filterKey).catch((error: unknown) => {
      setStatus(errorToString(error), "bad");
      scheduleRender();
    });
  }
});

render();

function createSession(key: DeviceKey, label: string): DeviceSession {
  const transport = new RcSerialTransport(115200);
  const session: DeviceSession = {
    key,
    label,
    transport,
    status: "idle",
    statusDetail: "",
    info: {},
    caps: new Set(),
    txInfo: {},
    config: cloneConfig(defaultConfig),
    state: null,
    binding: null,
    streamEnabled: false,
    pollTimer: 0,
    polling: false,
    dirty: false,
    dirtyFields: new Set(),
    logLines: [],
  };

  transport.on("status", ({ status, detail }) => {
    session.status = status;
    session.statusDetail = detail ?? "";
    if (status === "connected") setStatus(`${session.label} connected. Run Discover to refresh role/caps.`, "ok");
    if (status === "reconnecting") setStatus(`${session.label} disconnected; reconnecting.`, "warn");
    if (status === "disconnected") setStatus(`${session.label} disconnected.`, "warn");
    render();
  });
  transport.on("write", ({ line }) => {
    appendLog(session, `> ${line}`);
  });
  transport.on("line", ({ line }) => {
    if (isLiveStateMessage(parseRcLine(line))) return;
    appendLog(session, line);
  });
  transport.on("message", ({ message }) => {
    if (isLiveStateMessage(message)) {
      if (message.kind === "state") session.state = normalizeState(message.fields);
      else mergeLiveStateFields(session, message.fields);
      queueLiveStateUpdate(session);
      return;
    }
    applyMessage(session, message);
    render();
  });
  transport.on("error", ({ error }) => {
    appendLog(session, `! ${errorToString(error)}`);
  });

  return session;
}

function render(): void {
  if (renderTimer !== 0) {
    window.clearTimeout(renderTimer);
    renderTimer = 0;
  }
  appRoot.innerHTML = `
    <main class="app-shell">
      <section class="hero panel">
        <div>
          <p class="eyebrow">XLRS tools</p>
          <h1>RC Web Serial UI</h1>
          <p class="lede">Calibration, config, filters, and binding workflows for the <code>rc.v1</code> protocol.</p>
        </div>
        <span class="tag ${navigator.serial ? "ok" : "bad"}">${navigator.serial ? "Web Serial ready" : "Web Serial unavailable"}</span>
      </section>

      <section class="device-grid">
        ${connectionPanel(ui.rc)}
        ${connectionPanel(ui.rx)}
      </section>

      <section class="status-banner ${ui.statusLevel}">${escapeHtml(ui.statusText)}</section>

      <nav class="tabs" aria-label="WebUI screens">
        ${tabButton("calibration", "Calibration Wizard")}
        ${tabButton("channels", "Channels")}
        ${tabButton("filters", "Filters")}
        ${tabButton("binding", "Binding Wizard")}
      </nav>

      ${activeScreen()}

      <section class="device-grid">
        ${terminalPanel(ui.rc)}
        ${terminalPanel(ui.rx)}
      </section>
    </main>
  `;
}

function scheduleRender(): void {
  if (renderTimer !== 0) return;
  renderTimer = window.setTimeout(() => {
    renderTimer = 0;
    if (isEditingElement(document.activeElement)) {
      scheduleRender();
      return;
    }
    render();
  }, 500);
}

function isEditingElement(element: Element | null): boolean {
  return element instanceof HTMLInputElement ||
    element instanceof HTMLTextAreaElement ||
    element instanceof HTMLSelectElement;
}

function connectionPanel(session: DeviceSession): string {
  const connected = isConnected(session);
  const caps = session.caps.size ? Array.from(session.caps).sort().join(", ") : "none";
  return `
    <article class="panel connection-panel">
      <div class="panel-title">
        <h2>${session.label}</h2>
        <span class="tag ${connected ? "ok" : ""}">${session.status}</span>
      </div>
      <dl class="summary-list">
        <div><dt>Role</dt><dd>${escapeHtml(session.info.role ?? "unknown")}</dd></div>
        <div><dt>FW</dt><dd>${escapeHtml(session.info.fw ?? "-")}</dd></div>
        <div><dt>Caps</dt><dd>${escapeHtml(caps)}</dd></div>
      </dl>
      <p class="muted">${escapeHtml(session.statusDetail)}</p>
      <div class="actions">
        <button data-action="connect" data-device="${session.key}" ${disabled(!navigator.serial)}>Connect</button>
        <button class="secondary" data-action="discover" data-device="${session.key}" ${disabled(!connected)}>Discover</button>
        <button class="secondary" data-action="stream" data-device="${session.key}" ${disabled(!connected)}>${session.streamEnabled ? "Stop stream" : "Stream state"}</button>
        <button class="secondary" data-action="disconnect" data-device="${session.key}" ${disabled(!connected)}>Disconnect</button>
      </div>
      ${liveState(session)}
    </article>
  `;
}

function liveState(session: DeviceSession): string {
  const state = session.state;
  return `
    <div class="live-grid" data-live-state="${session.key}">
      <div><span>ADC</span><strong data-live-field="adc">${escapeHtml(state?.adc.join(", ") || "-")}</strong></div>
      <div><span>Filtered</span><strong data-live-field="adc_filtered">${escapeHtml(state?.adcFiltered.join(", ") || "-")}</strong></div>
      <div><span>Channels</span><strong data-live-field="ch">${escapeHtml(state?.ch.join(", ") || "-")}</strong></div>
      <div><span>Toggles</span><strong data-live-field="toggles">${escapeHtml(state?.toggles.join(", ") || "-")}</strong></div>
      <div><span>Safety</span><strong data-live-field="safety">${state ? liveSafetyText(state) : "-"}</strong></div>
      <div><span>Link</span><strong data-live-field="link">${state ? liveLinkText(state) : "-"}</strong></div>
      <div><span>Battery</span><strong data-live-field="battery">${state ? liveBatteryText(state) : "-"}</strong></div>
    </div>
  `;
}

function queueLiveStateUpdate(session: DeviceSession): void {
  liveStateDirty.add(session.key);
  if (liveStateFrame !== 0) return;
  liveStateFrame = window.requestAnimationFrame(() => {
    liveStateFrame = 0;
    const sessions = Array.from(liveStateDirty).map((key) => ui[key]);
    liveStateDirty.clear();
    sessions.forEach(updateLiveStatePanel);
  });
}

function updateLiveStatePanel(session: DeviceSession): void {
  if (!session.state) return;
  const root = document.querySelector<HTMLElement>(`[data-live-state="${session.key}"]`);
  if (root) {
    setLiveField(root, "adc", session.state.adc.join(", ") || "-");
    setLiveField(root, "adc_filtered", session.state.adcFiltered.join(", ") || "-");
    setLiveField(root, "ch", session.state.ch.join(", ") || "-");
    setLiveField(root, "toggles", session.state.toggles.join(", ") || "-");
    setLiveField(root, "safety", liveSafetyText(session.state));
    setLiveField(root, "link", `LQ ${session.state.lq} RSSI ${session.state.rssi} SNR ${session.state.snr}`);
    setLiveField(root, "battery", `TX ${mv(session.state.txBattMv)} RX ${mv(session.state.rxBattMv)}`);
  }
  updateLiveTables(session);
  updateStickMonitors(session);
}

function setLiveField(root: HTMLElement, field: string, value: string): void {
  const element = root.querySelector<HTMLElement>(`[data-live-field="${field}"]`);
  if (element) element.textContent = value;
}

function liveLinkText(state: LiveState): string {
  return escapeHtml(`LQ ${state.lq} RSSI ${state.rssi} SNR ${state.snr}`);
}

function liveBatteryText(state: LiveState): string {
  return escapeHtml(`TX ${mv(state.txBattMv)} RX ${mv(state.rxBattMv)}`);
}

function liveSafetyText(state: LiveState): string {
  if (state.safetyHold) return "hold active";
  return state.haveChannels ? "live" : "waiting";
}

function stickSafetyText(state: LiveState): string {
  if (state.safetyHold) return "Safety hold active: preview moves, CRSF output stays disarmed";
  return state.haveChannels ? "Mapped channels live" : "Waiting for live channels";
}

function terminalPanel(session: DeviceSession): string {
  return `
    <section class="panel terminal-panel">
      <div class="panel-title">
        <h2>${session.label} terminal</h2>
        <button class="secondary" data-action="clear-log" data-device="${session.key}">Clear</button>
      </div>
      <pre class="terminal" data-terminal="${session.key}">${escapeHtml(session.logLines.join("\n"))}</pre>
      <form class="send-row" data-send-device="${session.key}">
        <input type="text" name="line" autocomplete="off" placeholder="Raw command, e.g. rc.v1 hello seq=1" ${disabled(!isConnected(session))} />
        <button type="button" class="secondary" data-action="send-raw" data-device="${session.key}" ${disabled(!isConnected(session))}>Send</button>
      </form>
    </section>
  `;
}

function activeScreen(): string {
  if (ui.activeScreen === "channels") return channelsScreen();
  if (ui.activeScreen === "filters") return filtersScreen();
  if (ui.activeScreen === "binding") return bindingScreen();
  return calibrationScreen();
}

function calibrationScreen(): string {
  const config = ui.rc.config;
  return `
    <section class="panel screen-panel">
      <div class="panel-title split">
        <div>
          <h2>Calibration Wizard</h2>
          <p class="muted">Center, sample full stick travel, then finish. Manual edits persist immediately.</p>
        </div>
        <button class="secondary" data-action="load-config" ${disabled(!isConnected(ui.rc))}>Load current config</button>
      </div>
      <ol class="wizard">
        <li>
          <strong>1. Start</strong>
          <span>Center sticks and initialize firmware min/max from current ADC.</span>
          <button data-action="cal-start" ${disabled(!isConnected(ui.rc))}>Start</button>
        </li>
        <li>
          <strong>2. Sample</strong>
          <span>Move every axis through full travel. Press sample repeatedly while moving sticks.</span>
          <button data-action="cal-sample" ${disabled(!isConnected(ui.rc))}>Sample</button>
        </li>
        <li>
          <strong>3. Finish</strong>
          <span>Return sticks to center, then finish and save calibration.</span>
          <span class="inline-actions">
            <button data-action="cal-finish" ${disabled(!isConnected(ui.rc))}>Finish calibration</button>
          </span>
        </li>
      </ol>
      ${stickMonitor(ui.rc)}
      ${filterTuningPanel("Calibration filter tuning")}
      <div class="table-wrap">
        <table>
          <thead><tr><th>Axis</th><th>Min</th><th>Center</th><th>Max</th><th>Raw ADC</th><th>Filtered ADC</th><th>Output</th></tr></thead>
          <tbody>
            ${AXES.map((axis, index) => `
              <tr>
                <td>${axis}</td>
                <td>${calibrationControl(`cal.min.${index}`, config.calMin[index] ?? 0)}</td>
                <td>${calibrationControl(`cal.center.${index}`, config.calCenter[index] ?? 2048)}</td>
                <td>${calibrationControl(`cal.max.${index}`, config.calMax[index] ?? 4095)}</td>
                <td data-live-adc-index="${index}">${ui.rc.state?.adc[index] ?? "-"}</td>
                <td data-live-adc-filtered-index="${index}">${ui.rc.state?.adcFiltered[index] ?? "-"}</td>
                <td data-live-ch-index="${index}">${ui.rc.state?.ch[index] ?? "-"}</td>
              </tr>
            `).join("")}
          </tbody>
        </table>
      </div>
    </section>
  `;
}

function channelsScreen(): string {
  const config = ui.rc.config;
  return `
    <section class="panel screen-panel">
      <div class="panel-title split">
        <div>
          <h2>Channel Mapping, Invert, Deadzone, Trim, Cutoffs</h2>
          <p class="muted">Changes apply to runtime output immediately. Use Save after verification to persist to EEPROM.</p>
        </div>
        <button class="secondary" data-action="load-config" ${disabled(!isConnected(ui.rc))}>Reload</button>
      </div>

      ${stickMonitor(ui.rc)}

      <h3>Axis setup</h3>
      <div class="table-wrap">
        <table>
          <thead><tr><th>Axis</th><th>Function</th><th>Invert</th><th>Deadzone</th><th>Live ADC</th></tr></thead>
          <tbody>${AXES.map((axis, index) => axisRow(axis, index, config)).join("")}</tbody>
        </table>
      </div>

      <h3>Channel trims and cutoffs</h3>
      <div class="table-wrap">
        <table>
          <thead><tr><th>Channel</th><th>Trim</th><th>Cutoff min</th><th>Cutoff max</th><th>Live output</th></tr></thead>
          <tbody>${CHANNELS.map((channel, index) => channelRow(channel, index, config)).join("")}</tbody>
        </table>
      </div>
    </section>
  `;
}

function axisRow(axis: string, index: number, config: RcConfig): string {
  return `
    <tr>
      <td>${axis}</td>
      <td>
        <select data-config-path="axis.function.${index}">
          ${FUNCTION_OPTIONS.map((label, value) => `<option value="${value}" ${selected(config.functions[index] === value)}>${label}</option>`).join("")}
        </select>
      </td>
      <td><input type="checkbox" data-config-path="axis.invert.${index}" ${checked(config.invert[index] === 1)} /></td>
      <td><input type="number" min="0" max="512" data-config-path="axis.deadzone.${index}" value="${config.deadzone[index] ?? 0}" /></td>
      <td data-live-adc-index="${index}">${ui.rc.state?.adc[index] ?? "-"}</td>
    </tr>
  `;
}

function channelRow(channel: string, index: number, config: RcConfig): string {
  return `
    <tr>
      <td>${channel}</td>
      <td><input type="number" min="-500" max="500" data-config-path="channel.trim.${index}" value="${config.trim[index] ?? 0}" /></td>
      <td><input type="number" min="0" max="2048" data-config-path="channel.cutoff_min.${index}" value="${config.cutoffMin[index] ?? 172}" /></td>
      <td><input type="number" min="0" max="2048" data-config-path="channel.cutoff_max.${index}" value="${config.cutoffMax[index] ?? 1811}" /></td>
      <td data-live-ch-index="${index}">${ui.rc.state?.ch[index] ?? "-"}</td>
    </tr>
  `;
}

function stickMonitor(session: DeviceSession): string {
  return `
    <div class="stick-monitor" data-stick-monitor="${session.key}">
      <div class="stick-status ${session.state?.safetyHold ? "warn" : ""}" data-stick-safety>
        ${session.state ? stickSafetyText(session.state) : "Waiting for live channels"}
      </div>
      <div class="stick-pair">
        ${stickPad("left", "Left stick", "Throttle", "Rudder", session.state)}
        ${stickPad("right", "Right stick", "Elevator", "Aileron", session.state)}
      </div>
      <div class="channel-bars">
        ${[0, 1, 2, 3].map((index) => channelBar(index, session.state?.ch[index])).join("")}
      </div>
    </div>
  `;
}

function stickPad(id: "left" | "right", title: string, xLabel: string, yLabel: string, state: LiveState | null): string {
  const position = stickPosition(id, state);
  return `
    <div class="stick-readout">
      <div class="stick-title">
        <strong>${title}</strong>
        <span>${xLabel} / ${yLabel}</span>
      </div>
      <div class="stick-pad">
        <span class="stick-axis horizontal"></span>
        <span class="stick-axis vertical"></span>
        <span class="stick-dot" data-stick-dot="${id}" style="left: ${position.x}%; top: ${position.y}%"></span>
      </div>
    </div>
  `;
}

function channelBar(index: number, value: number | undefined): string {
  const percent = crsfPercent(value, 0);
  return `
    <div class="channel-bar" data-live-bar-index="${index}">
      <span>${CHANNELS[index]}</span>
      <div class="bar-track"><span class="bar-fill" data-live-bar-fill style="width: ${percent}%"></span></div>
      <strong data-live-bar-value>${value ?? "-"}</strong>
    </div>
  `;
}

function filtersScreen(): string {
  return `
    <section class="panel screen-panel">
      <div class="panel-title split">
        <div>
          <h2>Filters and Defaults</h2>
          <p class="muted">Filter changes apply to runtime output immediately. Use Save after verification to persist to EEPROM.</p>
        </div>
        <span class="tag ok">live apply</span>
      </div>

      ${filterTuningPanel("Filter tuning")}

      <div class="danger-zone">
        <h3>Defaults</h3>
        <p class="muted">Sends <code>reset_defaults target=rc_config</code>, persists defaults, then reloads config.</p>
        <button class="danger" data-action="defaults" ${disabled(!isConnected(ui.rc))}>Reset RC config defaults</button>
      </div>
    </section>
  `;
}

function calibrationControl(path: string, value: number): string {
  return `
    <div class="range-control">
      <input type="range" min="0" max="4095" step="1" data-config-path="${path}" value="${value}" />
      <input type="number" min="0" max="4095" data-config-path="${path}" value="${value}" />
    </div>
  `;
}

function filterTuningPanel(title: string): string {
  const filter = ui.rc.config.filter;
  return `
    <div class="filter-tuning">
      <h3>${title}</h3>
      <div class="form-grid">
        <label>Oversample level
          <input type="number" min="1" max="32" data-filter-key="oversample" value="${filter.oversample}" />
        </label>
        <label>Low-pass aggressiveness
          <span class="range-control">
            <input type="range" min="0" max="100" step="1" data-filter-key="lowPass" value="${filter.lowPass}" />
            <input type="number" min="0" max="100" data-filter-key="lowPass" value="${filter.lowPass}" />
          </span>
        </label>
        <label>High-pass aggressiveness
          <span class="range-control">
            <input type="range" min="0" max="100" step="1" data-filter-key="highPass" value="${filter.highPass}" />
            <input type="number" min="0" max="100" data-filter-key="highPass" value="${filter.highPass}" />
          </span>
        </label>
        <label class="check">
          <input type="checkbox" data-filter-key="temporalBlend" ${checked(filter.temporalBlend)} />
          Temporal blend
        </label>
      </div>
    </div>
  `;
}

function bindingScreen(): string {
  const comparison = bindingComparison();
  return `
    <section class="panel screen-panel">
      <div class="panel-title split">
        <div>
          <h2>Binding Wizard</h2>
          <p class="muted">TX is written through RC USB. RX is written through direct RX USB. Matching <code>uid_check</code> verifies the pair.</p>
        </div>
        <span class="tag ${comparison.level}">${comparison.label}</span>
      </div>

      <label class="phrase">Binding phrase
        <input type="password" data-binding-phrase maxlength="32" value="${escapeAttribute(ui.phrase)}" placeholder="1..32 byte phrase" />
      </label>

      <div class="device-grid">
        ${bindingSide("TX via RC USB", ui.rc, "tx")}
        ${bindingSide("RX direct USB", ui.rx, "rx")}
      </div>

      <div class="comparison ${comparison.level}">
        <strong>${comparison.title}</strong>
        <p>${escapeHtml(comparison.message)}</p>
      </div>
    </section>
  `;
}

function bindingSide(title: string, session: DeviceSession, target: BindingTarget): string {
  return `
    <article class="subpanel">
      <h3>${title}</h3>
      ${target === "tx" ? txInfoBlock(session) : ""}
      <dl class="summary-list">
        <div><dt>UID</dt><dd>${escapeHtml(session.binding?.uid || "-")}</dd></div>
        <div><dt>UID check</dt><dd>${escapeHtml(session.binding?.uidCheck || "-")}</dd></div>
        <div><dt>Persisted</dt><dd>${session.binding ? yesNo(session.binding.persisted) : "-"}</dd></div>
        <div><dt>Requires reboot</dt><dd>${session.binding ? yesNo(session.binding.requiresReboot) : "-"}</dd></div>
      </dl>
      <div class="actions">
        ${target === "tx" ? `<button class="secondary" data-action="tx-hello" ${disabled(!isConnected(session))}>Discover TX</button>` : ""}
        <button data-action="binding-get" data-target="${target}" ${disabled(!isConnected(session))}>Read</button>
        <button data-action="binding-set" data-target="${target}" ${disabled(!isConnected(session) || !ui.phrase)}>Set phrase</button>
        <button data-action="binding-verify" data-target="${target}" ${disabled(!isConnected(session) || !ui.phrase)}>Verify</button>
        <button class="danger" data-action="binding-clear" data-target="${target}" ${disabled(!isConnected(session))}>Clear</button>
      </div>
    </article>
  `;
}

function txInfoBlock(session: DeviceSession): string {
  if (!Object.keys(session.txInfo).length) {
    return `<p class="muted">Run <code>tx_hello</code> to discover the attached LoRa TX through RC USB.</p>`;
  }
  const present = session.txInfo.tx_present === "1";
  return `<p class="${present ? "success" : "error"}">Attached TX ${present ? "present" : "missing"}; UID check ${escapeHtml(session.txInfo.uid_check ?? "-")}</p>`;
}

async function handleAction(target: HTMLElement): Promise<void> {
  const action = target.dataset.action ?? "";
  const device = target.dataset.device as DeviceKey | undefined;
  const session = device ? ui[device] : undefined;

  if (action === "screen") {
    ui.activeScreen = target.dataset.screen as UiState["activeScreen"];
    render();
    return;
  }
  if (action === "connect" && session) await connect(session);
  if (action === "disconnect" && session) {
    stopStatePolling(session);
    await session.transport.streamState(250, false).catch(() => undefined);
    await session.transport.disconnect();
  }
  if (action === "discover" && session) await discover(session);
  if (action === "stream" && session) await streamState(session);
  if (action === "clear-log" && session) {
    session.logLines = [];
    render();
  }
  if (action === "send-raw" && session) {
    const form = target.closest<HTMLFormElement>("form");
    if (form) await sendRaw(form, session);
  }
  if (action === "load-config") await loadConfig();
  if (action === "apply-config") await applyConfig();
  if (action === "save-config") await saveConfig();
  if (action === "defaults") await resetDefaults();
  if (action === "cal-start") await calibrationStart();
  if (action === "cal-sample") await calibrationSample();
  if (action === "cal-finish") await calibrationFinish();
  if (action === "tx-hello") await txHello();
  if (action === "binding-get") await bindingCommand(target.dataset.target as BindingTarget, "get");
  if (action === "binding-set") await bindingCommand(target.dataset.target as BindingTarget, "set");
  if (action === "binding-verify") await bindingCommand(target.dataset.target as BindingTarget, "verify");
  if (action === "binding-clear") await bindingCommand(target.dataset.target as BindingTarget, "clear");
}

async function connect(session: DeviceSession): Promise<void> {
  if (isConnected(session)) await session.transport.disconnect();
  await session.transport.connect({ useRememberedPort: false });
  await session.transport.streamState(250, false).catch(() => undefined);
  await discover(session);
}

async function discover(session: DeviceSession): Promise<void> {
  const response = await session.transport.hello();
  session.info = { ...response.fields };
  collectCaps(session, response.fields.caps);
  setStatus(`${session.label} discovered as ${response.fields.role ?? "unknown"}.`, "ok");
  render();
}

async function streamState(session: DeviceSession): Promise<void> {
  const enable = !session.streamEnabled;
  if (enable) {
    stopStatePolling(session);
    try {
      await session.transport.streamState(100, true);
      session.streamEnabled = true;
      setStatus(`${session.label} live state streaming enabled.`, "ok");
    } catch (error) {
      session.streamEnabled = true;
      startStatePolling(session);
      setStatus(`${session.label} stream_state failed; using controlled polling fallback: ${errorToString(error)}`, "warn");
    }
  } else {
    session.streamEnabled = false;
    stopStatePolling(session);
    await session.transport.streamState(250, false).catch(() => undefined);
    setStatus(`${session.label} live state stopped.`, "ok");
  }
  render();
}

function startStatePolling(session: DeviceSession): void {
  stopStatePolling(session);
  const poll = async () => {
    if (!session.streamEnabled || session.polling || !isConnected(session)) return;
    session.polling = true;
    try {
      const response = await session.transport.getState();
      session.state = normalizeState(response.fields);
      queueLiveStateUpdate(session);
    } catch (error) {
      appendLog(session, `! state poll: ${errorToString(error)}`);
    } finally {
      session.polling = false;
    }
  };
  void poll();
  session.pollTimer = window.setInterval(() => void poll(), 500);
}

function stopStatePolling(session: DeviceSession): void {
  if (session.pollTimer !== 0) {
    window.clearInterval(session.pollTimer);
    session.pollTimer = 0;
  }
  session.polling = false;
}

async function sendRaw(form: HTMLFormElement, session: DeviceSession): Promise<void> {
  const input = form?.elements.namedItem("line") as HTMLInputElement | null;
  const line = input?.value.trim();
  if (!line) return;
  await session.transport.sendLine(line);
  if (input) input.value = "";
}

async function loadConfig(): Promise<void> {
  const response = await ui.rc.transport.getConfig();
  ui.rc.config = normalizeConfig(response.fields);
  ui.rc.dirty = false;
  ui.rc.dirtyFields.clear();
  setStatus("Loaded RC handset config.", "ok");
  render();
}

async function applyConfig(): Promise<void> {
  const fields = collectConfigFields(ui.rc.config);
  if (!fields.length) {
    setStatus("No RC config fields to save.", "info");
    return;
  }
  for (const [field, value] of fields) {
    await ui.rc.transport.setConfig(field, value);
  }
  ui.rc.dirty = false;
  ui.rc.dirtyFields.clear();
  await refreshStateSnapshot();
  setStatus("Applied RC config to runtime. Use Save to persist after verification.", "ok");
  render();
}

async function saveConfig(): Promise<void> {
  if (ui.rc.dirty) {
    await applyConfig();
  }
  await ui.rc.transport.save();
  await refreshStateSnapshot();
  setStatus("Saved current RC config to EEPROM.", "ok");
  render();
}

async function persistConfigPath(path: string): Promise<void> {
  const [field, value] = configCommandForPath(path);
  await ui.rc.transport.setConfig(field, value);
  ui.rc.dirty = false;
  ui.rc.dirtyFields.clear();
  await refreshStateSnapshot();
  setStatus("Applied RC config to runtime. Use Save to persist after verification.", "ok");
}

async function persistFilterChange(_key: keyof FilterConfig): Promise<void> {
  await ui.rc.transport.setConfig("filter", filterConfigValue());
  ui.rc.dirty = false;
  ui.rc.dirtyFields.clear();
  await refreshStateSnapshot();
  setStatus("Applied filter config to runtime. Use Save to persist after verification.", "ok");
}

async function refreshStateSnapshot(): Promise<void> {
  if (!isConnected(ui.rc)) return;
  const response = await ui.rc.transport.getState();
  ui.rc.state = normalizeState(response.fields);
  queueLiveStateUpdate(ui.rc);
}

async function resetDefaults(): Promise<void> {
  await ui.rc.transport.resetDefaults("rc_config");
  await loadConfig();
  await refreshStateSnapshot();
  setStatus("Reset defaults, saved, and reloaded RC config.", "ok");
  render();
}

async function calibrationStart(): Promise<void> {
  const response = await ui.rc.transport.calibrationStart();
  mergeLiveStateFields(ui.rc, response.fields);
  queueLiveStateUpdate(ui.rc);
  setStatus("Calibration started. Move sticks through full travel.", "ok");
  render();
}

async function calibrationSample(): Promise<void> {
  const sample = await ui.rc.transport.calibrationSample();
  mergeCalibrationFields(ui.rc, sample.fields);
  mergeLiveStateFields(ui.rc, sample.fields);
  const response = await ui.rc.transport.getState();
  ui.rc.state = normalizeState(response.fields);
  queueLiveStateUpdate(ui.rc);
  setStatus("Captured calibration sample.", "ok");
  render();
}

async function calibrationFinish(): Promise<void> {
  await ui.rc.transport.calibrationFinish(true);
  await loadConfig();
  await refreshStateSnapshot();
  setStatus("Calibration finished and saved.", "ok");
  render();
}

async function txHello(): Promise<void> {
  const response = await ui.rc.transport.txHello();
  ui.rc.txInfo = { ...response.fields };
  setStatus(response.fields.tx_present === "1" ? "Attached TX discovered through RC USB." : "TX missing behind RC USB.", response.fields.tx_present === "1" ? "ok" : "warn");
  render();
}

async function bindingCommand(target: BindingTarget, command: "get" | "set" | "verify" | "clear"): Promise<void> {
  const session = target === "tx" ? ui.rc : ui.rx;
  let response: RcMessage;
  if (command === "get") response = await session.transport.bindingGet(target);
  else if (command === "set") response = await session.transport.bindingSet(target, ui.phrase);
  else if (command === "verify") response = await session.transport.bindingVerify(target, ui.phrase);
  else response = await session.transport.bindingClear(target);

  session.binding = normalizeBinding(response.fields);
  setStatus(`${target.toUpperCase()} binding ${command} complete.`, "ok");
  render();
}

function applyMessage(session: DeviceSession, message: RcMessage): void {
  if (message.fields.role) session.info.role = message.fields.role;
  if (message.fields.fw) session.info.fw = message.fields.fw;
  collectCaps(session, message.fields.caps);

  if (message.kind === "state") session.state = normalizeState(message.fields);
  if (message.kind === "config") {
    session.config = normalizeConfig(message.fields);
    session.dirty = false;
    session.dirtyFields.clear();
  }
  if (message.kind === "binding") session.binding = normalizeBinding(message.fields);
}

function isLiveStateMessage(message: RcMessage): boolean {
  return message.kind === "state" ||
    message.fields.adc !== undefined ||
    message.fields.adc_filtered !== undefined ||
    message.fields.ch !== undefined ||
    message.fields.toggles !== undefined ||
    message.fields.safety_hold !== undefined;
}

function mergeLiveStateFields(session: DeviceSession, fields: Record<string, string>): void {
  const current = session.state ?? emptyLiveState();
  session.state = {
    ...current,
    adc: fields.adc ? parseNumberArray(fields.adc, current.adc) : current.adc,
    adcFiltered: fields.adc_filtered ? parseNumberArray(fields.adc_filtered, current.adcFiltered) : current.adcFiltered,
    ch: fields.ch ? parseNumberArray(fields.ch, current.ch) : current.ch,
    toggles: fields.toggles ? parseNumberArray(fields.toggles, current.toggles) : current.toggles,
    lq: fields.lq ?? current.lq,
    rssi: fields.rssi ?? current.rssi,
    snr: fields.snr ?? current.snr,
    txBattMv: fields.tx_batt_mv ?? current.txBattMv,
    rxBattMv: fields.rx_batt_mv ?? current.rxBattMv,
    txPresent: fields.tx_present === undefined ? current.txPresent : boolField(fields.tx_present),
    safetyHold: fields.safety_hold === undefined ? current.safetyHold : boolField(fields.safety_hold),
    haveChannels: fields.have_channels === undefined ? current.haveChannels : boolField(fields.have_channels),
  };
}

function mergeCalibrationFields(session: DeviceSession, fields: Record<string, string>): void {
  if (fields.cal_min) session.config.calMin = parseNumberArray(fields.cal_min, session.config.calMin);
  if (fields.cal_center) session.config.calCenter = parseNumberArray(fields.cal_center, session.config.calCenter);
  if (fields.cal_max) session.config.calMax = parseNumberArray(fields.cal_max, session.config.calMax);
}

function emptyLiveState(): LiveState {
  return {
    adc: [],
    adcFiltered: [],
    ch: [],
    toggles: [],
    lq: "-",
    rssi: "-",
    snr: "-",
    txBattMv: "",
    rxBattMv: "",
    txPresent: false,
    safetyHold: false,
    haveChannels: false,
  };
}

function updateLiveTables(session: DeviceSession): void {
  if (session.key !== "rc" || !session.state) return;
  updateIndexedText("[data-live-adc-index]", session.state.adc);
  updateIndexedText("[data-live-adc-filtered-index]", session.state.adcFiltered);
  updateIndexedText("[data-live-ch-index]", session.state.ch);
}

function updateIndexedText(selector: string, values: number[]): void {
  document.querySelectorAll<HTMLElement>(selector).forEach((element) => {
    const indexText = element.dataset.liveAdcIndex ?? element.dataset.liveAdcFilteredIndex ?? element.dataset.liveChIndex;
    const index = Number(indexText);
    element.textContent = Number.isInteger(index) && values[index] !== undefined ? String(values[index]) : "-";
  });
}

function syncConfigControls(path: string, source: HTMLInputElement | HTMLSelectElement): void {
  const [group, , indexText] = path.split(".");
  if (group === "cal") {
    const index = Number(indexText);
    if (Number.isInteger(index)) {
      syncMirroredControls(`[data-config-path="cal.min.${index}"]`, source, ui.rc.config.calMin[index]);
      syncMirroredControls(`[data-config-path="cal.center.${index}"]`, source, ui.rc.config.calCenter[index]);
      syncMirroredControls(`[data-config-path="cal.max.${index}"]`, source, ui.rc.config.calMax[index]);
      return;
    }
  }
  syncMirroredControls(`[data-config-path="${cssEscape(path)}"]`, source);
}

function syncMirroredControls(selector: string, source: HTMLInputElement | HTMLSelectElement, value?: string | number | boolean): void {
  const nextValue = value ?? readInputValue(source);
  document.querySelectorAll<HTMLInputElement | HTMLSelectElement>(selector).forEach((element) => {
    if (element === source && value === undefined) return;
    if (element instanceof HTMLInputElement && element.type === "checkbox") {
      element.checked = Boolean(nextValue);
      return;
    }
    element.value = String(nextValue);
  });
}

function updateStickMonitors(session: DeviceSession): void {
  if (!session.state) return;
  const state = session.state;
  document.querySelectorAll<HTMLElement>(`[data-stick-monitor="${session.key}"]`).forEach((root) => {
    const safety = root.querySelector<HTMLElement>("[data-stick-safety]");
    if (safety) {
      safety.textContent = stickSafetyText(state);
      safety.classList.toggle("warn", state.safetyHold);
    }
    for (const stick of ["left", "right"] as const) {
      const dot = root.querySelector<HTMLElement>(`[data-stick-dot="${stick}"]`);
      const position = stickPosition(stick, state);
      if (dot) {
        dot.style.left = `${position.x}%`;
        dot.style.top = `${position.y}%`;
      }
    }
    root.querySelectorAll<HTMLElement>("[data-live-bar-index]").forEach((bar) => {
      const index = Number(bar.dataset.liveBarIndex);
      const value = Number.isInteger(index) ? session.state?.ch[index] : undefined;
      const fill = bar.querySelector<HTMLElement>("[data-live-bar-fill]");
      const label = bar.querySelector<HTMLElement>("[data-live-bar-value]");
      if (fill) fill.style.width = `${crsfPercent(value, 0)}%`;
      if (label) label.textContent = value === undefined ? "-" : String(value);
    });
  });
}

function stickPosition(id: "left" | "right", state: LiveState | null): { x: number; y: number } {
  const xIndex = id === "left" ? 3 : 1;
  const yIndex = id === "left" ? 2 : 0;
  return {
    x: crsfPercent(state?.ch[xIndex]),
    y: 100 - crsfPercent(state?.ch[yIndex]),
  };
}

function crsfPercent(value: number | undefined, fallback = 50): number {
  if (!Number.isFinite(value)) return fallback;
  const clamped = Math.min(CRSF_RAW_MAX, Math.max(CRSF_RAW_MIN, Number(value)));
  return Math.round(((clamped - CRSF_RAW_MIN) * 10000) / (CRSF_RAW_MAX - CRSF_RAW_MIN)) / 100;
}

function normalizeConfig(fields: Record<string, string>): RcConfig {
  const filter = parseStringArray(fields.filter);
  return {
    version: numberField(fields.version, 0),
    calMin: parseNumberArray(fields.cal_min, defaultConfig.calMin),
    calCenter: parseNumberArray(fields.cal_center, defaultConfig.calCenter),
    calMax: parseNumberArray(fields.cal_max, defaultConfig.calMax),
    invert: parseNumberArray(fields.invert, defaultConfig.invert),
    deadzone: parseNumberArray(fields.deadzone, defaultConfig.deadzone),
    functions: parseNumberArray(fields.function, defaultConfig.functions),
    trim: parseNumberArray(fields.trim, defaultConfig.trim),
    cutoffMin: parseNumberArray(fields.cutoff_min, defaultConfig.cutoffMin),
    cutoffMax: parseNumberArray(fields.cutoff_max, defaultConfig.cutoffMax),
    filter: {
      oversample: numberField(filter[0], defaultConfig.filter.oversample),
      lowPass: numberField(fields["filter.low_pass"] ?? fields.low_pass ?? filter[1], defaultConfig.filter.lowPass),
      temporalBlend: boolField(filter[2]),
      highPass: numberField(fields["filter.high_pass"] ?? fields.high_pass ?? filter[3], defaultConfig.filter.highPass),
    },
  };
}

function normalizeState(fields: Record<string, string>): LiveState {
  return {
    adc: parseNumberArray(fields.adc, []),
    adcFiltered: parseNumberArray(fields.adc_filtered, []),
    ch: parseNumberArray(fields.ch, []),
    toggles: parseNumberArray(fields.toggles, []),
    lq: fields.lq ?? "-",
    rssi: fields.rssi ?? "-",
    snr: fields.snr ?? "-",
    txBattMv: fields.tx_batt_mv ?? "",
    rxBattMv: fields.rx_batt_mv ?? "",
    txPresent: boolField(fields.tx_present),
    safetyHold: boolField(fields.safety_hold),
    haveChannels: fields.have_channels === undefined ? fields.ch !== undefined : boolField(fields.have_channels),
  };
}

function normalizeBinding(fields: Record<string, string>): BindingInfo {
  return {
    target: fields.target ?? "",
    uid: fields.uid ?? "",
    uidCheck: fields.uid_check ?? "",
    persisted: boolField(fields.persisted),
    requiresReboot: boolField(fields.requires_reboot),
    phrase: fields.phrase ?? "",
  };
}

function collectConfigFields(config: RcConfig, onlyFields?: Set<string>): Array<[string, string | number]> {
  const fields: Array<[string, string | number]> = [];
  const include = (field: string) => !onlyFields || onlyFields.has(field);
  for (let index = 0; index < 4; index += 1) {
    pushConfigField(fields, include, `cal.axis.${index}`, [
      config.calMin[index] ?? defaultConfig.calMin[index],
      config.calCenter[index] ?? defaultConfig.calCenter[index],
      config.calMax[index] ?? defaultConfig.calMax[index],
    ].join(","));
    pushConfigField(fields, include, `axis.invert.${index}`, config.invert[index] ? 1 : 0);
    pushConfigField(fields, include, `axis.deadzone.${index}`, config.deadzone[index] ?? 0);
    pushConfigField(fields, include, `axis.function.${index}`, config.functions[index] ?? index);
  }
  for (let index = 0; index < 8; index += 1) {
    pushConfigField(fields, include, `channel.trim.${index}`, config.trim[index] ?? 0);
    pushConfigField(fields, include, `channel.cutoff_min.${index}`, config.cutoffMin[index] ?? 172);
    pushConfigField(fields, include, `channel.cutoff_max.${index}`, config.cutoffMax[index] ?? 1811);
  }
  if (include("filter")) fields.push(["filter", filterConfigValue()]);
  return fields;
}

function configCommandForPath(path: string): [string, string | number] {
  const [group, key, indexText] = path.split(".");
  const index = Number(indexText);
  if (group === "cal" && Number.isInteger(index)) {
    return [
      `cal.axis.${index}`,
      [
        ui.rc.config.calMin[index] ?? defaultConfig.calMin[index],
        ui.rc.config.calCenter[index] ?? defaultConfig.calCenter[index],
        ui.rc.config.calMax[index] ?? defaultConfig.calMax[index],
      ].join(","),
    ];
  }
  if (group === "axis" && Number.isInteger(index)) {
    if (key === "function") return [path, ui.rc.config.functions[index] ?? index];
    if (key === "invert") return [path, ui.rc.config.invert[index] ? 1 : 0];
    if (key === "deadzone") return [path, ui.rc.config.deadzone[index] ?? 0];
  }
  if (group === "channel" && Number.isInteger(index)) {
    if (key === "trim") return [path, ui.rc.config.trim[index] ?? 0];
    if (key === "cutoff_min") return [path, ui.rc.config.cutoffMin[index] ?? 172];
    if (key === "cutoff_max") return [path, ui.rc.config.cutoffMax[index] ?? 1811];
  }
  throw new Error(`Unsupported config field: ${path}`);
}

function filterConfigValue(): string {
  const filter = ui.rc.config.filter;
  return [filter.oversample, filter.lowPass, filter.temporalBlend ? 1 : 0, filter.highPass].join(",");
}

function pushConfigField(
  fields: Array<[string, string | number]>,
  include: (field: string) => boolean,
  field: string,
  value: string | number,
): void {
  if (include(field)) fields.push([field, value]);
}

function markDirtyFilterFields(key: keyof FilterConfig): void {
  ui.rc.dirtyFields.add("filter");
  if (key === "lowPass") ui.rc.dirtyFields.add("filter.low_pass");
  if (key === "highPass") ui.rc.dirtyFields.add("filter.high_pass");
}

function bindingComparison(): { level: StatusLevel; label: string; title: string; message: string } {
  const tx = ui.rc.binding;
  const rx = ui.rx.binding;
  if (!tx && !rx) {
    return { level: "warn", label: "not verified", title: "No binding verification yet", message: "Set and verify both sides to compare UID checks." };
  }
  if (!tx || !rx) {
    return { level: "warn", label: "partial", title: "Only one side has binding data", message: "Finish the other side before rebooting or flying." };
  }
  if (!tx.uidCheck || !rx.uidCheck) {
    return { level: "warn", label: "missing uid_check", title: "UID check missing", message: "Firmware response did not include uid_check on both sides." };
  }
  if (tx.uidCheck !== rx.uidCheck) {
    return { level: "bad", label: "mismatch", title: "TX/RX binding mismatch", message: `TX ${tx.uidCheck} does not match RX ${rx.uidCheck}. Rewrite both with the same phrase.` };
  }
  if (tx.requiresReboot || rx.requiresReboot) {
    return { level: "warn", label: "match, reboot", title: "UID checks match", message: "At least one side reports requires_reboot=1 before the phrase is active." };
  }
  return { level: "ok", label: "match", title: "UID checks match", message: "Both sides report the same uid_check." };
}

function updateConfig(path: string, value: string | number | boolean): void {
  const [group, key, indexText] = path.split(".");
  const index = Number(indexText);
  if (!Number.isInteger(index)) return;
  if (group === "cal") {
    if (key === "min") ui.rc.config.calMin[index] = clampInt(Number(value), 0, 4095);
    if (key === "center") ui.rc.config.calCenter[index] = clampInt(Number(value), 0, 4095);
    if (key === "max") ui.rc.config.calMax[index] = clampInt(Number(value), 0, 4095);
    normalizeCalibrationAxis(index, key);
  }
  if (group === "axis") {
    if (key === "function") ui.rc.config.functions[index] = Number(value);
    if (key === "invert") ui.rc.config.invert[index] = value ? 1 : 0;
    if (key === "deadzone") ui.rc.config.deadzone[index] = Number(value);
  }
  if (group === "channel") {
    if (key === "trim") ui.rc.config.trim[index] = Number(value);
    if (key === "cutoff_min") ui.rc.config.cutoffMin[index] = Number(value);
    if (key === "cutoff_max") ui.rc.config.cutoffMax[index] = Number(value);
  }
}

function updateFilter(key: keyof FilterConfig, value: string | number | boolean): void {
  if (key === "temporalBlend") {
    ui.rc.config.filter[key] = Boolean(value);
  } else {
    const max = key === "oversample" ? 32 : 100;
    const min = key === "oversample" ? 1 : 0;
    ui.rc.config.filter[key] = clampInt(Number(value), min, max);
  }
}

function normalizeCalibrationAxis(index: number, changedKey: string): void {
  let min = ui.rc.config.calMin[index] ?? 0;
  let center = ui.rc.config.calCenter[index] ?? 2048;
  let max = ui.rc.config.calMax[index] ?? 4095;

  min = clampInt(min, 0, 4093);
  max = clampInt(max, 2, 4095);
  center = clampInt(center, 1, 4094);

  if (changedKey === "min") {
    if (min >= max - 1) min = max - 2;
    if (center <= min) center = min + 1;
  } else if (changedKey === "max") {
    if (max <= min + 1) max = min + 2;
    if (center >= max) center = max - 1;
  } else {
    if (center <= min) center = min + 1;
    if (center >= max) center = max - 1;
  }

  if (min < 0) min = 0;
  if (max > 4095) max = 4095;
  if (center <= min) center = min + 1;
  if (center >= max) center = max - 1;

  ui.rc.config.calMin[index] = min;
  ui.rc.config.calCenter[index] = center;
  ui.rc.config.calMax[index] = max;
}

function parseNumberArray(value: string | undefined, fallback: number[]): number[] {
  const parsed = parseStringArray(value)
    .map((item) => Number(item))
    .filter((item) => Number.isFinite(item));
  return parsed.length ? parsed : [...fallback];
}

function parseStringArray(value: string | undefined): string[] {
  return value ? value.split(",").map((item) => item.trim()).filter(Boolean) : [];
}

function numberField(value: string | undefined, fallback: number): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function boolField(value: string | undefined): boolean {
  return value === "1" || value === "true";
}

function collectCaps(session: DeviceSession, caps: string | undefined): void {
  if (!caps) return;
  for (const cap of caps.split(",").map((item) => item.trim()).filter(Boolean)) {
    session.caps.add(cap);
  }
}

function appendLog(session: DeviceSession, line: string): void {
  const time = new Date().toLocaleTimeString();
  session.logLines.push(`[${time}] ${line}`);
  if (session.logLines.length > MAX_LOG_LINES) {
    session.logLines.splice(0, session.logLines.length - MAX_LOG_LINES);
  }
  updateTerminalPanel(session);
}

function updateTerminalPanel(session: DeviceSession): void {
  const terminal = document.querySelector<HTMLPreElement>(`[data-terminal="${session.key}"]`);
  if (!terminal) {
    scheduleRender();
    return;
  }
  terminal.textContent = session.logLines.join("\n");
  terminal.scrollTop = terminal.scrollHeight;
}

function tabButton(screen: UiState["activeScreen"], label: string): string {
  return `<button class="secondary ${ui.activeScreen === screen ? "active" : ""}" data-action="screen" data-screen="${screen}">${label}</button>`;
}

function isConnected(session: DeviceSession): boolean {
  return session.status === "connected" || session.status === "reconnecting";
}

function setStatus(message: string, level: StatusLevel): void {
  ui.statusText = message;
  ui.statusLevel = level;
  const banner = document.querySelector<HTMLElement>(".status-banner");
  if (banner) {
    banner.className = `status-banner ${level}`;
    banner.textContent = message;
  }
}

function updateBindingButtonState(): void {
  document.querySelectorAll<HTMLButtonElement>('[data-action="binding-set"], [data-action="binding-verify"]').forEach((button) => {
    const target = button.dataset.target as BindingTarget;
    const session = target === "tx" ? ui.rc : ui.rx;
    button.disabled = !isConnected(session) || !ui.phrase;
  });
}

function readInputValue(input: HTMLInputElement | HTMLSelectElement): string | number | boolean {
  if (input instanceof HTMLInputElement && input.type === "checkbox") return input.checked;
  if (input instanceof HTMLInputElement && (input.type === "number" || input.type === "range")) return Number(input.value);
  if (input instanceof HTMLSelectElement) return Number(input.value);
  return input.value;
}

function clampInt(value: number, min: number, max: number): number {
  if (!Number.isFinite(value)) return min;
  return Math.round(Math.min(max, Math.max(min, value)));
}

function cssEscape(value: string): string {
  return globalThis.CSS?.escape ? globalThis.CSS.escape(value) : value.replace(/["\\]/g, "\\$&");
}

function cloneConfig(config: RcConfig): RcConfig {
  return structuredClone(config) as RcConfig;
}

function disabled(value: boolean): string {
  return value ? "disabled" : "";
}

function selected(value: boolean): string {
  return value ? "selected" : "";
}

function checked(value: boolean): string {
  return value ? "checked" : "";
}

function yesNo(value: boolean): string {
  return value ? "yes" : "no";
}

function mv(value: string): string {
  return value ? `${escapeHtml(value)} mV` : "-";
}

function errorToString(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

function escapeHtml(value: string): string {
  return value.replace(/[&<>"']/g, (char) => {
    switch (char) {
      case "&":
        return "&amp;";
      case "<":
        return "&lt;";
      case ">":
        return "&gt;";
      case '"':
        return "&quot;";
      default:
        return "&#39;";
    }
  });
}

function escapeAttribute(value: string): string {
  return escapeHtml(value).replace(/`/g, "&#96;");
}
