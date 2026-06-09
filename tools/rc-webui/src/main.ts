import "./style.css";
import { RcMessage, RcSerialTransport, TransportStatus } from "./rcTransport";

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
  highPass: boolean;
}

interface LiveState {
  adc: number[];
  ch: number[];
  toggles: number[];
  lq: string;
  rssi: string;
  snr: string;
  txBattMv: string;
  rxBattMv: string;
  txPresent: boolean;
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

const AXES = ["Aileron", "Elevator", "Throttle", "Rudder"];
const CHANNELS = ["CH1", "CH2", "CH3", "CH4", "CH5", "CH6", "CH7", "CH8"];
const FUNCTION_OPTIONS = ["Aileron", "Elevator", "Throttle", "Rudder"];
const MAX_LOG_LINES = 80;

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
    highPass: false,
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
    ui.rc.dirty = true;
    render();
    return;
  }

  const filterKey = target.dataset.filterKey as keyof FilterConfig | undefined;
  if (filterKey) {
    updateFilter(filterKey, readInputValue(target));
    ui.rc.dirty = true;
    render();
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
    if (line.startsWith("rc.v1 state ")) return;
    appendLog(session, line);
  });
  transport.on("message", ({ message }) => {
    applyMessage(session, message);
    if (message.kind === "state") updateLiveStatePanel(session);
    else render();
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
        ${tabButton("filters", "Filters / Save")}
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
  if (!session.state) return `<p class="muted state-empty" data-live-state="${session.key}">No live state yet.</p>`;
  return `
    <div class="live-grid" data-live-state="${session.key}">
      <div><span>ADC</span><strong data-live-field="adc">${escapeHtml(session.state.adc.join(", ") || "-")}</strong></div>
      <div><span>Channels</span><strong data-live-field="ch">${escapeHtml(session.state.ch.join(", ") || "-")}</strong></div>
      <div><span>Toggles</span><strong data-live-field="toggles">${escapeHtml(session.state.toggles.join(", ") || "-")}</strong></div>
      <div><span>Link</span><strong data-live-field="link">${liveLinkText(session.state)}</strong></div>
      <div><span>Battery</span><strong data-live-field="battery">${liveBatteryText(session.state)}</strong></div>
    </div>
  `;
}

function updateLiveStatePanel(session: DeviceSession): void {
  if (!session.state) return;
  const root = document.querySelector<HTMLElement>(`[data-live-state="${session.key}"]`);
  if (!root || root.classList.contains("state-empty")) {
    scheduleRender();
    return;
  }
  setLiveField(root, "adc", session.state.adc.join(", ") || "-");
  setLiveField(root, "ch", session.state.ch.join(", ") || "-");
  setLiveField(root, "toggles", session.state.toggles.join(", ") || "-");
  setLiveField(root, "link", `LQ ${session.state.lq} RSSI ${session.state.rssi} SNR ${session.state.snr}`);
  setLiveField(root, "battery", `TX ${mv(session.state.txBattMv)} RX ${mv(session.state.rxBattMv)}`);
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

function terminalPanel(session: DeviceSession): string {
  return `
    <section class="panel terminal-panel">
      <div class="panel-title">
        <h2>${session.label} terminal</h2>
        <button class="secondary" data-action="clear-log" data-device="${session.key}">Clear</button>
      </div>
      <pre class="terminal">${escapeHtml(session.logLines.join("\n"))}</pre>
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
          <p class="muted">Center, sample full stick travel, then finish with optional save.</p>
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
          <span>Return sticks to center, then apply or apply and persist.</span>
          <span class="inline-actions">
            <button data-action="cal-finish" data-save="0" ${disabled(!isConnected(ui.rc))}>Apply only</button>
            <button data-action="cal-finish" data-save="1" ${disabled(!isConnected(ui.rc))}>Apply + save</button>
          </span>
        </li>
      </ol>
      <div class="table-wrap">
        <table>
          <thead><tr><th>Axis</th><th>Min</th><th>Center</th><th>Max</th><th>Live ADC</th></tr></thead>
          <tbody>
            ${AXES.map((axis, index) => `
              <tr>
                <td>${axis}</td>
                <td>${config.calMin[index] ?? "-"}</td>
                <td>${config.calCenter[index] ?? "-"}</td>
                <td>${config.calMax[index] ?? "-"}</td>
                <td>${ui.rc.state?.adc[index] ?? "-"}</td>
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
          <p class="muted">Edit locally, then apply to send <code>set_config</code> updates and <code>apply</code>.</p>
        </div>
        <button class="secondary" data-action="load-config" ${disabled(!isConnected(ui.rc))}>Reload</button>
      </div>

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

      ${applyPanel()}
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
      <td>${ui.rc.state?.adc[index] ?? "-"}</td>
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
      <td>${ui.rc.state?.ch[index] ?? "-"}</td>
    </tr>
  `;
}

function filtersScreen(): string {
  const filter = ui.rc.config.filter;
  return `
    <section class="panel screen-panel">
      <div class="panel-title split">
        <div>
          <h2>Filters, Apply, Save, Defaults</h2>
          <p class="muted">Filter controls write both named fields where available and the compact <code>filter</code> array.</p>
        </div>
        <span class="tag ${ui.rc.dirty ? "warn" : "ok"}">${ui.rc.dirty ? "edited locally" : "in sync"}</span>
      </div>

      <div class="form-grid">
        <label>Oversample level
          <input type="number" min="0" max="64" data-filter-key="oversample" value="${filter.oversample}" />
        </label>
        <label>Low-pass level
          <input type="number" min="0" max="100" data-filter-key="lowPass" value="${filter.lowPass}" />
        </label>
        <label class="check">
          <input type="checkbox" data-filter-key="temporalBlend" ${checked(filter.temporalBlend)} />
          Temporal blend
        </label>
        <label class="check">
          <input type="checkbox" data-filter-key="highPass" ${checked(filter.highPass)} />
          High-pass drift filter
        </label>
      </div>

      ${applyPanel()}

      <div class="danger-zone">
        <h3>Defaults</h3>
        <p class="muted">Sends <code>reset_defaults target=rc_config</code>, applies, then reloads config.</p>
        <button class="danger" data-action="defaults" ${disabled(!isConnected(ui.rc))}>Reset RC config defaults</button>
      </div>
    </section>
  `;
}

function applyPanel(): string {
  return `
    <div class="apply-panel">
      <div>
        <strong>${ui.rc.dirty ? "Unsaved local edits" : "No local edits"}</strong>
        <p class="muted">Apply updates runtime config. Save persists the active config on the device.</p>
      </div>
      <div class="actions">
        <button data-action="apply-config" ${disabled(!isConnected(ui.rc))}>Apply edited config</button>
        <button class="secondary" data-action="save-config" ${disabled(!isConnected(ui.rc))}>Save</button>
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
  if (action === "cal-finish") await calibrationFinish(target.dataset.save === "1");
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
}

async function streamState(session: DeviceSession): Promise<void> {
  const enable = !session.streamEnabled;
  session.streamEnabled = enable;
  if (enable) {
    await session.transport.streamState(250, false).catch(() => undefined);
    startStatePolling(session);
  } else {
    stopStatePolling(session);
    await session.transport.streamState(250, false).catch(() => undefined);
  }
  setStatus(`${session.label} live state ${enable ? "polling enabled" : "polling stopped"}.`, "ok");
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
      updateLiveStatePanel(session);
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
  setStatus("Loaded RC handset config.", "ok");
}

async function applyConfig(): Promise<void> {
  for (const [field, value] of collectConfigFields(ui.rc.config)) {
    await ui.rc.transport.setConfig(field, value);
  }
  await ui.rc.transport.apply();
  ui.rc.dirty = false;
  setStatus("Applied edited config. Use Save to persist.", "ok");
}

async function saveConfig(): Promise<void> {
  await ui.rc.transport.save();
  setStatus("Saved current RC config.", "ok");
}

async function resetDefaults(): Promise<void> {
  await ui.rc.transport.resetDefaults("rc_config");
  await ui.rc.transport.apply();
  await loadConfig();
  setStatus("Reset defaults, applied, and reloaded RC config.", "ok");
}

async function calibrationStart(): Promise<void> {
  await ui.rc.transport.calibrationStart();
  setStatus("Calibration started. Move sticks through full travel.", "ok");
}

async function calibrationSample(): Promise<void> {
  await ui.rc.transport.calibrationSample();
  const response = await ui.rc.transport.getState();
  ui.rc.state = normalizeState(response.fields);
  setStatus("Captured calibration sample.", "ok");
}

async function calibrationFinish(save: boolean): Promise<void> {
  await ui.rc.transport.calibrationFinish(save);
  await loadConfig();
  setStatus(save ? "Calibration finished and saved." : "Calibration finished and applied.", "ok");
}

async function txHello(): Promise<void> {
  const response = await ui.rc.transport.txHello();
  ui.rc.txInfo = { ...response.fields };
  setStatus(response.fields.tx_present === "1" ? "Attached TX discovered through RC USB." : "TX missing behind RC USB.", response.fields.tx_present === "1" ? "ok" : "warn");
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
}

function applyMessage(session: DeviceSession, message: RcMessage): void {
  if (message.fields.role) session.info.role = message.fields.role;
  if (message.fields.fw) session.info.fw = message.fields.fw;
  collectCaps(session, message.fields.caps);

  if (message.kind === "state") session.state = normalizeState(message.fields);
  if (message.kind === "config") {
    session.config = normalizeConfig(message.fields);
    session.dirty = false;
  }
  if (message.kind === "binding") session.binding = normalizeBinding(message.fields);
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
      highPass: boolField(fields["filter.high_pass"] ?? fields.high_pass ?? filter[3]),
    },
  };
}

function normalizeState(fields: Record<string, string>): LiveState {
  return {
    adc: parseNumberArray(fields.adc, []),
    ch: parseNumberArray(fields.ch, []),
    toggles: parseNumberArray(fields.toggles, []),
    lq: fields.lq ?? "-",
    rssi: fields.rssi ?? "-",
    snr: fields.snr ?? "-",
    txBattMv: fields.tx_batt_mv ?? "",
    rxBattMv: fields.rx_batt_mv ?? "",
    txPresent: boolField(fields.tx_present),
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

function collectConfigFields(config: RcConfig): Array<[string, string | number]> {
  const fields: Array<[string, string | number]> = [];
  for (let index = 0; index < 4; index += 1) {
    fields.push([`cal.min.${index}`, config.calMin[index] ?? defaultConfig.calMin[index]]);
    fields.push([`cal.center.${index}`, config.calCenter[index] ?? defaultConfig.calCenter[index]]);
    fields.push([`cal.max.${index}`, config.calMax[index] ?? defaultConfig.calMax[index]]);
    fields.push([`axis.invert.${index}`, config.invert[index] ? 1 : 0]);
    fields.push([`axis.deadzone.${index}`, config.deadzone[index] ?? 0]);
    fields.push([`axis.function.${index}`, config.functions[index] ?? index]);
  }
  for (let index = 0; index < 8; index += 1) {
    fields.push([`channel.trim.${index}`, config.trim[index] ?? 0]);
    fields.push([`channel.cutoff_min.${index}`, config.cutoffMin[index] ?? 172]);
    fields.push([`channel.cutoff_max.${index}`, config.cutoffMax[index] ?? 1811]);
  }
  fields.push(["filter.low_pass", config.filter.lowPass]);
  fields.push(["filter.high_pass", config.filter.highPass ? 1 : 0]);
  fields.push([
    "filter",
    [config.filter.oversample, config.filter.lowPass, config.filter.temporalBlend ? 1 : 0, config.filter.highPass ? 1 : 0].join(","),
  ]);
  return fields;
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
  if (key === "temporalBlend" || key === "highPass") {
    ui.rc.config.filter[key] = Boolean(value);
  } else {
    ui.rc.config.filter[key] = Number(value);
  }
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
  render();
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
  if (input instanceof HTMLInputElement && input.type === "number") return Number(input.value);
  if (input instanceof HTMLSelectElement) return Number(input.value);
  return input.value;
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
