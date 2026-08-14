'use strict';

/* ---------------------------------------------------------------------
 * IP <-> uint32 helpers.
 *
 * Byte order convention: the firmware stores NetworkConfig.static_ip / gateway /
 * netmask as a plain uint32_t built the same way the config_json layer (Task 18)
 * serializes it: MOST significant octet of the dotted-quad in the HIGH byte of
 * the integer (standard network / big-endian byte order), i.e. for "192.168.1.50":
 *   value = (192 << 24) | (168 << 16) | (1 << 8) | 50
 * This is NOT the raw in-memory byte order of an ESP32 IPAddress (which stores
 * octets in wire order at ascending addresses, effectively little-endian when
 * read as a uint32 on that little-endian CPU) — the config layer explicitly
 * re-packs to big-endian before JSON so the value is portable across any client.
 * Keep this function and config_json's ipToUint32/uint32ToIp in lock-step; if one
 * side changes the byte order, both must change together.
 * ------------------------------------------------------------------- */
function ipToString(u32) {
  u32 = u32 >>> 0;
  return [
    (u32 >>> 24) & 0xff,
    (u32 >>> 16) & 0xff,
    (u32 >>> 8) & 0xff,
    u32 & 0xff,
  ].join('.');
}

function stringToIp(str) {
  const parts = String(str).trim().split('.');
  if (parts.length !== 4) return 0;
  let u32 = 0;
  for (let i = 0; i < 4; i++) {
    const octet = parseInt(parts[i], 10);
    if (isNaN(octet) || octet < 0 || octet > 255) return 0;
    u32 = (u32 * 256) + octet;
  }
  return u32 >>> 0;
}

/* ---------------------------------------------------------------------
 * Fetch helper with a sticky error banner.
 * ------------------------------------------------------------------- */
const errorBanner = document.getElementById('error-banner');
let errorHideTimer = null;

function showError(msg) {
  errorBanner.textContent = msg;
  errorBanner.classList.remove('hidden');
  if (errorHideTimer) clearTimeout(errorHideTimer);
  errorHideTimer = setTimeout(() => errorBanner.classList.add('hidden'), 6000);
}

async function api(path, method, body) {
  method = method || 'GET';
  const opts = { method, headers: {} };
  if (body !== undefined) {
    opts.headers['Content-Type'] = 'application/json';
    opts.body = JSON.stringify(body);
  }
  let resp;
  try {
    resp = await fetch(path, opts);
  } catch (err) {
    showError('Network error calling ' + path + ': ' + err.message);
    throw err;
  }
  let json = null;
  const text = await resp.text();
  if (text.length) {
    try { json = JSON.parse(text); } catch (e) { /* not JSON, leave null */ }
  }
  if (!resp.ok) {
    const msg = (json && json.error) ? json.error : ('HTTP ' + resp.status);
    showError(path + ': ' + msg);
    throw new Error(msg);
  }
  return json;
}

/* ---------------------------------------------------------------------
 * Hash router.
 * ------------------------------------------------------------------- */
const ROUTES = ['dashboard', 'receivers', 'output', 'pwm', 'voltage', 'network', 'firmware'];

function currentRoute() {
  const h = location.hash.replace(/^#\/?/, '');
  return ROUTES.includes(h) ? h : 'dashboard';
}

function renderRoute() {
  const route = currentRoute();
  ROUTES.forEach((r) => {
    document.getElementById('page-' + r).classList.toggle('hidden', r !== route);
  });
  document.querySelectorAll('.main-nav a').forEach((a) => {
    a.classList.toggle('active', a.dataset.route === route);
  });
  onRouteEnter(route);
}

window.addEventListener('hashchange', renderRoute);

function onRouteEnter(route) {
  if (route === 'receivers') loadReceivers();
  else if (route === 'output') loadOutput();
  else if (route === 'pwm') loadPwm();
  else if (route === 'voltage') loadVoltageConfig();
  else if (route === 'network') loadNetwork();
  else if (route === 'firmware') loadFirmware();
}

/* ---------------------------------------------------------------------
 * Dashboard: 1 Hz status polling, paused when tab hidden.
 * ------------------------------------------------------------------- */
const PROTOCOL_NAMES = ['NONE', 'CRSF', 'SBUS', 'MAVLINK'];
let dashboardTimer = null;

function renderStatus(s) {
  document.getElementById('dash-active-receiver').textContent =
    s.active_receiver < 0 ? 'none' : String(s.active_receiver);
  document.getElementById('dash-active-protocol').textContent =
    PROTOCOL_NAMES[s.active_protocol] || 'NONE';
  document.getElementById('dash-rssi').textContent = s.rssi_percent + ' %';
  document.getElementById('dash-lq').textContent = s.lq_percent + ' %';
  document.getElementById('dash-failsafe').textContent = s.failsafe ? 'FAILSAFE' : 'ok';
  document.getElementById('dash-switch-count').textContent = String(s.switch_count);
  document.getElementById('dash-battery-voltage').textContent = s.battery_voltage.toFixed(2) + ' V';
  document.getElementById('dash-adc-mv').textContent = s.adc_millivolts + ' mV';
  document.getElementById('dash-uptime').textContent = formatUptime(s.uptime_s);
  document.getElementById('dash-free-heap').textContent =
    (s.free_heap / 1024).toFixed(1) + ' KB';
  document.getElementById('dash-wifi-connected').textContent = s.wifi_connected ? 'connected' : 'down';
  document.getElementById('dash-wifi-ap-mode').textContent = s.wifi_ap_mode ? 'AP' : 'STA';
  document.getElementById('dash-wifi-rssi').textContent = s.wifi_rssi + ' dBm';
  document.getElementById('dash-ip').textContent = ipToString(s.ip);
}

function formatUptime(seconds) {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = seconds % 60;
  return String(h).padStart(2, '0') + ':' + String(m).padStart(2, '0') + ':' +
    String(s).padStart(2, '0');
}

async function pollDashboard() {
  if (document.hidden) return;
  try {
    const s = await api('/api/status');
    renderStatus(s);
  } catch (e) { /* error banner already shown */ }
}

function startDashboardPolling() {
  if (dashboardTimer) return;
  pollDashboard();
  dashboardTimer = setInterval(pollDashboard, 1000);
}

document.addEventListener('visibilitychange', () => {
  if (!document.hidden) pollDashboard();
});

async function refreshLogs() {
  try {
    const r = await api('/api/logs');
    document.getElementById('dash-log-lines').textContent = (r.lines || []).join('\n');
  } catch (e) { /* handled */ }
}

document.getElementById('dash-refresh-logs-btn').addEventListener('click', refreshLogs);

/* ---------------------------------------------------------------------
 * Receivers page.
 * ------------------------------------------------------------------- */
function fillReceiverPort(idx, cfg) {
  document.getElementById('rx' + idx + '-enabled').checked = !!cfg.enabled;
  document.getElementById('rx' + idx + '-protocol').value = String(cfg.protocol);
  document.getElementById('rx' + idx + '-priority').value = cfg.priority;
  document.getElementById('rx' + idx + '-baud').value = cfg.baud;
  document.getElementById('rx' + idx + '-rxpin').value = cfg.rx_pin;
  document.getElementById('rx' + idx + '-txpin').value = cfg.tx_pin;
  document.getElementById('rx' + idx + '-inverted').checked = !!cfg.inverted;
}

function readReceiverPort(idx) {
  return {
    enabled: document.getElementById('rx' + idx + '-enabled').checked,
    protocol: parseInt(document.getElementById('rx' + idx + '-protocol').value, 10),
    priority: parseInt(document.getElementById('rx' + idx + '-priority').value, 10),
    baud: parseInt(document.getElementById('rx' + idx + '-baud').value, 10),
    rx_pin: parseInt(document.getElementById('rx' + idx + '-rxpin').value, 10),
    tx_pin: parseInt(document.getElementById('rx' + idx + '-txpin').value, 10),
    inverted: document.getElementById('rx' + idx + '-inverted').checked,
  };
}

async function loadReceivers() {
  const r = await api('/api/config/receivers');
  fillReceiverPort(0, r.receivers[0]);
  fillReceiverPort(1, r.receivers[1]);
  document.getElementById('sel-rssi-threshold').value = r.selection.rssi_threshold_percent;
  document.getElementById('sel-lq-threshold').value = r.selection.lq_threshold_percent;
  document.getElementById('sel-hysteresis').value = r.selection.hysteresis_percent;
  document.getElementById('sel-switch-delay').value = r.selection.switch_delay_ms;
  document.getElementById('sel-min-active-time').value = r.selection.min_active_time_ms;
  document.getElementById('sel-link-timeout').value = r.selection.link_timeout_ms;
}

document.getElementById('receivers-form').addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const body = {
    receivers: [readReceiverPort(0), readReceiverPort(1)],
    selection: {
      rssi_threshold_percent: parseInt(document.getElementById('sel-rssi-threshold').value, 10),
      lq_threshold_percent: parseInt(document.getElementById('sel-lq-threshold').value, 10),
      hysteresis_percent: parseInt(document.getElementById('sel-hysteresis').value, 10),
      switch_delay_ms: parseInt(document.getElementById('sel-switch-delay').value, 10),
      min_active_time_ms: parseInt(document.getElementById('sel-min-active-time').value, 10),
      link_timeout_ms: parseInt(document.getElementById('sel-link-timeout').value, 10),
    },
  };
  const r = await api('/api/config/receivers', 'POST', body);
  flashSaveStatus('receivers-save-status', r);
});

/* ---------------------------------------------------------------------
 * Output page (incl. channel map + system/failsafe fields).
 * ------------------------------------------------------------------- */
function buildChannelMapRows() {
  const tbody = document.getElementById('channel-map-body');
  tbody.innerHTML = '';
  for (let out = 0; out < 16; out++) {
    const tr = document.createElement('tr');
    tr.innerHTML =
      '<td>' + out + '</td>' +
      '<td><input type="number" min="0" max="15" step="1" id="out-map-' + out + '"></td>' +
      '<td><input type="number" min="988" max="2012" step="1" id="sys-failsafe-ch-' + out + '"></td>';
    tbody.appendChild(tr);
  }
}
buildChannelMapRows();

async function loadOutput() {
  const r = await api('/api/config/output');
  document.getElementById('out-protocol').value = String(r.output.protocol);
  document.getElementById('out-baud').value = r.output.baud;
  document.getElementById('out-txpin').value = r.output.tx_pin;
  document.getElementById('out-rxpin').value = r.output.rx_pin;
  document.getElementById('out-inverted').checked = !!r.output.inverted;
  for (let i = 0; i < 16; i++) {
    document.getElementById('out-map-' + i).value = r.output.channel_map[i];
  }
  document.getElementById('sys-failsafe-mode').value = String(r.system.failsafe_mode);
  document.getElementById('sys-log-level').value = String(r.system.log_level);
  document.getElementById('sys-serial-console').checked = !!r.system.serial_console;
  for (let i = 0; i < 16; i++) {
    document.getElementById('sys-failsafe-ch-' + i).value = r.system.failsafe_channels[i];
  }
}

document.getElementById('output-form').addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const channel_map = [];
  const failsafe_channels = [];
  for (let i = 0; i < 16; i++) {
    channel_map.push(parseInt(document.getElementById('out-map-' + i).value, 10));
    failsafe_channels.push(parseInt(document.getElementById('sys-failsafe-ch-' + i).value, 10));
  }
  const body = {
    output: {
      protocol: parseInt(document.getElementById('out-protocol').value, 10),
      baud: parseInt(document.getElementById('out-baud').value, 10),
      tx_pin: parseInt(document.getElementById('out-txpin').value, 10),
      rx_pin: parseInt(document.getElementById('out-rxpin').value, 10),
      inverted: document.getElementById('out-inverted').checked,
      channel_map: channel_map,
    },
    system: {
      log_level: parseInt(document.getElementById('sys-log-level').value, 10),
      serial_console: document.getElementById('sys-serial-console').checked,
      failsafe_mode: parseInt(document.getElementById('sys-failsafe-mode').value, 10),
      failsafe_channels: failsafe_channels,
    },
  };
  const r = await api('/api/config/output', 'POST', body);
  flashSaveStatus('output-save-status', r);
});

/* ---------------------------------------------------------------------
 * PWM page: 4 identical row templates.
 * ------------------------------------------------------------------- */
function pwmRowTemplate(i) {
  return `
    <fieldset class="card pwm-row" data-idx="${i}">
      <legend>PWM ${i}</legend>
      <label class="row"><span>Mode</span>
        <select id="pwm${i}-mode">
          <option value="0">DISABLED</option>
          <option value="1">SERVO</option>
          <option value="2">SWITCH</option>
        </select></label>
      <label class="row"><span>Pin</span>
        <input type="number" id="pwm${i}-pin" min="0" max="39"></label>
      <label class="row"><span>Source channel</span>
        <input type="number" id="pwm${i}-source" min="0" max="15"></label>
      <label class="row"><span>Invert</span>
        <input type="checkbox" id="pwm${i}-invert"></label>
      <label class="row"><span>Failsafe us</span>
        <input type="number" id="pwm${i}-failsafe-us" min="988" max="2012"></label>
      <div class="servo-only">
        <label class="row"><span>Update rate Hz</span>
          <input type="number" id="pwm${i}-rate" min="1" max="400"></label>
      </div>
      <div class="switch-only">
        <label class="row"><span>Switch threshold us</span>
          <input type="number" id="pwm${i}-switch-threshold" min="988" max="2012"></label>
        <label class="row"><span>Switch active-high</span>
          <input type="checkbox" id="pwm${i}-switch-active-high"></label>
      </div>
    </fieldset>`;
}

function buildPwmRows() {
  const container = document.getElementById('pwm-rows');
  let html = '';
  for (let i = 0; i < 4; i++) html += pwmRowTemplate(i);
  container.innerHTML = html;
  for (let i = 0; i < 4; i++) {
    document.getElementById('pwm' + i + '-mode').addEventListener('change', () => updatePwmRowVisibility(i));
  }
}
buildPwmRows();

function updatePwmRowVisibility(i) {
  const mode = parseInt(document.getElementById('pwm' + i + '-mode').value, 10);
  const row = document.querySelector('.pwm-row[data-idx="' + i + '"]');
  const servoOnly = row.querySelector('.servo-only');
  const switchOnly = row.querySelector('.switch-only');
  servoOnly.classList.toggle('hidden', mode !== 1);
  switchOnly.classList.toggle('hidden', mode !== 2);
}

function fillPwmRow(i, cfg) {
  document.getElementById('pwm' + i + '-mode').value = String(cfg.mode);
  document.getElementById('pwm' + i + '-pin').value = cfg.pin;
  document.getElementById('pwm' + i + '-source').value = cfg.source_channel;
  document.getElementById('pwm' + i + '-invert').checked = !!cfg.invert;
  document.getElementById('pwm' + i + '-failsafe-us').value = cfg.failsafe_us;
  document.getElementById('pwm' + i + '-rate').value = cfg.update_rate_hz;
  document.getElementById('pwm' + i + '-switch-threshold').value = cfg.switch_threshold_us;
  document.getElementById('pwm' + i + '-switch-active-high').checked = !!cfg.switch_active_high;
  updatePwmRowVisibility(i);
}

function readPwmRow(i) {
  return {
    mode: parseInt(document.getElementById('pwm' + i + '-mode').value, 10),
    pin: parseInt(document.getElementById('pwm' + i + '-pin').value, 10),
    source_channel: parseInt(document.getElementById('pwm' + i + '-source').value, 10),
    update_rate_hz: parseInt(document.getElementById('pwm' + i + '-rate').value, 10),
    invert: document.getElementById('pwm' + i + '-invert').checked,
    switch_threshold_us: parseInt(document.getElementById('pwm' + i + '-switch-threshold').value, 10),
    switch_active_high: document.getElementById('pwm' + i + '-switch-active-high').checked,
    failsafe_us: parseInt(document.getElementById('pwm' + i + '-failsafe-us').value, 10),
  };
}

async function loadPwm() {
  const r = await api('/api/config/pwm');
  for (let i = 0; i < 4; i++) fillPwmRow(i, r[i]);
}

document.getElementById('pwm-form').addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const body = [];
  for (let i = 0; i < 4; i++) body.push(readPwmRow(i));
  const r = await api('/api/config/pwm', 'POST', body);
  flashSaveStatus('pwm-save-status', r);
});

/* ---------------------------------------------------------------------
 * Voltage page: 2 Hz live ADC + config form + calibrate.
 * ------------------------------------------------------------------- */
let voltageTimer = null;

async function pollVoltage() {
  if (document.hidden) return;
  try {
    const s = await api('/api/status');
    document.getElementById('volt-live-adc-mv').textContent = s.adc_millivolts + ' mV';
    document.getElementById('volt-live-voltage').textContent = s.battery_voltage.toFixed(2) + ' V';
    const cellCount = parseInt(document.getElementById('volt-cell-count').value, 10) || 1;
    document.getElementById('volt-live-per-cell').textContent =
      (s.battery_voltage / cellCount).toFixed(2) + ' V';
  } catch (e) { /* handled */ }
}

function startVoltagePolling() {
  if (voltageTimer) return;
  pollVoltage();
  voltageTimer = setInterval(pollVoltage, 500);
}
startVoltagePolling();

async function loadVoltageConfig() {
  const r = await api('/api/config/voltage');
  document.getElementById('volt-enabled').checked = !!r.enabled;
  document.getElementById('volt-adc-pin').value = r.adc_pin;
  document.getElementById('volt-divider-ratio').value = r.divider_ratio;
  document.getElementById('volt-calibration-factor').value = r.calibration_factor;
  document.getElementById('volt-telemetry-override').checked = !!r.telemetry_override;
  document.getElementById('volt-cell-count').value = r.cell_count;
}

document.getElementById('voltage-form').addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const body = {
    enabled: document.getElementById('volt-enabled').checked,
    adc_pin: parseInt(document.getElementById('volt-adc-pin').value, 10),
    divider_ratio: parseFloat(document.getElementById('volt-divider-ratio').value),
    calibration_factor: parseFloat(document.getElementById('volt-calibration-factor').value),
    telemetry_override: document.getElementById('volt-telemetry-override').checked,
    cell_count: parseInt(document.getElementById('volt-cell-count').value, 10),
  };
  const r = await api('/api/config/voltage', 'POST', body);
  flashSaveStatus('voltage-save-status', r);
});

document.getElementById('volt-calibrate-btn').addEventListener('click', async () => {
  const input = window.prompt('Enter the actual measured battery voltage (V), e.g. 12.60:');
  if (input === null) return;
  const actual = parseFloat(input);
  if (isNaN(actual) || actual <= 0) {
    showError('Invalid voltage entered');
    return;
  }
  const r = await api('/api/voltage/calibrate', 'POST', { actual: actual });
  document.getElementById('volt-calibration-factor').value = r.calibration_factor;
});

/* ---------------------------------------------------------------------
 * Network page.
 * ------------------------------------------------------------------- */
function updateStaticFieldsVisibility() {
  const dhcp = document.getElementById('net-use-dhcp').checked;
  document.getElementById('net-static-fields').classList.toggle('hidden', dhcp);
}
document.getElementById('net-use-dhcp').addEventListener('change', updateStaticFieldsVisibility);

async function loadNetwork() {
  const r = await api('/api/config/network');
  document.getElementById('net-ssid').value = r.ssid;
  document.getElementById('net-password').value = r.password;
  document.getElementById('net-ap-mode').checked = !!r.ap_mode;
  document.getElementById('net-hostname').value = r.hostname;
  document.getElementById('net-use-dhcp').checked = !!r.use_dhcp;
  document.getElementById('net-static-ip').value = ipToString(r.static_ip);
  document.getElementById('net-gateway').value = ipToString(r.gateway);
  document.getElementById('net-netmask').value = ipToString(r.netmask);
  updateStaticFieldsVisibility();
}

document.getElementById('network-form').addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const body = {
    ssid: document.getElementById('net-ssid').value,
    password: document.getElementById('net-password').value,
    ap_mode: document.getElementById('net-ap-mode').checked,
    use_dhcp: document.getElementById('net-use-dhcp').checked,
    static_ip: stringToIp(document.getElementById('net-static-ip').value),
    gateway: stringToIp(document.getElementById('net-gateway').value),
    netmask: stringToIp(document.getElementById('net-netmask').value),
    hostname: document.getElementById('net-hostname').value,
  };
  const r = await api('/api/config/network', 'POST', body);
  flashSaveStatus('network-save-status', r);
  if (r.reboot_required) {
    document.getElementById('network-save-status').textContent += ' (reboot required)';
  }
});

/* ---------------------------------------------------------------------
 * Firmware page: version, restart, factory reset, OTA upload.
 * ------------------------------------------------------------------- */
async function loadFirmware() {
  const s = await api('/api/status');
  document.getElementById('fw-version').textContent = s.firmware_version;
}

document.getElementById('fw-restart-btn').addEventListener('click', async () => {
  if (!window.confirm('Restart the router now? Active RC link will be briefly interrupted.')) return;
  await api('/api/system/restart', 'POST');
});

document.getElementById('fw-factory-reset-btn').addEventListener('click', async () => {
  if (!window.confirm('Factory reset ALL configuration to defaults? This cannot be undone.')) return;
  await api('/api/system/factory-reset', 'POST');
});

document.getElementById('fw-ota-upload-btn').addEventListener('click', () => {
  const fileInput = document.getElementById('fw-ota-file');
  const file = fileInput.files[0];
  if (!file) {
    showError('Choose a .bin file first');
    return;
  }
  const form = new FormData();
  form.append('firmware', file, file.name);

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/ota/upload');
  xhr.upload.addEventListener('progress', (ev) => {
    if (ev.lengthComputable) {
      const pct = Math.round((ev.loaded / ev.total) * 100);
      document.getElementById('fw-ota-progress').value = pct;
      document.getElementById('fw-ota-status').textContent = 'Uploading: ' + pct + '%';
    }
  });
  xhr.addEventListener('load', () => {
    document.getElementById('fw-ota-status').textContent = 'Upload complete, flashing...';
    pollOtaStatus();
  });
  xhr.addEventListener('error', () => {
    showError('OTA upload failed');
  });
  xhr.send(form);
});

let otaPollTimer = null;
function pollOtaStatus() {
  if (otaPollTimer) return;
  otaPollTimer = setInterval(async () => {
    try {
      const st = await api('/api/ota/status');
      document.getElementById('fw-ota-progress').value = st.progress;
      document.getElementById('fw-ota-status').textContent =
        st.state + (st.error ? (': ' + st.error) : '') + ' (' + st.progress + '%)';
      if (st.state === 'success' || st.state === 'failed') {
        clearInterval(otaPollTimer);
        otaPollTimer = null;
      }
    } catch (e) {
      // device likely rebooting after a successful flash; stop polling
      clearInterval(otaPollTimer);
      otaPollTimer = null;
      document.getElementById('fw-ota-status').textContent = 'Device restarting...';
    }
  }, 1000);
}

/* ---------------------------------------------------------------------
 * Misc helpers + boot.
 * ------------------------------------------------------------------- */
function flashSaveStatus(elementId, result) {
  const el = document.getElementById(elementId);
  el.textContent = result && result.ok ? 'Saved' : 'Save failed';
  setTimeout(() => { el.textContent = ''; }, 3000);
}

renderRoute();
startDashboardPolling();
refreshLogs();
