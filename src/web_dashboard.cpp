#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "config.h"
#include "obd_state.h"
#include "web_dashboard.h"
#include "can_recorder.h"
#include "elm327.h"

namespace {

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool mdnsStarted = false;
bool wifiWasConnected = false;
bool wifiReleased = false;

const char kDashboardHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>OBD Dashboard</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; background: #0b1120; color: #e2e8f0; }
    main { max-width: 900px; margin: 0 auto; padding: 20px; }
    h1 { margin: 0 0 4px; font-size: 1.4rem; }
    .meta { color: #94a3b8; margin-bottom: 16px; font-size: 0.85rem; }
    .alert {
      background: #451a1a; border: 1px solid #991b1b; color: #fecaca;
      border-radius: 10px; padding: 12px 14px; margin-bottom: 16px; font-size: 0.9rem;
    }
    .gauges { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 16px; margin-bottom: 20px; }
    .gauge { background: #131c31; border: 1px solid #24304d; border-radius: 16px; padding: 12px; text-align: center; }
    .gauge svg { width: 100%; height: auto; }
    .gauge .gv { font-size: 2.6rem; font-weight: 800; }
    .gauge .gl { color: #94a3b8; font-size: 0.85rem; text-transform: uppercase; letter-spacing: 0.05em; }
    .gauge .gu { color: #64748b; font-size: 0.8rem; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 12px; margin-bottom: 20px; }
    .card { background: #131c31; border: 1px solid #24304d; border-radius: 12px; padding: 12px; }
    .label { color: #94a3b8; font-size: 0.78rem; margin-bottom: 6px; text-transform: uppercase; letter-spacing: 0.04em; }
    .value { font-size: 1.6rem; font-weight: 700; line-height: 1.1; }
    .unit { font-size: 0.85rem; color: #64748b; font-weight: 500; }
    .stale { opacity: 0.35; }
    h2 { font-size: 1rem; color: #cbd5e1; margin: 20px 0 10px; }
    .section { background: #131c31; border: 1px solid #24304d; border-radius: 14px; padding: 16px; margin-bottom: 20px; }
    .row { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
    .spacer { flex: 1; }
    button {
      background: #1e293b; color: #e2e8f0; border: 1px solid #334155;
      border-radius: 8px; padding: 8px 14px; font-size: 0.9rem; cursor: pointer;
    }
    button:hover { background: #263449; }
    button.primary { background: #2563eb; border-color: #2563eb; color: #fff; }
    button.danger { background: #7f1d1d; border-color: #991b1b; color: #fecaca; }
    .peaks { display: grid; grid-template-columns: repeat(auto-fit, minmax(110px, 1fr)); gap: 12px; }
    .dtc-list { margin-top: 12px; display: flex; flex-direction: column; gap: 8px; }
    .dtc { background: #1a1030; border: 1px solid #4c1d95; border-radius: 8px; padding: 10px 12px; font-family: ui-monospace, monospace; font-size: 1rem; }
    .dtc-ok { color: #4ade80; }
    .pid-list { margin-top: 12px; display: grid; grid-template-columns: repeat(auto-fill, minmax(220px, 1fr)); gap: 6px; }
    .pid { background: #0f1830; border: 1px solid #24304d; border-radius: 6px; padding: 6px 10px; font-size: 0.85rem; display: flex; gap: 8px; }
    .pid code { color: #7dd3fc; font-family: ui-monospace, monospace; }
    .pid span { color: #cbd5e1; }
    .pid.known { border-color: #2f6d4a; }
    .ecu-list { margin-top: 12px; display: flex; flex-direction: column; gap: 8px; }
    .ecu {
      background: #0f1830; border: 1px solid #24304d; border-radius: 8px;
      padding: 10px 12px; display: grid; gap: 4px;
    }
    .ecu.present { border-color: #2f6d4a; }
    .ecu.absent { opacity: 0.45; }
    .ecu .eh { display: flex; gap: 8px; align-items: baseline; flex-wrap: wrap; }
    .ecu .ea { color: #7dd3fc; font-family: ui-monospace, monospace; font-size: 0.85rem; }
    .ecu .en { font-weight: 600; }
    .ecu .es { font-size: 0.8rem; color: #4ade80; }
    .ecu.absent .es { color: #64748b; }
    .ecu .meta { font-size: 0.8rem; color: #94a3b8; font-family: ui-monospace, monospace; }
    #ecu-vin { font-size: 1.15rem; word-break: break-all; }
    .did-list { margin-top: 12px; display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 6px; }
    .status { font-size: 0.85rem; color: #94a3b8; }
    .connected { color: #4ade80; }
    .disconnected { color: #f87171; }
    .chart-wrap { display: grid; grid-template-columns: 1fr; gap: 16px; }
    .chart { }
    .chart .ct { color: #94a3b8; font-size: 0.8rem; margin-bottom: 6px; }
    details.nmea summary { cursor: pointer; color: #cbd5e1; font-size: 1rem; list-style: none; display: flex; align-items: center; gap: 8px; }
    details.nmea summary::-webkit-details-marker { display: none; }
    details.nmea summary::before { content: '▸'; color: #64748b; }
    details.nmea[open] summary::before { content: '▾'; }
    details.nmea summary .count { color: #64748b; font-size: 0.8rem; font-weight: 400; }
    .nmea-log {
      max-height: 220px; overflow-y: auto; margin: 12px 0 0; padding: 10px;
      background: #0b1120; border: 1px solid #24304d; border-radius: 8px;
      font-family: ui-monospace, monospace; font-size: 0.72rem; line-height: 1.5;
      color: #7dd3fc; white-space: pre-wrap; word-break: break-all;
    }
    .nmea-empty { color: #64748b; }
  </style>
</head>
<body>
  <main>
    <h1>OBD Dashboard</h1>
    <div class="meta">VW Polo 2018 · NodeMCU-32S live telemetry</div>
    <div class="alert" id="alert" hidden></div>

    <div class="gauges">
      <div class="gauge"><div id="g-rpm"></div></div>
      <div class="gauge"><div id="g-speed"></div></div>
      <div class="gauge"><div id="g-boost"></div></div>
    </div>

    <div class="grid">
      <div class="card" id="coolant-card"><div class="label">Coolant</div><div class="value"><span id="coolant">--</span> <span class="unit">°C</span></div></div>
      <div class="card" id="load-card"><div class="label">Engine load</div><div class="value"><span id="load">--</span> <span class="unit">%</span></div></div>
      <div class="card" id="throttle-card"><div class="label">Throttle</div><div class="value"><span id="throttle">--</span> <span class="unit">%</span></div></div>
      <div class="card" id="intake-card"><div class="label">Intake air</div><div class="value"><span id="intake">--</span> <span class="unit">°C</span></div></div>
      <div class="card" id="maf-card"><div class="label">MAF</div><div class="value"><span id="maf">--</span> <span class="unit">g/s</span></div></div>
      <div class="card" id="timing-card"><div class="label">Timing</div><div class="value"><span id="timing">--</span> <span class="unit">°</span></div></div>
      <div class="card" id="fuel-card"><div class="label">Fuel</div><div class="value"><span id="fuel">--</span> <span class="unit">%</span></div></div>
      <div class="card" id="battery-card"><div class="label">Battery</div><div class="value"><span id="battery">--</span> <span class="unit">V</span></div></div>
    </div>

    <div class="section" id="elm-section">
      <div class="row">
        <h2 style="margin:0">Phone OBD apps (ELM327)</h2>
        <div class="spacer"></div>
        <span class="status" id="elm-status">Bluetooth starting…</span>
      </div>
      <div class="status" style="margin-top:8px">Hotspot on this phone → WiFi adapter at the IP, port 35000. No hotspot at boot → Bluetooth <b>OBDII</b> PIN <b>1234</b> (Android). Never both (one radio).</div>
      <div class="grid" style="margin:12px 0 0">
        <div class="card" id="elm-name-card"><div class="label">Bluetooth name</div><div class="value" id="elm-name">OBDII</div></div>
        <div class="card"><div class="label">PIN</div><div class="value" id="elm-pin">1234</div></div>
        <div class="card" id="elm-ip-card"><div class="label">WiFi adapter</div><div class="value" id="elm-ip">--</div></div>
        <div class="card" id="elm-conn-card"><div class="label">App linked</div><div class="value" id="elm-conn">No</div></div>
      </div>
    </div>

    <div class="section">
      <div class="row">
        <h2 style="margin:0">Charts (last 60s)</h2>
      </div>
      <div class="chart-wrap">
        <div class="chart"><div class="ct">RPM</div><svg id="chart-rpm" viewBox="0 0 600 120" preserveAspectRatio="none" style="width:100%;height:120px"></svg></div>
        <div class="chart"><div class="ct">Speed (km/h)</div><svg id="chart-speed" viewBox="0 0 600 120" preserveAspectRatio="none" style="width:100%;height:120px"></svg></div>
      </div>
    </div>

    <div class="section">
      <div class="row">
        <h2 style="margin:0">Session peaks</h2>
        <div class="spacer"></div>
        <button id="reset-peaks">Reset</button>
      </div>
      <div class="peaks" style="margin-top:12px">
        <div class="card"><div class="label">Max RPM</div><div class="value" id="max-rpm">--</div></div>
        <div class="card"><div class="label">Max speed</div><div class="value"><span id="max-speed">--</span> <span class="unit">km/h</span></div></div>
        <div class="card"><div class="label">Max coolant</div><div class="value"><span id="max-coolant">--</span> <span class="unit">°C</span></div></div>
        <div class="card"><div class="label">Max boost</div><div class="value"><span id="max-boost">--</span> <span class="unit">kPa</span></div></div>
      </div>
    </div>

    <div class="section">
      <div class="row">
        <h2 style="margin:0">Trouble codes</h2>
        <div class="spacer"></div>
        <button class="primary" id="read-dtc">Read codes</button>
        <button class="danger" id="clear-dtc">Clear codes</button>
      </div>
      <div class="dtc-list" id="dtc-list"><div class="status">Press "Read codes" to scan.</div></div>
    </div>

    <div class="section">
      <div class="row">
        <h2 style="margin:0">ABS / ESP (traction control)</h2>
        <div class="spacer"></div>
        <button class="primary" id="read-abs">Read codes</button>
        <button class="danger" id="clear-abs">Clear codes</button>
      </div>
      <div class="status" style="margin-top:8px">Chassis "C" codes from the ABS/ESP module (VW-specific, UDS). Won't clear if the fault is currently active.</div>
      <div class="dtc-list" id="abs-list"><div class="status">Press "Read codes" to scan the ABS module.</div></div>
    </div>

    <div class="section">
      <div class="row">
        <h2 style="margin:0">Airbag / SRS</h2>
        <div class="spacer"></div>
        <button class="primary" id="read-airbag">Read codes</button>
        <button class="danger" id="clear-airbag">Clear codes</button>
      </div>
      <div class="alert" style="margin:8px 0 0">⚠ Safety-critical. Reading is safe. Clearing only helps for soft/stored codes and will NOT clear an active fault or locked crash data — never rely on a cleared code to mean the airbag is functional. Disconnect the battery before working on SRS wiring.</div>
      <div class="dtc-list" id="airbag-list"><div class="status">Press "Read codes" to scan the airbag module.</div></div>
    </div>

    <div class="section">
      <div class="row">
        <h2 style="margin:0">UDS ECU scan</h2>
        <span id="ecu-count" class="status"></span>
        <div class="spacer"></div>
        <button class="primary" id="scan-ecus">Scan ECUs</button>
      </div>
      <div class="status" style="margin-top:8px">Read-only. Probes each VAG module through the gateway, then dumps engine identification DIDs (0xF180–0xF1FF). No coding, no security access, no writes.</div>
      <div class="grid" style="margin:12px 0 0">
        <div class="card"><div class="label">VIN</div><div class="value" id="ecu-vin">--</div></div>
      </div>
      <div class="ecu-list" id="ecu-list"><div class="status">Press "Scan ECUs" with ignition ON.</div></div>
      <div class="did-list" id="did-list"></div>
    </div>

    <div class="section">
      <div class="row">
        <h2 style="margin:0">Supported PIDs</h2>
        <span id="pid-count" class="status"></span>
        <div class="spacer"></div>
        <button class="primary" id="scan-pids">Scan vehicle</button>
      </div>
      <div class="pid-list" id="pid-list"><div class="status">Press "Scan vehicle" to list every PID the car supports.</div></div>
    </div>

    <div class="section">
      <div class="row">
        <h2 style="margin:0">CAN recorder</h2>
        <div class="spacer"></div>
        <button class="primary" id="rec-toggle">Start recording</button>
        <button id="rec-download">Download .log</button>
      </div>
      <details class="nmea" id="can-details" style="margin-top:12px">
        <summary>Raw CAN frames <span class="count" id="can-count"></span></summary>
        <div class="nmea-log" id="can-log"><span class="nmea-empty">Not recording. Press "Start recording" to capture raw frames.</span></div>
      </details>
    </div>

    <div class="status disconnected" id="status">Connecting...</div>
  </main>

  <script>
    const RPM_MAX = 7000, SPEED_MAX = 220, BOOST_MIN = -100, BOOST_MAX = 150;
    const rpmHist = [], speedHist = [], HIST = 120;

    function arcGauge(elId, value, max, valueText, label, unit, color) {
      const frac = Math.max(0, Math.min(1, value / max));
      const R = 80, CX = 100, CY = 100, START = 135, SWEEP = 270;
      function pt(angDeg, r) {
        const a = (angDeg) * Math.PI / 180;
        return [CX + r * Math.cos(a), CY + r * Math.sin(a)];
      }
      function arcPath(fromFrac, toFrac) {
        const a0 = START + SWEEP * fromFrac, a1 = START + SWEEP * toFrac;
        const [x0, y0] = pt(a0, R), [x1, y1] = pt(a1, R);
        const large = (a1 - a0) > 180 ? 1 : 0;
        return `M ${x0} ${y0} A ${R} ${R} 0 ${large} 1 ${x1} ${y1}`;
      }
      document.getElementById(elId).innerHTML =
        `<svg viewBox="0 0 200 175">
          <path d="${arcPath(0,1)}" fill="none" stroke="#24304d" stroke-width="14" stroke-linecap="round"/>
          <path d="${arcPath(0,frac)}" fill="none" stroke="${color}" stroke-width="14" stroke-linecap="round"/>
          <text x="100" y="98" text-anchor="middle" class="gv" fill="#f1f5f9" font-size="34" font-weight="800">${valueText}</text>
          <text x="100" y="122" text-anchor="middle" fill="#64748b" font-size="13">${unit}</text>
          <text x="100" y="165" text-anchor="middle" fill="#94a3b8" font-size="14" style="text-transform:uppercase">${label}</text>
        </svg>`;
    }

    function drawChart(svgId, data, maxHint, color) {
      const svg = document.getElementById(svgId);
      if (!data.length) { svg.innerHTML = ''; return; }
      const W = 600, H = 120, pad = 6;
      const max = Math.max(maxHint, ...data) || 1;
      const step = W / Math.max(1, HIST - 1);
      let d = '';
      data.forEach((v, i) => {
        const x = i * step;
        const y = H - pad - (v / max) * (H - 2 * pad);
        d += (i === 0 ? 'M' : 'L') + x.toFixed(1) + ' ' + y.toFixed(1) + ' ';
      });
      svg.innerHTML = `<polyline points="" fill="none"/>` +
        `<path d="${d}" fill="none" stroke="${color}" stroke-width="2.5"/>`;
    }

    function setText(id, text, cardId, valid) {
      document.getElementById(id).textContent = text;
      if (cardId) document.getElementById(cardId).classList.toggle('stale', !valid);
    }

    function render(data) {
      arcGauge('g-rpm', data.rpm_valid ? data.rpm : 0, RPM_MAX,
        data.rpm_valid ? Math.round(data.rpm) : '--', 'RPM', 'rev/min', '#38bdf8');
      arcGauge('g-speed', data.speed_valid ? data.speed_kmh : 0, SPEED_MAX,
        data.speed_valid ? data.speed_kmh : '--', 'Speed', 'km/h', '#4ade80');
      const boostShown = data.boost_valid ? data.boost_kpa : 0;
      arcGauge('g-boost', boostShown - BOOST_MIN, BOOST_MAX - BOOST_MIN,
        data.boost_valid ? data.boost_kpa : '--', 'Boost', 'kPa', '#f59e0b');

      setText('coolant', data.coolant_valid ? data.coolant_c : '--', 'coolant-card', data.coolant_valid);
      setText('load', data.load_valid ? data.engine_load_pct.toFixed(0) : '--', 'load-card', data.load_valid);
      setText('throttle', data.throttle_valid ? data.throttle_pct.toFixed(0) : '--', 'throttle-card', data.throttle_valid);
      setText('intake', data.intake_valid ? data.intake_air_c : '--', 'intake-card', data.intake_valid);
      setText('maf', data.maf_valid ? data.maf_gs.toFixed(1) : '--', 'maf-card', data.maf_valid);
      setText('timing', data.timing_valid ? data.timing_adv.toFixed(1) : '--', 'timing-card', data.timing_valid);
      setText('fuel', data.fuel_valid ? data.fuel_pct.toFixed(0) : '--', 'fuel-card', data.fuel_valid);
      setText('battery', data.battery_valid ? data.battery_v.toFixed(1) : '--', 'battery-card', data.battery_valid);

      setText('max-rpm', Math.round(data.max_rpm));
      setText('max-speed', data.max_speed_kmh);
      setText('max-coolant', data.max_coolant_c);
      setText('max-boost', data.max_boost_kpa);

      if (data.rpm_valid) { rpmHist.push(data.rpm); if (rpmHist.length > HIST) rpmHist.shift(); }
      if (data.speed_valid) { speedHist.push(data.speed_kmh); if (speedHist.length > HIST) speedHist.shift(); }
      drawChart('chart-rpm', rpmHist, RPM_MAX, '#38bdf8');
      drawChart('chart-speed', speedHist, SPEED_MAX, '#4ade80');

      renderElm(data);
      renderDtc(data);
      renderEcus(data);
      renderPids(data);

      const alert = document.getElementById('alert');
      if (data.elm_connected) {
        alert.hidden = false;
        alert.textContent = 'Phone OBD app owns the CAN bus — dashboard polling is paused.';
      } else if (data.can_ok && data.rpm_valid) {
        alert.hidden = true;
      } else {
        alert.hidden = false;
        alert.textContent = data.can_message || 'Waiting for OBD data';
      }

      const status = document.getElementById('status');
      status.textContent = data.age_ms > 0 && data.age_ms < 5000
        ? 'Live · updated ' + (Math.round(data.age_ms / 100) / 10) + 's ago'
        : (data.can_message || 'Connected · waiting for OBD data');
      status.className = 'status connected';
    }

    function renderElm(data) {
      const bt = data.elm_transport === 'bluetooth';
      setText('elm-name', data.elm_bt_name || 'OBDII', 'elm-name-card', bt);
      setText('elm-pin', data.elm_bt_pin || '1234');
      const ip = data.ip || '';
      setText('elm-ip', (!bt && ip && ip !== '0.0.0.0') ? (ip + ':' + (data.elm_port || 35000)) : '--',
        'elm-ip-card', !bt && !!ip && ip !== '0.0.0.0');
      const linked = !!data.elm_connected;
      setText('elm-conn', linked ? 'Yes' : 'No', 'elm-conn-card', linked);
      const st = document.getElementById('elm-status');
      if (linked) {
        st.textContent = 'App connected';
        st.className = 'status connected';
      } else if (bt) {
        st.textContent = 'Bluetooth OBDII PIN ' + (data.elm_bt_pin || '1234');
        st.className = 'status connected';
      } else if (ip && ip !== '0.0.0.0') {
        st.textContent = 'WiFi ' + ip + ':' + (data.elm_port || 35000);
        st.className = 'status connected';
      } else {
        st.textContent = 'Waiting for hotspot…';
        st.className = 'status disconnected';
      }
    }

    const PID_NAMES = {
      0x01: 'Monitor status since DTCs cleared', 0x03: 'Fuel system status', 0x04: 'Calculated engine load',
      0x05: 'Coolant temperature', 0x06: 'Short term fuel trim B1', 0x07: 'Long term fuel trim B1',
      0x08: 'Short term fuel trim B2', 0x09: 'Long term fuel trim B2', 0x0A: 'Fuel pressure',
      0x0B: 'Intake manifold pressure', 0x0C: 'Engine RPM', 0x0D: 'Vehicle speed', 0x0E: 'Timing advance',
      0x0F: 'Intake air temperature', 0x10: 'MAF air flow rate', 0x11: 'Throttle position',
      0x12: 'Commanded secondary air', 0x13: 'O2 sensors present', 0x14: 'O2 sensor 1', 0x15: 'O2 sensor 2',
      0x1C: 'OBD standard', 0x1F: 'Run time since start', 0x21: 'Distance with MIL on',
      0x22: 'Fuel rail pressure', 0x23: 'Fuel rail gauge pressure', 0x2C: 'Commanded EGR', 0x2D: 'EGR error',
      0x2E: 'Commanded evap purge', 0x2F: 'Fuel tank level', 0x30: 'Warm-ups since cleared',
      0x31: 'Distance since cleared', 0x32: 'Evap system vapor pressure', 0x33: 'Barometric pressure',
      0x3C: 'Catalyst temp B1S1', 0x40: 'PIDs supported 41-60', 0x42: 'Control module voltage',
      0x43: 'Absolute load value', 0x44: 'Commanded equivalence ratio', 0x45: 'Relative throttle position',
      0x46: 'Ambient air temperature', 0x47: 'Absolute throttle B', 0x49: 'Accelerator pedal D',
      0x4A: 'Accelerator pedal E', 0x4C: 'Commanded throttle actuator', 0x4D: 'Time with MIL on',
      0x51: 'Fuel type', 0x5C: 'Engine oil temperature', 0x5E: 'Engine fuel rate',
      0x20: 'PIDs supported 21-40', 0x60: 'PIDs supported 61-80', 0x80: 'PIDs supported 81-A0',
      0xA0: 'PIDs supported A1-C0', 0xC0: 'PIDs supported C1-E0', 0x00: 'PIDs supported 01-20'
    };
    function hex2(n) { return '0x' + n.toString(16).toUpperCase().padStart(2, '0'); }
    function hex4(n) { return n.toString(16).toUpperCase().padStart(4, '0'); }

    const DID_NAMES = {
      0xF180: 'Boot software ident', 0xF181: 'Application software ident',
      0xF182: 'Application data ident', 0xF186: 'Active diagnostic session',
      0xF187: 'Spare part number', 0xF188: 'VW ECU ident', 0xF189: 'Software version',
      0xF18A: 'System supplier ident', 0xF18C: 'ECU serial number', 0xF190: 'VIN',
      0xF191: 'Hardware number', 0xF194: 'Hardware version', 0xF195: 'Software version (alt)',
      0xF197: 'System name', 0xF19E: 'ASAM/ODX file ident', 0xF1A0: 'Workshop system name',
      0xF1A2: 'ASAM/ODX file version', 0xF1AA: 'Workshop system name', 0xF1DF: 'Engine/serial ident'
    };

    function renderEcus(data) {
      const list = document.getElementById('ecu-list');
      const count = document.getElementById('ecu-count');
      const vinEl = document.getElementById('ecu-vin');
      const didList = document.getElementById('did-list');
      vinEl.textContent = data.vin || '--';
      if (data.ecu_scan_status === 'idle') return;
      if (data.ecu_scan_status === 'scanning') {
        count.textContent = data.ecu_scan_label || 'Scanning…';
      } else if (data.ecu_scan_status === 'error') {
        count.textContent = 'Scan failed';
      } else {
        count.textContent = (data.ecu_scan_found || 0) + ' present';
      }
      const ecus = data.ecus || [];
      if (!ecus.length) {
        list.innerHTML = '<div class="status">Scanning…</div>';
      } else {
        list.innerHTML = ecus.map(e => {
          const state = !e.probed ? 'pending' : (e.present ? 'present' : 'absent');
          const badge = !e.probed ? '…' : (e.present ? 'present' : 'absent');
          const bits = [];
          if (e.part) bits.push(e.part);
          if (e.sw) bits.push('SW ' + e.sw);
          if (e.hw) bits.push('HW ' + e.hw);
          if (e.sys) bits.push(e.sys);
          bits.push(e.req + '/' + e.resp);
          if (e.present && e.dtc_count) bits.push(e.dtc_count + ' DTC');
          const dtcs = (e.present && e.dtcs && e.dtcs.length)
            ? e.dtcs.map(c => '<div class="dtc">' + c + '</div>').join('') : '';
          return '<div class="ecu ' + state + '"><div class="eh"><span class="ea">' + e.addr +
            '</span><span class="en">' + e.name + '</span><span class="es">' + badge +
            '</span></div><div class="meta">' + bits.join(' · ') + '</div>' + dtcs + '</div>';
        }).join('');
      }
      const dids = data.engine_dids || [];
      if (!dids.length) {
        didList.innerHTML = (data.ecu_scan_status === 'done')
          ? '<div class="status">No engine identification DIDs returned.</div>' : '';
        return;
      }
      didList.innerHTML = dids.map(d => {
        const name = DID_NAMES[d.id] || 'Data identifier';
        return '<div class="pid known"><code>' + hex4(d.id) + '</code><span>' + name +
          ' — ' + (d.value || '') + '</span></div>';
      }).join('');
    }

    function renderPids(data) {
      const list = document.getElementById('pid-list');
      const count = document.getElementById('pid-count');
      if (data.pid_scan_status === 'scanning') { list.innerHTML = '<div class="status">Scanning ECU…</div>'; return; }
      if (data.pid_scan_status === 'error') { list.innerHTML = '<div class="status">Scan failed / no response.</div>'; return; }
      if (data.pid_scan_status !== 'done') return;
      const pids = data.supported_pids || [];
      count.textContent = pids.length + ' supported';
      if (!pids.length) { list.innerHTML = '<div class="status">No PIDs reported.</div>'; return; }
      list.innerHTML = pids.map(p => {
        const name = PID_NAMES[p];
        return '<div class="pid' + (name ? ' known' : '') + '"><code>' + hex2(p) + '</code><span>' + (name || 'Unknown / manufacturer') + '</span></div>';
      }).join('');
    }

    function renderCodes(listId, status, codes) {
      const list = document.getElementById(listId);
      if (status === 'reading') {
        list.innerHTML = '<div class="status">Scanning…</div>';
      } else if (status === 'cleared') {
        list.innerHTML = '<div class="status dtc-ok">Codes cleared. Read again to confirm.</div>';
      } else if (status === 'error') {
        list.innerHTML = '<div class="status">No response / not supported.</div>';
      } else if (status === 'done') {
        list.innerHTML = (!codes || !codes.length)
          ? '<div class="status dtc-ok">No stored trouble codes.</div>'
          : codes.map(c => '<div class="dtc">' + c + '</div>').join('');
      }
    }

    function renderDtc(data) {
      renderCodes('dtc-list', data.dtc_status, data.dtcs);
      renderCodes('abs-list', data.abs_status, data.abs_dtcs);
      renderCodes('airbag-list', data.airbag_status, data.airbag_dtcs);
    }

    const canLines = [], CAN_VIEW_MAX = 300;
    let canRecording = false;
    function appendCan(line) {
      canLines.push(line);
      document.getElementById('can-count').textContent = '(' + canLines.length + ')';
      const log = document.getElementById('can-log');
      const view = canLines.length > CAN_VIEW_MAX ? canLines.slice(-CAN_VIEW_MAX) : canLines;
      const atBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 30;
      log.textContent = view.join('\n');
      if (atBottom) log.scrollTop = log.scrollHeight;
    }
    function updateRecUi() {
      const b = document.getElementById('rec-toggle');
      b.textContent = canRecording ? 'Stop recording' : 'Start recording';
      b.classList.toggle('danger', canRecording);
      b.classList.toggle('primary', !canRecording);
    }

    let socket, reconnectTimer;
    function send(cmd) { if (socket && socket.readyState === 1) socket.send(cmd); }

    document.getElementById('reset-peaks').onclick = () => send('reset_peaks');
    document.getElementById('scan-pids').onclick = () => send('scan_pids');
    document.getElementById('scan-ecus').onclick = () => send('scan_ecus');
    document.getElementById('read-dtc').onclick = () => send('read_dtc');
    document.getElementById('clear-dtc').onclick = () => {
      if (confirm('Clear all stored trouble codes and turn off the check-engine light?')) send('clear_dtc');
    };
    document.getElementById('read-abs').onclick = () => send('read_abs');
    document.getElementById('clear-abs').onclick = () => {
      if (confirm('Clear stored ABS/ESP codes? This will not help if the fault is currently active.')) send('clear_abs');
    };
    document.getElementById('read-airbag').onclick = () => send('read_airbag');
    document.getElementById('clear-airbag').onclick = () => {
      if (confirm('SAFETY: Clearing airbag/SRS codes will NOT fix an active fault or crash data, and a cleared code does not mean the airbag will deploy. Only proceed for a known soft code. Continue?')) send('clear_airbag');
    };
    document.getElementById('rec-toggle').onclick = () => {
      canRecording = !canRecording;
      if (canRecording) {
        canLines.length = 0;
        document.getElementById('can-count').textContent = '';
        document.getElementById('can-log').textContent = '';
        document.getElementById('can-details').open = true;
      }
      send(canRecording ? 'rec_start' : 'rec_stop');
      updateRecUi();
    };
    document.getElementById('rec-download').onclick = () => {
      if (!canLines.length) { alert('No frames captured yet.'); return; }
      const blob = new Blob([canLines.join('\n') + '\n'], { type: 'text/plain' });
      const a = document.createElement('a');
      const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
      a.href = URL.createObjectURL(blob);
      a.download = 'capture-' + ts + '.log';
      a.click();
      URL.revokeObjectURL(a.href);
    };

    function connect() {
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
      socket = new WebSocket(protocol + '//' + location.host + '/ws');
      socket.onopen = () => {
        document.getElementById('status').textContent = 'Connected';
        document.getElementById('status').className = 'status connected';
      };
      socket.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.nmea !== undefined) { return; }
        if (data.can !== undefined) { appendCan(data.can); return; }
        if (typeof data.rec_active === 'boolean' && data.rec_active !== canRecording) {
          canRecording = data.rec_active;
          updateRecUi();
        }
        render(data);
      };
      socket.onclose = () => {
        document.getElementById('status').textContent = 'Disconnected — reconnecting...';
        document.getElementById('status').className = 'status disconnected';
        clearTimeout(reconnectTimer);
        reconnectTimer = setTimeout(connect, 1000);
      };
    }
    connect();
  </script>
</body>
</html>
)rawliteral";

String boolStr(bool v) {
  return v ? "true" : "false";
}

String buildObdJson() {
  const uint32_t ageMs =
      gObdState.lastUpdateMs == 0 ? 0 : millis() - gObdState.lastUpdateMs;

  String json = "{";
  json += "\"rpm\":" + String(gObdState.rpm, 1) + ",";
  json += "\"speed_kmh\":" + String(gObdState.speedKmh) + ",";
  json += "\"coolant_c\":" + String(gObdState.coolantC) + ",";
  json += "\"engine_load_pct\":" + String(gObdState.engineLoadPct, 1) + ",";
  json += "\"throttle_pct\":" + String(gObdState.throttlePct, 1) + ",";
  json += "\"intake_air_c\":" + String(gObdState.intakeAirC) + ",";
  json += "\"maf_gs\":" + String(gObdState.mafGs, 2) + ",";
  json += "\"timing_adv\":" + String(gObdState.timingAdv, 1) + ",";
  json += "\"fuel_pct\":" + String(gObdState.fuelPct, 1) + ",";
  json += "\"battery_v\":" + String(gObdState.batteryV, 2) + ",";
  json += "\"map_kpa\":" + String(gObdState.mapKpa) + ",";
  json += "\"boost_kpa\":" + String(gObdState.boostKpa) + ",";

  json += "\"rpm_valid\":" + boolStr(gObdState.rpmValid) + ",";
  json += "\"speed_valid\":" + boolStr(gObdState.speedValid) + ",";
  json += "\"coolant_valid\":" + boolStr(gObdState.coolantValid) + ",";
  json += "\"load_valid\":" + boolStr(gObdState.engineLoadValid) + ",";
  json += "\"throttle_valid\":" + boolStr(gObdState.throttleValid) + ",";
  json += "\"intake_valid\":" + boolStr(gObdState.intakeAirValid) + ",";
  json += "\"maf_valid\":" + boolStr(gObdState.mafValid) + ",";
  json += "\"timing_valid\":" + boolStr(gObdState.timingValid) + ",";
  json += "\"fuel_valid\":" + boolStr(gObdState.fuelValid) + ",";
  json += "\"battery_valid\":" + boolStr(gObdState.batteryValid) + ",";
  json += "\"boost_valid\":" + boolStr(gObdState.boostValid) + ",";

  json += "\"max_rpm\":" + String(gObdState.maxRpm, 0) + ",";
  json += "\"max_speed_kmh\":" + String(gObdState.maxSpeedKmh) + ",";
  json += "\"max_coolant_c\":" + String(gObdState.maxCoolantC) + ",";
  json += "\"max_boost_kpa\":" + String(gObdState.maxBoostKpa) + ",";

  json += "\"dtc_status\":\"" + String(dtcStatusName()) + "\",";
  json += "\"dtcs\":[";
  for (uint8_t i = 0; i < gObdState.dtcCount; i++) {
    if (i > 0) json += ",";
    json += "\"" + String(gObdState.dtcCodes[i]) + "\"";
  }
  json += "],";

  for (uint8_t m = 0; m < UDS_MODULE_COUNT; m++) {
    const UdsModuleState &mod = gObdState.udsModules[m];
    const String key = kUdsModules[m].key;
    json += "\"" + key + "_status\":\"" + String(dtcStatusText(mod.status)) + "\",";
    json += "\"" + key + "_dtcs\":[";
    for (uint8_t i = 0; i < mod.dtcCount; i++) {
      if (i > 0) json += ",";
      json += "\"" + String(mod.dtcCodes[i]) + "\"";
    }
    json += "],";
  }

  json += "\"pid_scan_status\":\"" + String(scanStatusName()) + "\",";
  json += "\"supported_count\":" + String(gObdState.supportedCount) + ",";
  json += "\"supported_pids\":[";
  for (uint8_t i = 0; i < gObdState.supportedCount; i++) {
    if (i > 0) json += ",";
    json += String(gObdState.supportedPids[i]);
  }
  json += "],";

  json += "\"ecu_scan_status\":\"" + String(ecuScanStatusName()) + "\",";
  json += "\"ecu_scan_label\":\"" + String(gObdState.ecuScanLabel) + "\",";
  json += "\"ecu_scan_found\":" + String(gObdState.ecuScanFound) + ",";
  json += "\"vin\":\"" + String(gObdState.vin) + "\",";
  json += "\"ecus\":[";
  for (uint8_t i = 0; i < UDS_MODULE_COUNT; i++) {
    const UdsModuleConfig &cfg = kUdsModules[i];
    const UdsModuleState &mod = gObdState.udsModules[i];
    if (i > 0) json += ",";
    char addr[3], req[8], resp[8];
    snprintf(addr, sizeof(addr), "%02X", cfg.vagAddr);
    snprintf(req, sizeof(req), "%03lX", cfg.reqId);
    snprintf(resp, sizeof(resp), "%03lX", cfg.respId);
    json += "{\"key\":\"" + String(cfg.key) + "\",";
    json += "\"addr\":\"" + String(addr) + "\",";
    json += "\"name\":\"" + String(cfg.name) + "\",";
    json += "\"req\":\"" + String(req) + "\",";
    json += "\"resp\":\"" + String(resp) + "\",";
    json += "\"probed\":" + boolStr(mod.probed) + ",";
    json += "\"present\":" + boolStr(mod.present) + ",";
    json += "\"part\":\"" + String(mod.partNumber) + "\",";
    json += "\"sw\":\"" + String(mod.swVersion) + "\",";
    json += "\"hw\":\"" + String(mod.hwNumber) + "\",";
    json += "\"sys\":\"" + String(mod.sysName) + "\",";
    json += "\"dtc_count\":" + String(mod.dtcCount) + ",";
    json += "\"dtcs\":[";
    for (uint8_t d = 0; d < mod.dtcCount; d++) {
      if (d > 0) json += ",";
      json += "\"" + String(mod.dtcCodes[d]) + "\"";
    }
    json += "]}";
  }
  json += "],";
  json += "\"engine_dids\":[";
  for (uint8_t i = 0; i < gObdState.engineDidCount; i++) {
    if (i > 0) json += ",";
    json += "{\"id\":" + String(gObdState.engineDids[i].id) + ",";
    json += "\"value\":\"" + String(gObdState.engineDids[i].value) + "\"}";
  }
  json += "],";

  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"elm_bt_name\":\"" + String(ELM327_BT_NAME) + "\",";
  json += "\"elm_bt_pin\":\"" + String(ELM327_BT_PIN) + "\",";
  json += "\"elm_port\":" + String(ELM327_TCP_PORT) + ",";
  json += "\"elm_transport\":\"" + String(elm327UsingBluetooth() ? "bluetooth" : "wifi") + "\",";
  json += "\"elm_connected\":" + boolStr(elm327ClientConnected()) + ",";

  json += "\"gps_valid\":" + boolStr(gObdState.gpsValid) + ",";
  json += "\"lat\":" + String(gObdState.latitude, 6) + ",";
  json += "\"lon\":" + String(gObdState.longitude, 6) + ",";
  json += "\"gps_speed_kmh\":" + String(gObdState.gpsSpeedKmh, 1) + ",";
  json += "\"gps_alt_m\":" + String(gObdState.altitudeM, 0) + ",";
  json += "\"gps_heading\":" + String(gObdState.headingDeg, 0) + ",";
  json += "\"gps_sats\":" + String(gObdState.satellites) + ",";

  json += "\"can_ok\":" + boolStr(gObdState.canReady && gObdState.lastCanError == 0) + ",";
  json += "\"bus_active\":" + boolStr(gObdState.busActive) + ",";
  json += "\"can_message\":\"" + String(canStatusMessage()) + "\",";
  json += "\"rec_active\":" + boolStr(canRecordActive()) + ",";
  json += "\"rec_count\":" + String(canRecordCount()) + ",";
  json += "\"age_ms\":" + String(ageMs);
  json += "}";

  return json;
}

void handleWsCommand(const String &cmd) {
  if (cmd == "reset_peaks") {
    gObdState.cmdResetPeaks = true;
  } else if (cmd == "read_dtc") {
    gObdState.cmdReadDtc = true;
  } else if (cmd == "clear_dtc") {
    gObdState.cmdClearDtc = true;
  } else if (cmd == "scan_pids") {
    gObdState.cmdScanPids = true;
  } else if (cmd == "scan_ecus") {
    if (gObdState.ecuScanStatus != SCAN_RUNNING) {
      gObdState.cmdScanEcus = true;
    }
  } else if (cmd == "rec_start") {
    canRecordStart();
  } else if (cmd == "rec_stop") {
    canRecordStop();
  } else {
    // UDS module commands: read_<key> / clear_<key> (e.g. read_abs, clear_airbag)
    for (uint8_t i = 0; i < UDS_MODULE_COUNT; i++) {
      const String key = kUdsModules[i].key;
      if (cmd == "read_" + key) {
        gObdState.udsModules[i].cmdRead = true;
        return;
      }
      if (cmd == "clear_" + key) {
        gObdState.udsModules[i].cmdClear = true;
        return;
      }
    }
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      client->text(buildObdJson());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo *info = static_cast<AwsFrameInfo *>(arg);
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        String msg;
        msg.reserve(len);
        for (size_t i = 0; i < len; i++) {
          msg += static_cast<char>(data[i]);
        }
        handleWsCommand(msg);
      }
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
    case WS_EVT_DISCONNECT:
      break;
  }
}

void startMdns() {
  if (mdnsStarted || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("obd", "tcp", ELM327_TCP_PORT);
    mdnsStarted = true;
    Serial.printf("Dashboard shortcut: http://%s.local\n", MDNS_HOSTNAME);
  }
}

void onWiFiConnected() {
  Serial.print("Phone hotspot connected, dashboard at http://");
  Serial.print(WiFi.localIP());
  Serial.printf(" or http://%s.local\n", MDNS_HOSTNAME);
  elm327OnWifiUp();
  startMdns();
}

void maintainWiFi() {
  if (wifiReleased) {
    return;
  }
  const wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (!wifiWasConnected) {
      onWiFiConnected();
    }
    wifiWasConnected = true;
    return;
  }

  if (wifiWasConnected) {
    Serial.println("Phone hotspot lost — auto-reconnect (not calling WiFi.begin again)");
    wifiWasConnected = false;
    mdnsStarted = false;
  }
  // setAutoReconnect(true) already retries. Calling WiFi.begin() while STA is
  // connecting overflows the WiFi event queue and panics Bluedroid
  // (hash_map_set data != NULL) when Classic BT is also running.
}

}  // namespace

void initWebDashboard() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("Joining phone hotspot \"%s\" (non-blocking)...\n", WIFI_SSID);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", kDashboardHtml);
  });

  server.begin();

  Serial.println("Web dashboard ready (WebSocket: /ws)");
}

void handleWebDashboard() {
  if (wifiReleased) {
    return;
  }
  maintainWiFi();
  ws.cleanupClients();
}

bool waitForHotspot(uint32_t timeoutMs) {
  const uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(deadline - millis()) > 0) {
    handleWebDashboard();
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(50);
  }
  return WiFi.status() == WL_CONNECTED;
}

void stopWifiRadio() {
  wifiReleased = true;
  WiFi.setAutoReconnect(false);
  ws.closeAll();
  server.end();
  WiFi.disconnect(true, false);
  delay(50);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi off — radio free for Bluetooth serial");
}

void broadcastObdState() {
  if (ws.count() == 0) {
    return;
  }

  const String json = buildObdJson();
  ws.textAll(json);
}

void broadcastNmea(const String &line) {
  if (ws.count() == 0 || line.length() == 0) {
    return;
  }

  String json = "{\"nmea\":\"";
  for (size_t i = 0; i < line.length(); i++) {
    const char c = line[i];
    if (c == '"' || c == '\\') {
      json += '\\';
    }
    json += c;
  }
  json += "\"}";
  ws.textAll(json);
}

void broadcastCanFrame(const String &line) {
  if (ws.count() == 0 || line.length() == 0) {
    return;
  }

  // candump lines contain only ASCII digits, letters, and "().#-" - no JSON
  // escaping needed.
  String json = "{\"can\":\"";
  json += line;
  json += "\"}";
  ws.textAll(json);
}
