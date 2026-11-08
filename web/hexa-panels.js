// hexa-panels.js -- a "Hexapod" tab for linorobot2-cockpit's web UI: the miniHexa's camera, battery and sonar.
//
// Loaded by one <script> line placed BEFORE app-core.js in cockpit's index.html (web/install_panels.sh does
// that): this file adds its tab button and pane synchronously, so cockpit's own tab switching binds them like
// its own tabs. Nothing in cockpit is changed otherwise.
//
// Battery and sonar come over rosbridge (port 9090 on the page's host, like the Map Viewer): /battery
// (sensor_msgs/BatteryState) and /sonar (sensor_msgs/Range). This tab keeps its own socket, open only while
// the tab is shown, so it never touches the Map Viewer's subscriptions.
//
// The camera is a separate board serving HTTP: /capture (one JPEG, server on port 80) and :81/stream (MJPEG,
// ONE client at a time: a page holding it locks out camera_bridge and every other reader). The panel polls
// /capture by default; "Live stream" takes the stream slot until the tab is left or Stop is pressed.
// The camera's address: ?cam=<url> on the page, else the last one typed here, else window.HEXA_CAMERA_URL
// (a deployment may set it in hexa-site.js), else empty.
(() => {
  "use strict";

  // A 2S Li-ion pack. Under 7.0 V the walk should end; 6.8 V is the floor at rest.
  const BAT_FULL = 8.4, BAT_WARN = 7.0, BAT_FLOOR = 6.8;
  const HISTORY_S = 300;           // the charts' span
  const STALE_MS = 3000;           // no message for this long: the value is shown as stale
  const CAM_KEY = "hexa-panels.camera-url";

  // ------------------------------------------------------------------ the tab
  const nav = document.querySelector("nav.tabs");
  const main = document.querySelector("main.tab-content");
  if (!nav || !main) return;

  const btn = document.createElement("button");
  btn.className = "tab-btn tab-tool";
  btn.dataset.tab = "hexapod";
  btn.textContent = "Hexapod";
  const sep = nav.querySelector(".tab-sep");
  if (sep) sep.after(btn); else nav.appendChild(btn);

  const style = document.createElement("style");
  style.textContent = `
    #tab-hexapod .hx-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 14px; }
    #tab-hexapod .hx-cam { grid-column: 1 / -1; }
    #tab-hexapod .hx-big { font-size: 34px; font-weight: 650; font-variant-numeric: tabular-nums; line-height: 1.1; }
    #tab-hexapod .hx-unit { font-size: 15px; color: var(--text-dim); font-weight: 500; margin-left: 4px; }
    #tab-hexapod .hx-sub { color: var(--text-dim); font-size: 12.5px; font-variant-numeric: tabular-nums; margin-top: 4px; }
    #tab-hexapod .hx-ok { color: var(--accent-ok); }
    #tab-hexapod .hx-warn { color: var(--accent-warn); }
    #tab-hexapod .hx-bad { color: var(--accent-danger); }
    #tab-hexapod .hx-stale { opacity: 0.45; }
    #tab-hexapod canvas.hx-chart { width: 100%; height: 110px; display: block; margin-top: 10px;
      background: #ffffff06; border: 1px solid var(--border-soft); border-radius: 6px; }
    #tab-hexapod .hx-bar { height: 10px; border-radius: 5px; background: #ffffff10; margin-top: 10px; overflow: hidden; }
    #tab-hexapod .hx-bar > div { height: 100%; width: 0; background: var(--accent-primary); transition: width 0.15s; }
    #tab-hexapod .hx-view { margin-top: 10px; background: #000; border: 1px solid var(--border-soft); border-radius: 6px;
      min-height: 120px; display: flex; align-items: center; justify-content: center; overflow: hidden; }
    #tab-hexapod .hx-view img { max-width: 100%; max-height: 70vh; display: block; }
    #tab-hexapod .hx-view .hx-sub { padding: 20px; text-align: center; }
  `;
  document.head.appendChild(style);

  const pane = document.createElement("section");
  pane.className = "tab-pane";
  pane.id = "tab-hexapod";
  pane.innerHTML = `
    <div class="card">
      <h2>🕷️ Hexapod <span class="badge-pill badge-idle u-text-0-7rem u-font-600" id="hx-link">not connected</span></h2>
      <p class="hint">The miniHexa's own sensors. Battery and sonar over <b>rosbridge</b> on port 9090 of this
        page's host, while this tab is open; the camera straight from its board.</p>
    </div>
    <div class="hx-grid">
      <div class="card" id="hx-bat-card">
        <h2>🔋 Battery <code class="u-text-0-7rem">/battery</code></h2>
        <div class="hx-big" id="hx-bat-v">--<span class="hx-unit">V</span></div>
        <div class="hx-sub" id="hx-bat-sub">waiting for /battery</div>
        <canvas class="hx-chart" id="hx-bat-chart"></canvas>
        <div class="hx-sub">Last ${HISTORY_S / 60} min. Lines: ${BAT_WARN.toFixed(1)} V (end the walk),
          ${BAT_FLOOR.toFixed(1)} V (floor at rest). Walking sags the pack by a few tenths.</div>
      </div>
      <div class="card" id="hx-sonar-card">
        <h2>📡 Sonar <code class="u-text-0-7rem">/sonar</code></h2>
        <div class="hx-big" id="hx-sonar-r">--<span class="hx-unit">cm</span></div>
        <div class="hx-sub" id="hx-sonar-sub">waiting for /sonar</div>
        <div class="hx-bar"><div id="hx-sonar-bar"></div></div>
        <canvas class="hx-chart" id="hx-sonar-chart"></canvas>
        <div class="hx-sub">Last ${HISTORY_S / 60} min, 0 to the sensor's max range.</div>
      </div>
      <div class="card" id="hx-gait-card">
        <h2>🦵 Gait <code class="u-text-0-7rem">/gait_params</code></h2>
        <div class="row">
          <button class="btn" data-gait="0" title="Body at the standard height, 30 mm swing, up to 214 mm/s">Tripod</button>
          <button class="btn" data-gait="2" title="Body 20 mm up (belly 65 mm), 50 mm swing, up to 64 mm/s">High walk</button>
          <button class="btn" data-gait="3" title="Body 40 mm up (belly 85 mm), 80 mm swing, up to 17 mm/s">Climb pose</button>
          <button class="btn" data-gait="1" title="Middle legs up, the four corners crawl, up to 20 mm/s">Quadruped</button>
        </div>
        <div class="hx-sub" id="hx-gait-sub">not sent</div>
        <div class="hx-sub"><b>High walk</b> steps over cables and pipes up to about 4.5 cm: the feet swing 5 cm
          high and the belly clears 6.5 cm. <b>Climb pose</b> raises the body as high as the legs still walk:
          the feet swing 8 cm high and the belly clears 8.5 cm, at a slow crawl. A new gait is taken when the
          robot stands; the body rises or lowers at 2 cm/s before it walks. Nav2 is tuned for the tripod: drive
          the others by teleop.</div>
      </div>
      <div class="card hx-cam">
        <h2>📷 Camera</h2>
        <div class="row">
          <label style="flex:1; min-width:220px;">Camera board URL
            <input type="text" id="hx-cam-url" placeholder="http://camera-ip" spellcheck="false" autocomplete="off">
          </label>
          <label>Mode
            <select id="hx-cam-mode">
              <option value="1000">Snapshot 1 Hz</option>
              <option value="500" selected>Snapshot 2 Hz</option>
              <option value="200">Snapshot 5 Hz</option>
              <option value="stream">Live stream (takes the only stream slot)</option>
            </select>
          </label>
          <button class="btn btn-primary" id="hx-cam-start">Start</button>
          <button class="btn btn-danger" id="hx-cam-stop">Stop</button>
        </div>
        <div class="hx-sub" id="hx-cam-sub">stopped</div>
        <div class="hx-view" id="hx-cam-view"><div class="hx-sub">Press Start.</div></div>
      </div>
    </div>`;
  main.appendChild(pane);

  const $ = (id) => document.getElementById(id);

  // ------------------------------------------------------------ the charts
  // A rolling series in seconds since page load; drawn on demand, at most once a frame.
  class Chart {
    constructor(canvas, { lo, hi, lines = [], fmt }) {
      this.c = canvas; this.lo = lo; this.hi = hi; this.lines = lines; this.fmt = fmt;
      this.pts = []; this.queued = false;
    }
    push(v) {
      const t = performance.now() / 1000;
      this.pts.push([t, v]);
      while (this.pts.length && this.pts[0][0] < t - HISTORY_S) this.pts.shift();
      this.draw();
    }
    draw() {
      if (this.queued) return;
      this.queued = true;
      requestAnimationFrame(() => { this.queued = false; this._draw(); });
    }
    _draw() {
      const c = this.c, dpr = window.devicePixelRatio || 1;
      const w = c.clientWidth, h = c.clientHeight;
      if (!w || !h) return;
      if (c.width !== Math.round(w * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
      const g = c.getContext("2d");
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      g.clearRect(0, 0, w, h);
      const lo = typeof this.lo === "function" ? this.lo() : this.lo;
      const hi = typeof this.hi === "function" ? this.hi() : this.hi;
      const y = (v) => h - 4 - (Math.min(Math.max(v, lo), hi) - lo) / (hi - lo) * (h - 8);
      const now = performance.now() / 1000;
      const x = (t) => w - (now - t) / HISTORY_S * w;
      g.font = "10px " + getComputedStyle(document.body).fontFamily;
      this.lines.forEach(([v, color], i) => {   // labels alternate sides: close lines stay readable
        g.strokeStyle = color; g.globalAlpha = 0.6; g.setLineDash([4, 4]);
        g.beginPath(); g.moveTo(0, y(v)); g.lineTo(w, y(v)); g.stroke();
        g.setLineDash([]); g.globalAlpha = 1; g.fillStyle = color;
        const label = this.fmt(v);
        g.fillText(label, i % 2 ? w - 4 - g.measureText(label).width : 4, y(v) - 3);
      });
      if (this.pts.length < 2) return;
      g.strokeStyle = "#818cf8"; g.lineWidth = 1.5;
      g.beginPath();
      this.pts.forEach(([t, v], i) => (i ? g.lineTo(x(t), y(v)) : g.moveTo(x(t), y(v))));
      g.stroke();
    }
  }

  const batChart = new Chart($("hx-bat-chart"), {
    lo: 6.4, hi: BAT_FULL + 0.1, fmt: (v) => v.toFixed(1) + " V",
    lines: [[BAT_WARN, "#eab308"], [BAT_FLOOR, "#ef4444"]],
  });
  let sonarMax = 4.0;
  const sonarChart = new Chart($("hx-sonar-chart"), { lo: 0, hi: () => sonarMax, fmt: (v) => v.toFixed(1) + " m" });

  // ------------------------------------------------------------ rosbridge
  let ws = null, wsWanted = false, wsTimer = null;
  const last = { battery: 0, sonar: 0 };

  function setLink(text, cls) {
    const el = $("hx-link");
    el.textContent = text;
    el.className = "badge-pill u-text-0-7rem u-font-600 " + cls;
  }

  function wsConnect() {
    clearTimeout(wsTimer);
    if (ws) return;
    const url = `ws://${window.location.hostname || "localhost"}:9090`;
    setLink("connecting…", "badge-idle");
    try { ws = new WebSocket(url); } catch (e) { setLink("bad URL", "pill-off"); return; }
    ws.onopen = () => {
      setLink("rosbridge connected", "pill-ok");
      const sub = (topic, type, throttle) =>
        ws.send(JSON.stringify({ op: "subscribe", topic, type, queue_length: 1, throttle_rate: throttle }));
      sub("/battery", "sensor_msgs/msg/BatteryState", 500);
      sub("/sonar", "sensor_msgs/msg/Range", 100);
    };
    ws.onmessage = (ev) => {
      let m; try { m = JSON.parse(ev.data); } catch { return; }
      if (m.op !== "publish") return;
      if (m.topic === "/battery") onBattery(m.msg);
      else if (m.topic === "/sonar") onSonar(m.msg);
    };
    ws.onclose = () => {
      ws = null;
      gaitAdvertised = false;
      setLink(wsWanted ? "rosbridge lost, retrying" : "not connected", wsWanted ? "pill-off" : "badge-idle");
      if (wsWanted) wsTimer = setTimeout(wsConnect, 3000);
    };
    ws.onerror = () => {};  // onclose follows
  }

  function wsClose() {
    wsWanted = false;
    clearTimeout(wsTimer);
    if (ws) { try { ws.close(); } catch { /* gone */ } }
  }

  // ------------------------------------------------------------ gait
  // The firmware's gait_params: [period, duty, lift, stride, mode], zeros keep the mode's defaults
  // (0 tripod, 1 quad, 2 high, 3 climb). Best effort on the board, so sent three times (idempotent).
  const GAIT_NAMES = { 0: "tripod", 1: "quadruped", 2: "high walk", 3: "climb pose" };
  let gaitAdvertised = false;
  function sendGait(mode) {
    if (!ws || ws.readyState !== WebSocket.OPEN) { $("hx-gait-sub").textContent = "rosbridge not connected"; return; }
    if (!gaitAdvertised) {
      ws.send(JSON.stringify({ op: "advertise", topic: "/gait_params", type: "std_msgs/msg/Float32MultiArray" }));
      gaitAdvertised = true;
    }
    const msg = { layout: { dim: [], data_offset: 0 }, data: [0, 0, 0, 0, mode] };
    for (let i = 0; i < 3; i++)
      setTimeout(() => { if (ws) ws.send(JSON.stringify({ op: "publish", topic: "/gait_params", msg })); }, i * 300);
    $("hx-gait-sub").textContent = `sent: ${GAIT_NAMES[mode]} at ${new Date().toLocaleTimeString()} (taken when it stands)`;
  }
  pane.querySelectorAll("[data-gait]").forEach((b) => b.addEventListener("click", () => sendGait(Number(b.dataset.gait))));

  // ------------------------------------------------------------ battery
  let batMin = Infinity, batMax = -Infinity;
  const SUPPLY = ["unknown", "charging", "discharging", "not charging", "full"];

  function onBattery(m) {
    const v = Number(m.voltage);
    if (!isFinite(v)) return;
    last.battery = performance.now();
    batMin = Math.min(batMin, v); batMax = Math.max(batMax, v);
    const el = $("hx-bat-v");
    el.innerHTML = `${v.toFixed(2)}<span class="hx-unit">V</span>`;
    el.className = "hx-big " + (v < BAT_FLOOR ? "hx-bad" : v < BAT_WARN ? "hx-warn" : "hx-ok");
    const pct = Math.round(Math.min(Math.max((v - 6.6) / (BAT_FULL - 6.6), 0), 1) * 100);
    const st = SUPPLY[m.power_supply_status] || "unknown";
    const verdict = v < BAT_FLOOR ? "below the floor: lie down and charge"
      : v < BAT_WARN ? "low: end the walk" : "ok";
    $("hx-bat-sub").textContent =
      `${verdict} · ~${pct} % by voltage · min ${batMin.toFixed(2)} / max ${batMax.toFixed(2)} V · supply ${st}`;
    batChart.push(v);
  }

  // ------------------------------------------------------------ sonar
  function onSonar(m) {
    const r = Number(m.range), lo = Number(m.min_range), hi = Number(m.max_range);
    last.sonar = performance.now();
    if (isFinite(hi) && hi > 0) sonarMax = hi;
    const el = $("hx-sonar-r"), sub = $("hx-sonar-sub"), bar = $("hx-sonar-bar");
    const fov = isFinite(m.field_of_view) ? (m.field_of_view * 180 / Math.PI).toFixed(0) : "?";
    if (!isFinite(r) || r < lo || r > hi) {
      el.innerHTML = `--<span class="hx-unit">cm</span>`;
      el.className = "hx-big";
      sub.textContent = `nothing in range (${(lo * 100).toFixed(0)}–${(hi * 100).toFixed(0)} cm, ${fov}° cone)`;
      bar.style.width = "100%";
      bar.style.background = "var(--text-dim)";
      return;
    }
    el.innerHTML = `${(r * 100).toFixed(1)}<span class="hx-unit">cm</span>`;
    el.className = "hx-big " + (r < 0.15 ? "hx-bad" : r < 0.30 ? "hx-warn" : "");
    sub.textContent = `${m.header?.frame_id || "sonar"} · ${(lo * 100).toFixed(0)}–${(hi * 100).toFixed(0)} cm, ${fov}° cone`;
    bar.style.width = `${Math.min(r / hi, 1) * 100}%`;
    bar.style.background = r < 0.15 ? "var(--accent-danger)" : r < 0.30 ? "var(--accent-warn)" : "var(--accent-primary)";
    sonarChart.push(r);
  }

  // Values that stopped coming are dimmed, not left looking live.
  setInterval(() => {
    const now = performance.now();
    $("hx-bat-card").classList.toggle("hx-stale", !!last.battery && now - last.battery > STALE_MS);
    $("hx-sonar-card").classList.toggle("hx-stale", !!last.sonar && now - last.sonar > STALE_MS);
    batChart.draw(); sonarChart.draw();
  }, 1000);

  // ------------------------------------------------------------ camera
  const camUrl = $("hx-cam-url"), camMode = $("hx-cam-mode"), camView = $("hx-cam-view"), camSub = $("hx-cam-sub");
  let camTimer = null, camImg = null, camFrames = 0, camT0 = 0, camRun = 0;

  function camBase() {
    let u = camUrl.value.trim().replace(/\/+$/, "");
    if (u && !/^https?:\/\//.test(u)) u = "http://" + u;
    return u.replace(/:\d+$/, "");   // the board's two servers are fixed ports: 80 and 81
  }

  function camStop(note = "stopped") {
    camRun++;
    clearTimeout(camTimer);
    if (camImg) { camImg.onload = camImg.onerror = null; camImg.src = ""; }   // drops the stream socket
    camImg = null;
    camView.innerHTML = `<div class="hx-sub">${note}</div>`;
    camSub.textContent = note;
  }

  function camStart() {
    const base = camBase();
    if (!base) { camStop("Type the camera board's URL first."); return; }
    try { localStorage.setItem(CAM_KEY, camUrl.value.trim()); } catch { /* private window */ }
    camStop("starting…");
    const run = camRun;
    camImg = new Image();
    camImg.alt = "hexapod camera";
    camView.innerHTML = "";
    camView.appendChild(camImg);
    camFrames = 0; camT0 = performance.now();
    if (camMode.value === "stream") {
      camImg.onload = () => { camSub.textContent = `live stream from ${base}:81/stream · Stop releases it`; };
      camImg.onerror = () => { if (run === camRun) camStop(`no stream from ${base}:81 (another reader holds it, or the board is off)`); };
      camImg.src = `${base}:81/stream`;
      camSub.textContent = `opening ${base}:81/stream…`;
      return;
    }
    const period = Number(camMode.value);
    const next = () => {
      if (run !== camRun) return;
      camImg.src = `${base}/capture?t=${Date.now()}`;
    };
    camImg.onload = () => {
      if (run !== camRun) return;
      camFrames++;
      const fps = camFrames / ((performance.now() - camT0) / 1000);
      camSub.textContent = `snapshots from ${base}/capture · ${camImg.naturalWidth}×${camImg.naturalHeight} · ${fps.toFixed(1)} frames/s`;
      camTimer = setTimeout(next, period);
    };
    camImg.onerror = () => {
      if (run !== camRun) return;
      camSub.textContent = `no frame from ${base}/capture, retrying`;
      camTimer = setTimeout(next, 2000);
    };
    next();
  }

  $("hx-cam-start").addEventListener("click", camStart);
  $("hx-cam-stop").addEventListener("click", () => camStop());
  camMode.addEventListener("change", () => { if (camImg) camStart(); });

  let saved = "";
  try { saved = localStorage.getItem(CAM_KEY) || ""; } catch { /* private window */ }
  camUrl.value = new URLSearchParams(window.location.search).get("cam") || saved || window.HEXA_CAMERA_URL || "";

  // ------------------------------------------------------------ shown / hidden
  // The tab's socket and the camera live only while the tab is shown: the stream slot is the camera's only one.
  let shown = false;
  function onTab(active) {
    if (active === shown) return;
    shown = active;
    if (active) {
      wsWanted = true; wsConnect();
      setTimeout(() => { batChart.draw(); sonarChart.draw(); }, 50);
      if (camBase()) camStart();
    } else {
      wsClose();
      camStop();
    }
  }
  nav.addEventListener("click", (e) => {
    const b = e.target.closest(".tab-btn");
    if (b) setTimeout(() => onTab(b.dataset.tab === "hexapod"), 0);
  });
  document.addEventListener("visibilitychange", () => {
    if (!pane.classList.contains("active")) return;
    if (document.hidden) { camStop("paused while the page is hidden"); }
    else if (camBase()) camStart();
  });
  window.hexaPanels = { onBattery, onSonar, camStart, camStop };
})();
