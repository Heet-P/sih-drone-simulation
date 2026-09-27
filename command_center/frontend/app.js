const $ = (id) => document.getElementById(id);
const $$ = (sel) => document.querySelectorAll(sel);

let status = null;   // latest /api/status payload
let mapData = null;  // latest /api/map payload
let sensorMode = "day";

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------

async function post(path, button, payload) {
  if (button) button.disabled = true;
  try {
    const res = await fetch(path, payload === undefined ? { method: "POST" } : {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const body = await res.json();
    if (!res.ok || !body.ok) {
      appendLog(`[gui] ${path} failed: ${body.message}`);
      toast("warn", body.message || `${path} failed`);
    }
    return body;
  } catch (err) {
    appendLog(`[gui] ${path} error: ${err}`);
    return null;
  } finally {
    // Buttons are re-enabled by the next /api/status update, which
    // reflects real process/mission state rather than this click.
    if (button) setTimeout(() => status && applyButtonState(status), 400);
  }
}

function setHtml(el, html) {
  // Only touch the DOM when the content changes (keeps animations from
  // replaying on every 2 Hz map update).
  if (el.dataset.html !== html) {
    el.innerHTML = html;
    el.dataset.html = html;
  }
}

function setLabel(button, text, icon) {
  button.querySelector("span").textContent = text;
  if (icon) button.querySelector("use").setAttribute("href", icon);
}

// ---------------------------------------------------------------
// Logs: Telemetry-view console + the log drawer (any view, key L)
// ---------------------------------------------------------------

const LOG_MAX = 4000;
const logLines = [];  // {src, text, level}
const logEl = $("log");
const drawer = $("drawer");
const drawerConsole = $("drawer-console");
const logSources = new Set(["bridge", "feed", "gui"]);
let logQuery = "";

function logGroup(src) {
  return ["gui", "mode", "payload"].includes(src) ? "gui" : src;
}

function lineLevel(text) {
  if (/\b(ERROR|Err\]|Traceback|failed|exception)/i.test(text)) return "err";
  if (/\b(WARN|WARNING|CRITICAL)\b/.test(text)) return "warn";
  return "";
}

function lineVisible(l) {
  return logSources.has(logGroup(l.src)) && (!logQuery || l.text.toLowerCase().includes(logQuery));
}

function lineNode(l) {
  const div = document.createElement("div");
  if (l.level) div.className = `l-${l.level}`;
  div.innerHTML = `<span class="src s-${logGroup(l.src)}">${escapeHtml(l.src)}</span> ${escapeHtml(l.text)}`;
  return div;
}

function appendLog(line) {
  const m = /^\[([\w-]+)\]\s?(.*)$/s.exec(line);
  const l = { src: m ? m[1] : "gui", text: m ? m[2] : line };
  l.level = lineLevel(l.text);
  logLines.push(l);
  if (logLines.length > LOG_MAX) logLines.splice(0, logLines.length - LOG_MAX);

  const atBottom = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 20;
  logEl.textContent += line + "\n";
  if (logEl.textContent.length > 400000) logEl.textContent = logEl.textContent.slice(-300000);
  if (atBottom) logEl.scrollTop = logEl.scrollHeight;

  if (lineVisible(l)) {
    drawerConsole.appendChild(lineNode(l));
    while (drawerConsole.childElementCount > LOG_MAX) drawerConsole.firstChild.remove();
    if ($("log-follow").checked) drawerConsole.scrollTop = drawerConsole.scrollHeight;
  }
}

function rebuildConsole() {
  drawerConsole.innerHTML = "";
  const frag = document.createDocumentFragment();
  for (const l of logLines) if (lineVisible(l)) frag.appendChild(lineNode(l));
  drawerConsole.appendChild(frag);
  drawerConsole.scrollTop = drawerConsole.scrollHeight;
}

let unseenAlerts = 0;
function setDrawer(open) {
  drawer.classList.toggle("open", open);
  drawer.setAttribute("aria-hidden", String(!open));
  $("btn-logs").classList.toggle("on", open);
  if (open) {
    unseenAlerts = 0;
    $("log-badge").classList.remove("show");
    drawerConsole.scrollTop = drawerConsole.scrollHeight;
  }
}
$("btn-logs").onclick = () => setDrawer(!drawer.classList.contains("open"));
$("drawer-close").onclick = () => setDrawer(false);
document.addEventListener("keydown", (e) => {
  if (e.target.matches("input, select, textarea")) return;
  if (e.key === "l" || e.key === "L") setDrawer(!drawer.classList.contains("open"));
  if (e.key === "Escape") setDrawer(false);
});
$$("#drawer-tabs button").forEach((b) => (b.onclick = () => {
  $$("#drawer-tabs button").forEach((x) => x.classList.toggle("active", x === b));
  const consoleTab = b.dataset.tab === "console";
  drawerConsole.hidden = !consoleTab;
  $("drawer-events").hidden = consoleTab;
  $("drawer-filters").style.visibility = consoleTab ? "visible" : "hidden";
  if (consoleTab) drawerConsole.scrollTop = drawerConsole.scrollHeight;
}));
$("drawer-filters").style.visibility = "hidden";
$$("#drawer-filters input[type=checkbox][value]").forEach((c) => (c.onchange = () => {
  if (c.checked) logSources.add(c.value); else logSources.delete(c.value);
  rebuildConsole();
}));
$("log-search").oninput = (e) => { logQuery = e.target.value.trim().toLowerCase(); rebuildConsole(); };

function noteAlert() {
  if (drawer.classList.contains("open")) return;
  unseenAlerts += 1;
  const b = $("log-badge");
  b.textContent = unseenAlerts;
  b.classList.add("show");
}

// ---------------------------------------------------------------
// Views (sidebar)
// ---------------------------------------------------------------

function showView(name) {
  $$(".nav-btn").forEach((b) => b.classList.toggle("active", b.dataset.view === name));
  $$(".view").forEach((v) => v.classList.toggle("active", v.dataset.view === name));
  try { localStorage.setItem("view", name); } catch (e) { /* storage unavailable */ }
}
$$(".nav-btn").forEach((b) => (b.onclick = () => showView(b.dataset.view)));
try { if (localStorage.getItem("view")) showView(localStorage.getItem("view")); } catch (e) { /* ignore */ }

// ---------------------------------------------------------------
// Actions
// ---------------------------------------------------------------

const buttons = {
  sim: $("btn-sim"),
  feed: $("btn-feed"),
  mode: $("btn-mode"),
  mission: $("btn-mission"),
  land: $("btn-land"),
  drop: $("btn-drop"),
  report: $("btn-report"),
  estop: $("btn-estop"),
};
const worldSelect = $("world-select");

buttons.sim.onclick = () => (status && status.sim_running
  ? post("/api/sim/stop", buttons.sim)
  : post("/api/sim/launch", buttons.sim, { world: worldSelect.value, seed: $("seed-input").value.trim() || null }));
buttons.feed.onclick = () => (status && status.feed_running
  ? post("/api/feed/stop", buttons.feed)
  : post("/api/feed/start", buttons.feed));
buttons.mission.onclick = () => post("/api/mission/start", buttons.mission, {
  planner: $("planner-select").value,
  on_confirm: $("confirm-select").value,
  mission_type: $("mission-type-select").value,
  hop_altitude_m: $("hop-alt-input").value.trim() || null,
});
function applyMissionType() {
  const hop = $("mission-type-select").value === "hop";
  setLabel(buttons.mission, hop ? "Start Hop Test" : "Start Search & Rescue");
}
$("mission-type-select").onchange = applyMissionType;
applyMissionType();
buttons.land.onclick = () => post("/api/mission/land", buttons.land);
buttons.drop.onclick = () => post("/api/payload/drop", buttons.drop);
buttons.estop.onclick = () => post("/api/emergency_stop", buttons.estop);
buttons.report.onclick = () => window.open("/report", "_blank");
$("btn-report-2").onclick = () => window.open("/report", "_blank");

async function setEnvironment(mode) {
  buttons.mode.disabled = true;
  const body = await post("/api/mode", null, { mode });
  if (body) appendLog(`[gui] ${body.message}`);
  buttons.mode.disabled = false;
}
buttons.mode.onclick = () => setEnvironment(sensorMode === "night" ? "day" : "night");
$("env-select").onchange = (e) => setEnvironment(e.target.value);

function applySensorMode(mode) {
  if (mode === sensorMode) return;
  sensorMode = mode;
  const night = mode === "night";
  document.body.classList.toggle("night", night);
  $("env-select").value = mode;
  document.querySelector(".sun-ico use").setAttribute("href", night ? "#i-moon" : "#i-sun");
  setLabel(buttons.mode, night ? "Switch to Day" : "Switch to Night");
}

function applyButtonState(s) {
  buttons.sim.disabled = false;
  buttons.sim.classList.toggle("primary", !s.sim_running);
  buttons.sim.classList.toggle("running", s.sim_running);
  if (s.real) {
    setLabel(buttons.sim, s.sim_running ? "Disconnect Drone" : "Connect Drone", s.sim_running ? "#i-stop" : "#i-play");
  } else {
    setLabel(buttons.sim, s.sim_running ? "Stop Simulation" : "Launch Simulation", s.sim_running ? "#i-stop" : "#i-play");
  }
  document.body.classList.toggle("real-drone", !!s.real);
  worldSelect.disabled = s.sim_running;
  $("seed-input").disabled = s.sim_running;
  if (s.sim_running && s.world) worldSelect.value = s.world;

  buttons.feed.disabled = !s.sim_running && !s.feed_running;
  buttons.feed.classList.toggle("running", s.feed_running);
  setLabel(buttons.feed, s.feed_running ? "Stop Camera Feed" : "Start Camera Feed");

  const flying = !["IDLE", "COMPLETE", "ABORTED", "PILOT"].includes(s.mission_stage);
  buttons.mission.disabled = !s.bridge_running || flying;
  buttons.land.disabled = !s.bridge_running;
  buttons.drop.disabled = !s.sim_running || s.payload_dropped;
}

// ---------------------------------------------------------------
// Camera feed: AI / thermal / split compare
// ---------------------------------------------------------------

const stage = $("feed-stage");
const thermalImg = $("thermal");
let feedMode = "ai";

function setFeedMode(mode) {
  feedMode = mode;
  stage.classList.remove("mode-ai", "mode-thermal", "mode-split");
  stage.classList.add(`mode-${mode}`);
  $$("#view-seg button").forEach((b) => b.classList.toggle("active", b.dataset.feed === mode));
  $("camera-select").value = mode;
  // Only hold the thermal stream open while it's on screen.
  const want = mode === "ai" ? "" : "/video_feed/thermal";
  if ((thermalImg.getAttribute("src") || "") !== want) {
    if (want) thermalImg.setAttribute("src", want);
    else thermalImg.removeAttribute("src");
  }
}
$$("#view-seg button").forEach((b) => (b.onclick = () => setFeedMode(b.dataset.feed)));
$("camera-select").onchange = (e) => setFeedMode(e.target.value);

function setSplit(pct) {
  pct = Math.max(8, Math.min(92, pct));
  stage.style.setProperty("--split", pct);
  const line = $("split-line");
  line.setAttribute("x1", pct + 4);
  line.setAttribute("x2", pct - 4);
}
const knob = $("split-knob");
knob.addEventListener("pointerdown", (e) => {
  knob.setPointerCapture(e.pointerId);
  const move = (ev) => {
    const r = stage.getBoundingClientRect();
    setSplit(((ev.clientX - r.left) / r.width) * 100);
  };
  knob.addEventListener("pointermove", move);
  knob.addEventListener("pointerup", () => knob.removeEventListener("pointermove", move), { once: true });
});
knob.addEventListener("keydown", (e) => {
  const cur = parseFloat(stage.style.getPropertyValue("--split")) || 50;
  if (e.key === "ArrowLeft") setSplit(cur - 4);
  if (e.key === "ArrowRight") setSplit(cur + 4);
});
setSplit(50);

$("btn-fullscreen").onclick = () => {
  const feed = $("feed");
  if (document.fullscreenElement) document.exitFullscreen();
  else if (feed.requestFullscreen) feed.requestFullscreen();
};
$("btn-reticle").onclick = (e) => {
  $("feed").classList.toggle("show-reticle");
  e.currentTarget.classList.toggle("on");
};

// ---------------------------------------------------------------
// Status stream (telemetry, processes)
// ---------------------------------------------------------------

const STAGE_LABEL = {
  IDLE: "IDLE", STARTING: "STARTING", ARMING: "ARMING", TAKEOFF: "TAKEOFF", CLIMBING: "CLIMBING",
  SEARCHING: "SEARCHING", VERIFYING: "VERIFYING CANDIDATE", MARKING: "MARKING CASUALTY",
  APPROACH: "APPROACHING", LANDING: "LANDING", RETURNING: "RETURNING HOME", COMPLETE: "COMPLETE", ABORTED: "ABORTED", PILOT: "PILOT OVERRIDE", HOVERING: "HOVERING",
};

function connectStatus() {
  const source = new EventSource("/api/status");
  const pill = $("connection");
  source.onerror = () => {
    pill.className = "chip off";
    pill.querySelector("span").textContent = "Offline";
  };
  source.onmessage = (event) => {
    const s = JSON.parse(event.data);
    status = s;
    const ready = s.sim_running && s.bridge_running;
    pill.className = `chip ${ready ? "ok" : "standby"}`;
    pill.querySelector("span").textContent = ready ? "System Ready" : "Standby";

    applySensorMode(s.sensor_mode);
    $("t-armed").textContent = s.armed ? "ARMED" : "disarmed";
    $("t-mode").textContent = s.mode;
    $("t-sensor").textContent = s.sensor_mode === "night" ? "THERMAL (night)" : "RGB (day)";
    $("t-stage").textContent = s.mission_stage;
    $("t-battery").textContent = s.battery_pct >= 0 ? `${s.battery_pct}% (${s.battery_voltage.toFixed(2)} V)` : "unknown";
    $("t-altitude").textContent = `${s.altitude.toFixed(2)} m`;
    $("t-position").textContent = `${s.latitude.toFixed(6)}, ${s.longitude.toFixed(6)}`;
    $("t-persons").textContent = s.persons_detected;
    $("t-confirmed").textContent = s.person_confirmed ? "CONFIRMED" : "searching…";
    $("t-payload").textContent = s.payload_dropped ? "RELEASED" : "attached";
    $("p-sim").classList.toggle("active", s.sim_running);
    $("p-bridge").classList.toggle("active", s.bridge_running);
    $("p-feed").classList.toggle("active", s.feed_running);

    $("s-alt").textContent = s.bridge_running ? s.altitude.toFixed(1) : "—";
    $("s-bat").textContent = s.battery_pct >= 0 ? s.battery_pct : "—";
    const bat = $("battery-chip");
    bat.querySelector("span").textContent = s.battery_pct >= 0 ? `${s.battery_pct}%` : "—";
    bat.classList.toggle("low", s.battery_pct >= 0 && s.battery_pct < 25);

    const stageName = s.mission_stage;
    const hud = $("hud-stage");
    hud.textContent = STAGE_LABEL[stageName] || stageName;
    hud.className = `hud-stage ${stageName === "VERIFYING" ? "verifying" : stageName === "MARKING" ? "marking" : ""}`;
    $("hud-alt").textContent = `${s.altitude.toFixed(1)} m`;
    $("t-stage-pill").textContent = STAGE_LABEL[stageName] || stageName;
    $("stage-pill").className = `stage-pill ${stageName === "VERIFYING" ? "verifying" : !["IDLE", "COMPLETE", "ABORTED", "PILOT"].includes(stageName) ? "active" : ""}`;

    const live = $("live-chip");
    live.classList.toggle("live", s.feed_running);
    $("live-text").textContent = s.feed_running ? "LIVE" : "OFFLINE";
    $("live-meta").textContent = s.feed_running
      ? (s.sensor_mode === "night" ? "640×480 · thermal AI (HIT-UAV)" : "640×480 · RGB AI (YOLOv8)")
      : (s.sim_running ? "Start the camera feed" : "Simulation not running");

    renderProgress();
    applyButtonState(s);
  };
}

function connectLogs() {
  const source = new EventSource("/api/logs");
  source.onmessage = (event) => appendLog(JSON.parse(event.data));
}

// ---------------------------------------------------------------
// Map stream: map, strip, rescue board, alerts, HUD
// ---------------------------------------------------------------

const mapViews = {
  "map-canvas": { zoom: 1, follow: false },
  "map-canvas-big": { zoom: 1, follow: false },
};
$$(".map-tools").forEach((tools) => {
  const view = mapViews[tools.dataset.map];
  tools.querySelectorAll("button").forEach((b) => {
    b.onclick = () => {
      if (b.dataset.zoom === "in") view.zoom = Math.min(6, view.zoom * 1.5);
      if (b.dataset.zoom === "out") view.zoom = Math.max(1, view.zoom / 1.5);
      if (b.dataset.zoom === "follow") {
        view.follow = !view.follow;
        if (view.follow && view.zoom < 2) view.zoom = 2.25;
        b.classList.toggle("on", view.follow);
      }
    };
  });
});

function renderLoop() {
  for (const [id, view] of Object.entries(mapViews)) {
    const canvas = $(id);
    if (canvas.offsetParent !== null) drawMap(canvas, mapData, view);
  }
  requestAnimationFrame(renderLoop);
}

function renderProgress() {
  let pct = null;
  let label = "Search progress";
  let value = "—";
  if (mapData && typeof mapData.pos === "number") {
    pct = mapData.pos * 100;
    label = mapData.config && mapData.config.planner === "grid" ? "Probability covered (grid)" : "Probability of success";
    value = `${pct.toFixed(0)}%`;
    $("t-search").textContent = `${value} probability covered`;
  } else if (status && status.search_total > 0) {
    pct = (100 * status.search_done) / status.search_total;
    value = `${status.search_done}/${status.search_total} wp`;
    $("t-search").textContent = value;
  }
  $("progress-label").textContent = label;
  $("progress-value").textContent = value;
  $("progress-bar").style.width = `${pct || 0}%`;
}

let audioCtx = null;
function alertTone(level) {
  // Two short tones for a critical alert (casualty, fire), one for a warning.
  try {
    audioCtx = audioCtx || new (window.AudioContext || window.webkitAudioContext)();
    const beeps = level === "critical" ? [880, 1175] : [660];
    beeps.forEach((freq, i) => {
      const osc = audioCtx.createOscillator();
      const gain = audioCtx.createGain();
      const t0 = audioCtx.currentTime + i * 0.16;
      osc.frequency.value = freq;
      gain.gain.setValueAtTime(0.0001, t0);
      gain.gain.exponentialRampToValueAtTime(0.15, t0 + 0.02);
      gain.gain.exponentialRampToValueAtTime(0.0001, t0 + 0.14);
      osc.connect(gain).connect(audioCtx.destination);
      osc.start(t0);
      osc.stop(t0 + 0.15);
    });
  } catch (err) { /* audio unavailable */ }
}

function toast(level, text) {
  const el = document.createElement("div");
  el.className = `toast ${level}`;
  el.innerHTML = `<svg class="i"><use href="#i-alert"/></svg><div>${escapeHtml(text)}</div>`;
  const box = $("toasts");
  box.prepend(el);
  while (box.children.length > 3) box.lastChild.remove();
  setTimeout(() => el.classList.add("out"), level === "critical" ? 7000 : 5000);
  setTimeout(() => el.remove(), level === "critical" ? 7500 : 5500);
}

// events holds the bridge's most recent alerts; total counts every alert
// ever raised this mission, so the new ones are the last (total - seen).
let seenEvents = null;
function renderAlerts(events, total) {
  const lists = [$("alerts"), $("drawer-events")];
  if (seenEvents === null || total < seenEvents) {
    // On (re)connect or a new mission, show the backlog without sounding it.
    for (const list of lists) {
      list.innerHTML = "";
      for (const ev of events) list.prepend(alertItem(ev, false));
    }
    seenEvents = total;
    return;
  }
  for (const ev of events.slice(Math.max(0, events.length - (total - seenEvents)))) {
    for (const list of lists) list.prepend(alertItem(ev, true));
    noteAlert();
    if (ev.level === "critical" || ev.level === "warn") {
      alertTone(ev.level);
      toast(ev.level, ev.text);
    }
  }
  seenEvents = total;
}

// Side view of the forward-lidar corridor: obstacles ahead (bars), the
// drone, and the altitude it is holding.
function renderObstacle(o) {
  const card = $("obstacle-card");
  const txt = $("obs-text");
  const canvas = $("obs-canvas");
  if (!o) return;
  card.classList.toggle("raised", !!o.raised && !o.blocked);
  card.classList.toggle("blocked", !!o.blocked);
  txt.className = `meta ${o.blocked ? "bad" : o.raised ? "warn" : ""}`;
  if (!o.lidar_live) txt.textContent = "lidar offline";
  else if (o.blocked) txt.textContent = `BLOCKED ${o.ahead_m} m ahead \u00b7 climbing`;
  else if (o.raised) txt.textContent = `obstacle ${o.top_m != null ? o.top_m + " m tall" : "below"} \u00b7 holding ${o.cmd_alt.toFixed(1)} m`;
  else if (o.ahead_m != null) txt.textContent = `clear \u00b7 low object ${o.ahead_m} m ahead`;
  else txt.textContent = `clear ahead${o.agl != null ? ` \u00b7 AGL ${o.agl.toFixed(1)} m` : ""}`;

  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (!w || !h) return;
  if (canvas.width !== Math.round(w * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const range = 35;
  const alt = (status && status.altitude) || 0;
  const maxH = Math.max(20, o.cmd_alt + 6, ...(o.profile || []).map((p) => p[1] + 3));
  const X = (d) => 28 + (d / range) * (w - 36);
  const Y = (z) => h - 8 - (z / maxH) * (h - 14);
  ctx.strokeStyle = "rgba(148,163,184,0.35)";
  ctx.beginPath(); ctx.moveTo(0, Y(0)); ctx.lineTo(w, Y(0)); ctx.stroke();
  for (const [d, top] of o.profile || []) {
    ctx.fillStyle = o.blocked ? "rgba(239,68,68,0.75)" : top > o.search_alt - 4 ? "rgba(245,158,11,0.75)" : "rgba(148,163,184,0.55)";
    const bw = Math.max(3, (2 / range) * (w - 36) - 1);
    ctx.fillRect(X(d) - bw / 2, Y(top), bw, Y(0) - Y(top));
  }
  ctx.setLineDash([4, 4]);
  ctx.strokeStyle = "rgba(74,222,128,0.9)";
  ctx.beginPath(); ctx.moveTo(X(0), Y(o.cmd_alt)); ctx.lineTo(w, Y(o.cmd_alt)); ctx.stroke();
  ctx.strokeStyle = "rgba(148,163,184,0.45)";
  ctx.beginPath(); ctx.moveTo(X(0), Y(o.search_alt)); ctx.lineTo(w, Y(o.search_alt)); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#f8fafc";
  ctx.beginPath(); ctx.moveTo(10, Y(alt) - 5); ctx.lineTo(24, Y(alt)); ctx.lineTo(10, Y(alt) + 5); ctx.closePath(); ctx.fill();
  ctx.fillStyle = "rgba(148,163,184,0.8)";
  ctx.font = "10px system-ui, sans-serif";
  ctx.fillText(`${range} m`, w - 30, h - 10);
  ctx.fillText(`${o.cmd_alt.toFixed(0)} m`, X(0) + 4, Y(o.cmd_alt) - 3);
}

function alertItem(ev, fresh) {
  const li = document.createElement("li");
  li.className = `${ev.level}${fresh ? " fresh" : ""}`;
  li.innerHTML = `<time>T+${fmtTime(ev.t)}</time>${escapeHtml(ev.text)}`;
  return li;
}

function renderBoard(d) {
  const victims = [...(d.victims || [])].sort((a, b) => (a.priority || 99) - (b.priority || 99));
  setHtml($("victims"), victims.length ? victims.map((v) => `
    <div class="victim-card ${v.priority === 1 ? "p1" : ""}">
      <div class="victim-head">
        <span class="victim-prio">P${v.priority || "?"}</span>
        <span class="victim-id">${v.id}</span>
        <span class="victim-meta">found T+${fmtTime(v.found_s)} &middot; ${v.sensor.toUpperCase()} ${v.conf.toFixed(2)}${v.moving ? " &middot; MOVING (responsive)" : " &middot; not moving"}</span>
      </div>
      <div class="victim-meta">${v.lat.toFixed(6)}, ${v.lon.toFixed(6)} &middot; E${v.e.toFixed(1)} N${v.n.toFixed(1)} m${v.aid_dropped ? " &middot; aid kit dropped" : ""}</div>
      ${v.risks && v.risks.length ? `<div class="victim-risks">&#9888; ${escapeHtml(v.risks.join(", "))}</div>` : ""}
      ${v.action ? `<div class="victim-action">&#10148; ${escapeHtml(v.action)}</div>` : ""}
    </div>`).join("") : '<p class="empty">No casualties confirmed yet.</p>');

  const hazards = (d.hazards || []).filter((h) => h.source !== "intel");
  const intel = (d.hazards || []).length - hazards.length;
  setHtml($("hazards"), (hazards.length ? hazards.map((h) => `
    <div class="hazard-row">
      <span>${(HAZARD_STYLE[h.type] || HAZARD_STYLE.structure).icon}</span>
      <b>${h.id}</b> ${escapeHtml(h.label)}
      <span class="dim">E${h.e.toFixed(0)} N${h.n.toFixed(0)} &middot; ~${(2 * h.radius).toFixed(0)} m${h.peak_k ? ` &middot; ${(h.peak_k - 273.15).toFixed(0)}&deg;C` : ""} &middot; ${h.sensors.join("+").toUpperCase()}</span>
    </div>`).join("") : '<p class="empty">None detected by the drone yet.</p>')
    + (intel ? `<p class="empty" style="margin-top:6px">+ ${intel} collapsed structures from pre-mission intel</p>` : ""));

  // Home strip: compact chips
  setHtml($("strip-victims"), victims.length ? victims.map((v) => `
    <span class="mini ${v.priority === 1 ? "p1" : "p2"}"><svg class="i"><use href="#i-person"/></svg><b>P${v.priority || "?"}</b> ${v.id}${v.risks && v.risks.length ? ` &middot; ${escapeHtml(v.risks[0])}` : ""}</span>`).join("")
    : '<span class="none">none yet</span>');
  setHtml($("strip-hazards"), hazards.length ? hazards.map((h) => `
    <span class="mini ${h.type}"><svg class="i"><use href="#i-${h.type === "fire" ? "flame" : "alert"}"/></svg><b>${h.id}</b> ${h.type === "fire" && h.peak_k ? `${(h.peak_k - 273.15).toFixed(0)}&deg;C` : escapeHtml(h.label.toLowerCase())}</span>`).join("")
    : '<span class="none">none yet</span>');

  const cfg = d.config || {};
  const planner = cfg.planner === "grid" ? "grid" : cfg.planner === "bayes" ? "bayesian" : "";
  $("map-meta").textContent = d.mission_id
    ? `${d.mission_id} · T+${fmtTime(d.t)} · ${planner}${d.detector_live ? "" : " · detector offline"}`
    : "";
}

function renderDrone(d) {
  const dr = d.drone || {};
  const hdg = ((((dr.yaw || 0) * 180) / Math.PI) + 360) % 360;
  $("compass-needle").style.transform = `rotate(${hdg}deg)`;
  $("hud-hdg").textContent = `${hdg.toFixed(0).padStart(3, "0")}°`;
  $("hud-spd").textContent = dr.speed != null ? `${dr.speed.toFixed(1)} m/s` : "—";
  $("s-spd").textContent = dr.speed != null ? dr.speed.toFixed(1) : "—";
  $("s-sat").textContent = dr.sats != null ? dr.sats : "—";
  const gps = $("gps-chip");
  const locked = dr.gps_fix >= 3;
  gps.querySelector("span").textContent = dr.gps_fix == null ? "GPS —" : locked ? `GPS Locked · ${dr.sats}` : "No GPS fix";
  gps.classList.toggle("ok-text", locked);
}

function connectMap() {
  const source = new EventSource("/api/map");
  source.onmessage = (event) => {
    mapData = JSON.parse(event.data);
    renderBoard(mapData);
    renderDrone(mapData);
    renderObstacle(mapData.obstacle);
    renderProgress();
    renderAlerts(mapData.events || [], mapData.events_total || 0);
  };
}

// ---------------------------------------------------------------
// Clock
// ---------------------------------------------------------------

function tickClock() {
  $("clock").textContent = `${new Date().toISOString().slice(11, 19)} UTC`;
}
setInterval(tickClock, 1000);
tickClock();

setFeedMode("ai");
connectStatus();
connectLogs();
connectMap();
requestAnimationFrame(renderLoop);
