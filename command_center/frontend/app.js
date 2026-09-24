const $ = (id) => document.getElementById(id);

const buttons = {
  launch: $("btn-launch"),
  stopSim: $("btn-stop-sim"),
  feed: $("btn-feed"),
  stopFeed: $("btn-stop-feed"),
  mission: $("btn-mission"),
  land: $("btn-land"),
  drop: $("btn-drop"),
  estop: $("btn-estop"),
  mode: $("btn-mode"),
};

let sensorMode = "day";

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
    }
  } catch (err) {
    appendLog(`[gui] ${path} error: ${err}`);
  }
  // Buttons are re-enabled by the next /api/status update, which
  // reflects real process/mission state rather than this click.
}

const worldSelect = $("world-select");
buttons.launch.onclick = () => post("/api/sim/launch", buttons.launch, { world: worldSelect.value });
buttons.stopSim.onclick = () => post("/api/sim/stop", buttons.stopSim);
buttons.feed.onclick = () => post("/api/feed/start", buttons.feed);
buttons.stopFeed.onclick = () => post("/api/feed/stop", buttons.stopFeed);
buttons.mission.onclick = () => post("/api/mission/start", buttons.mission);
buttons.land.onclick = () => post("/api/mission/land", buttons.land);
buttons.drop.onclick = () => post("/api/payload/drop", buttons.drop);
buttons.estop.onclick = () => post("/api/emergency_stop", buttons.estop);
buttons.mode.onclick = async () => {
  const next = sensorMode === "night" ? "day" : "night";
  buttons.mode.disabled = true;
  try {
    const res = await fetch("/api/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: next }),
    });
    const body = await res.json();
    appendLog(`[gui] ${body.message}`);
  } catch (err) {
    appendLog(`[gui] mode switch error: ${err}`);
  }
  buttons.mode.disabled = false;
};

function applySensorMode(mode) {
  if (mode === sensorMode) return;
  sensorMode = mode;
  const night = mode === "night";
  document.body.classList.toggle("night", night);
  buttons.mode.textContent = night ? "Switch to RGB / Day" : "Switch to Thermal / Night";
}

const logEl = $("log");
function appendLog(line) {
  const atBottom = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 20;
  logEl.textContent += line + "\n";
  if (atBottom) logEl.scrollTop = logEl.scrollHeight;
}

function setDot(el, active) {
  el.classList.toggle("active", active);
}

function applyButtonState(s) {
  buttons.launch.disabled = s.sim_running;
  // While a sim runs, show (and lock) the world it's running.
  worldSelect.disabled = s.sim_running;
  if (s.sim_running && s.world) worldSelect.value = s.world;
  buttons.stopSim.disabled = !s.sim_running;
  buttons.feed.disabled = !s.sim_running || s.feed_running;
  buttons.stopFeed.disabled = !s.feed_running;
  buttons.mission.disabled = !s.bridge_running;
  buttons.land.disabled = !s.bridge_running;
  buttons.drop.disabled = !s.sim_running || s.payload_dropped;
}

function connectStatus() {
  const source = new EventSource("/api/status");
  const pill = $("connection");

  source.onopen = () => {
    pill.textContent = "Connected";
    pill.className = "pill pill-online";
  };
  source.onerror = () => {
    pill.textContent = "Reconnecting…";
    pill.className = "pill pill-offline";
  };

  source.onmessage = (event) => {
    const s = JSON.parse(event.data);
    $("t-armed").textContent = s.armed ? "ARMED" : "disarmed";
    $("t-mode").textContent = s.mode;
    $("t-sensor").textContent = s.sensor_mode === "night" ? "THERMAL (night)" : "RGB (day)";
    applySensorMode(s.sensor_mode);
    $("t-stage").textContent = s.mission_stage;
    $("t-search").textContent = s.search_total > 0
      ? `${s.search_done} / ${s.search_total} waypoints`
      : "\u2014";
    $("t-battery").textContent = s.battery_pct >= 0
      ? `${s.battery_pct}% (${s.battery_voltage.toFixed(2)} V)`
      : "unknown";
    $("t-altitude").textContent = `${s.altitude.toFixed(2)} m`;
    $("t-position").textContent = `${s.latitude.toFixed(6)}, ${s.longitude.toFixed(6)}`;
    $("t-persons").textContent = s.persons_detected;
    $("t-confirmed").textContent = s.person_confirmed ? "CONFIRMED" : "searching…";
    $("t-payload").textContent = s.payload_dropped ? "RELEASED" : "attached";

    setDot($("p-sim"), s.sim_running);
    setDot($("p-bridge"), s.bridge_running);
    setDot($("p-feed"), s.feed_running);

    applyButtonState(s);
  };
}

function connectLogs() {
  const source = new EventSource("/api/logs");
  source.onmessage = (event) => appendLog(JSON.parse(event.data));
}

connectStatus();
connectLogs();
