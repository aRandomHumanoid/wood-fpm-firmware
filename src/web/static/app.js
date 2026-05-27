// Wood FPM frontend.
//
// Live state via Socket.IO; scan history is persisted to CSV on the server
// and the plot is redrawn from that CSV so the browser and stored history
// stay in sync.

const socket = io({ transports: ["websocket", "polling"] });
const MAX_SERIAL_CONSOLE_ENTRIES = 500;
const plotOptions = { responsive: true };
let serialConnected = false;
let serialSimulate = false;
let serialConsoleLoaded = false;
let serialConsoleEntries = [];
let displayedSerialConsoleEntries = [];
let serialFilterState = { suppressKeepalive: false };
let controllerBusy = false;
let scanRunning = false;
let loopScanRequested = false;
let loopScanRequestBody = null;
let jogRequestPending = false;
let jogRequestObservedBusy = false;

const $ = (id) => document.getElementById(id);
const setVal = (id, v, digits = 2) => {
  const el = $(id);
  if (!el) return;
  el.textContent = (v == null || Number.isNaN(v)) ? "—" : Number(v).toFixed(digits);
};
const setLed = (id, on, warn = false) => {
  const el = $(id); if (!el) return;
  el.classList.toggle("on", !!on && !warn);
  el.classList.toggle("warn", !!warn);
};

function updateJogMode(homed) {
  const el = $("jog-mode");
  if (!el) return;
  el.textContent = homed
    ? "Mode: absolute workspace (homed)"
    : "Mode: relative (unhomed)";
  el.classList.toggle("homed", !!homed);
}

function updateJogControls() {
  const disabled = controllerBusy || jogRequestPending;
  document.querySelectorAll(".jog").forEach((button) => {
    button.disabled = disabled;
  });
}

function updateScanControls() {
  const scanControlsDisabled = scanRunning || loopScanRequested || controllerBusy || !serialConnected;
  $("run-scan").disabled = scanControlsDisabled;
  $("loop-scan").disabled = scanControlsDisabled;
  $("loop-scan").textContent = loopScanRequested ? "Looping..." : "Loop scan";
  $("abort-scan").disabled = !(scanRunning || loopScanRequested);
  $("clear-plot").disabled = scanRunning || loopScanRequested;
}

function readScanRequestBody() {
  return {
    x_max: +$("s-xmax").value,
    probe_target_x: +$("s-probe-x").value,
    probe_speed_mm_s: +$("s-probe-speed").value,
    y_max: +$("s-ymax").value,
    y_min: +$("s-ymin").value,
    n_samples: +$("s-n").value,
  };
}

function clearLoopScanRequest() {
  loopScanRequested = false;
  loopScanRequestBody = null;
  updateScanControls();
}

async function startScan(body) {
  const result = await post("/api/scan", body);
  if (!result.ok) {
    if (result.error === "scan already running") {
      scanRunning = true;
      updateScanControls();
    }
    $("last-error").textContent = result.error || "scan failed";
    return false;
  }

  scanRunning = true;
  $("last-error").textContent = "";
  updateScanControls();
  return true;
}

async function startLoopScanIteration() {
  if (!loopScanRequested || !loopScanRequestBody) {
    return;
  }
  const started = await startScan(loopScanRequestBody);
  if (!started) {
    clearLoopScanRequest();
  }
}

// ---------- live state ----------
socket.on("state", (s) => {
  setVal("pos-x", s.x);
  setVal("pos-y", s.y);
  setVal("pos-z", s.z);
  setLed("homed-led", s.homed, false);
  updateJogMode(s.homed);
  controllerBusy = !!s.busy;
  if (typeof s.scan_running === "boolean") {
    scanRunning = !!s.scan_running;
  }
  if (jogRequestPending && controllerBusy) {
    jogRequestObservedBusy = true;
  }
  if (jogRequestPending && jogRequestObservedBusy && !controllerBusy) {
    jogRequestPending = false;
    jogRequestObservedBusy = false;
  }
  updateJogControls();
  updateScanControls();
  setLed("busy-led",  s.busy,  false);
  setLed("fault-led", s.fault, s.fault);
  if (typeof s.serial_connected === "boolean") {
    serialConnected = s.serial_connected;
    updateSerialPanel();
  }
  $("block-override").disabled = !s.busy;
  $("estop-reset").disabled = s.last_error !== "E-STOP";
  $("fault-clear").disabled = !(s.busy || s.fault || s.last_error);
  $("last-error").textContent = s.last_error || "";
});

// ---------- limits (one-shot) ----------
fetch("/api/limits").then(r => r.json()).then((L) => {
  $("lim-x").textContent = `${L.bounds.x_min} … ${L.bounds.x_max}`;
  $("lim-y").textContent = `${L.bounds.y_min} … ${L.bounds.y_max}`;
  setVal("lim-v", L.v_max_mm_s);
  setVal("lim-a", L.a_max_mm_s2);
  $("s-probe-speed").value = L.scan_feed_mm_s;
});

// ---------- jog / home / e-stop ----------
async function post(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: body ? JSON.stringify(body) : "{}",
  });
  return res.json();
}

function showActionError(result, fallbackMessage) {
  $("last-error").textContent = result?.error || fallbackMessage;
}

function formatError(err) {
  return err instanceof Error ? err.message : String(err);
}

function setSelectOptions(id, options, selectedValue) {
  const el = $(id);
  if (!el) return;
  const values = options.length > 0 ? options : [selectedValue || ""];
  el.innerHTML = "";
  values.forEach((value) => {
    const option = document.createElement("option");
    option.value = value;
    option.textContent = value || "No ports found";
    el.appendChild(option);
  });
  if (selectedValue && values.includes(selectedValue)) {
    el.value = selectedValue;
  }
}

function updateSerialPanel(status = null) {
  if (status) {
    serialConnected = !!status.connected;
    serialSimulate = !!status.simulate;
    $("serial-driver-mode").value = serialSimulate ? "simulation" : "serial";
    setSelectOptions("serial-port", status.ports || [], status.port || "");
    $("serial-baudrate").value = status.baudrate ?? "";
    $("serial-status").textContent = status.last_error || "";
  }
  setLed("serial-led", serialConnected, false);
  $("serial-toggle").textContent = serialConnected ? "Disconnect" : "Connect";
  $("serial-mode").textContent = serialSimulate ? "simulation" : "serial";
  updateScanControls();
}

function appendSerialEntry(entry) {
  const consoleEl = $("serial-console");
  if (!consoleEl || !entry) return;
  const line = document.createElement("div");
  line.className = `serial-line ${entry.direction || "meta"}`;
  const prefix = entry.direction === "tx" ? ">" : entry.direction === "rx" ? "<" : "*";
  line.textContent = `${prefix} ${entry.line || ""}`;
  consoleEl.appendChild(line);
  consoleEl.scrollTop = consoleEl.scrollHeight;
}

function createSerialFilterState() {
  return { suppressKeepalive: false };
}

function filterSerialEntry(entry, filterState) {
  const line = String(entry.line || "").trim();
  if (!filterState.suppressKeepalive && entry.direction === "tx" && /^M114(?:\s|;|$)/i.test(line)) {
    filterState.suppressKeepalive = true;
    return null;
  }

  if (filterState.suppressKeepalive) {
    if (entry.direction === "rx") {
      const lower = line.toLowerCase();
      if (lower === "ok" || lower.startsWith("ok ") || lower.startsWith("error") || lower.startsWith("!!")) {
        filterState.suppressKeepalive = false;
      }
      return null;
    }
    filterState.suppressKeepalive = false;
  }

  return entry;
}

function replaceSerialConsole(entries = displayedSerialConsoleEntries) {
  const consoleEl = $("serial-console");
  if (!consoleEl) return;
  consoleEl.innerHTML = "";
  entries.forEach((entry) => appendSerialEntry(entry));
}

function rebuildSerialConsole() {
  const filterEnabled = $("serial-filter-m114")?.checked;
  const filterState = createSerialFilterState();
  displayedSerialConsoleEntries = [];

  serialConsoleEntries.forEach((entry) => {
    const visibleEntry = filterEnabled ? filterSerialEntry(entry, filterState) : entry;
    if (visibleEntry) {
      displayedSerialConsoleEntries.push(visibleEntry);
    }
  });

  serialFilterState = filterState;
  replaceSerialConsole();
}

function trimSerialConsoleEntries() {
  if (serialConsoleEntries.length <= MAX_SERIAL_CONSOLE_ENTRIES) {
    return false;
  }
  serialConsoleEntries = serialConsoleEntries.slice(-MAX_SERIAL_CONSOLE_ENTRIES);
  return true;
}

async function refreshSerialStatus() {
  try {
    const res = await fetch("/api/serial", { cache: "no-store" });
    updateSerialPanel(await res.json());
  } catch (err) {
    $("serial-status").textContent = err instanceof Error ? err.message : String(err);
  }
}

async function refreshSerialConsole() {
  try {
    const res = await fetch("/api/serial/console", { cache: "no-store" });
    const payload = await res.json();
    serialConsoleEntries = payload.entries || [];
    rebuildSerialConsole();
    serialConsoleLoaded = true;
  } catch (err) {
    $("serial-status").textContent = formatError(err);
  }
}

document.querySelectorAll(".jog").forEach((b) => {
  b.addEventListener("click", async () => {
    if (controllerBusy || jogRequestPending) {
      return;
    }
    jogRequestPending = true;
    jogRequestObservedBusy = false;
    updateJogControls();
    const step = parseFloat($("jog-step").value) || 0;
    const dx = parseFloat(b.dataset.dx) * step;
    const dy = parseFloat(b.dataset.dy) * step;
    const result = await post("/api/jog", { dx, dy });
    if (!result.ok) {
      jogRequestPending = false;
      jogRequestObservedBusy = false;
      updateJogControls();
      showActionError(result, "jog failed");
    }
  });
});

$("home").addEventListener("click", async () => {
  const result = await post("/api/home");
  if (!result.ok) {
    showActionError(result, "home failed");
  }
});
$("estop").addEventListener("click", async () => {
  const result = await post("/api/stop");
  if (!result.ok) {
    showActionError(result, "E-STOP failed");
  }
});
$("estop-reset").addEventListener("click", async () => {
  const result = await post("/api/stop/reset");
  if (!result.ok) {
    showActionError(result, "E-STOP reset failed");
  }
});
$("block-override").addEventListener("click", async () => {
  const result = await post("/api/block/override");
  if (!result.ok) {
    showActionError(result, "block override failed");
  }
});
$("fault-clear").addEventListener("click", async () => {
  const result = await post("/api/fault/clear");
  if (!result.ok) {
    showActionError(result, "clear fault failed");
  }
});
$("serial-toggle").addEventListener("click", async () => {
  const url = serialConnected ? "/api/serial/disconnect" : "/api/serial/connect";
  const body = serialConnected ? null : {
    simulate: $("serial-driver-mode").value === "simulation",
    port: $("serial-port").value,
    baudrate: +$("serial-baudrate").value,
  };
  const result = await post(url, body);
  if (!result.ok) {
    $("serial-status").textContent = result.error || "serial update failed";
    return;
  }
  updateSerialPanel(result);
});
$("serial-refresh").addEventListener("click", () => refreshSerialStatus());
$("serial-send").addEventListener("click", async () => {
  const command = $("serial-command").value.trim();
  if (!command) return;
  const result = await post("/api/serial/command", { command });
  if (!result.ok) {
    $("serial-status").textContent = result.error || "command failed";
    return;
  }
  $("serial-command").value = "";
});
$("serial-command").addEventListener("keydown", (event) => {
  if (event.key === "Enter") {
    event.preventDefault();
    $("serial-send").click();
  }
});
$("serial-console-clear").addEventListener("click", async () => {
  const result = await post("/api/serial/console/clear");
  if (!result.ok) {
    $("serial-status").textContent = result.error || "clear failed";
    return;
  }
  serialConsoleEntries = [];
  displayedSerialConsoleEntries = [];
  serialFilterState = createSerialFilterState();
  replaceSerialConsole();
});
$("serial-filter-m114").addEventListener("change", () => {
  rebuildSerialConsole();
});

socket.on("serial_entry", (entry) => {
  if (!serialConsoleLoaded) return;
  serialConsoleEntries.push(entry);
  if (trimSerialConsoleEntries()) {
    rebuildSerialConsole();
    return;
  }

  const visibleEntry = $("serial-filter-m114")?.checked
    ? filterSerialEntry(entry, serialFilterState)
    : entry;

  if (!visibleEntry) {
    return;
  }

  displayedSerialConsoleEntries.push(visibleEntry);
  appendSerialEntry(visibleEntry);
});

// ---------- scan + plot ----------
const plotEl = $("plot");
const plotScanRows = new Map();
const plotScanOrder = [];
const plotTraceIndexByScanId = new Map();
let plotUpdateQueue = Promise.resolve();
const plotLayout = {
  paper_bgcolor: "#25252a",
  plot_bgcolor:  "#1c1c1f",
  font: { color: "#ddd" },
  margin: { l: 50, r: 20, t: 20, b: 40 },
  xaxis: { title: "Y (mm)", gridcolor: "#333" },
  yaxis: { title: "X contact (mm)", gridcolor: "#333" },
  showlegend: true,
};

function queuePlotUpdate(task) {
  plotUpdateQueue = plotUpdateQueue.then(task).catch((err) => {
    $("last-error").textContent = formatError(err);
  });
  return plotUpdateQueue;
}

function resetPlotState() {
  plotScanRows.clear();
  plotScanOrder.length = 0;
  plotTraceIndexByScanId.clear();
}

function ensurePlotScan(scanId) {
  let rows = plotScanRows.get(scanId);
  if (!rows) {
    rows = [];
    plotScanRows.set(scanId, rows);
    plotScanOrder.push(scanId);
  }
  return rows;
}

function setPlotRows(rows) {
  resetPlotState();
  rows.forEach((row) => {
    ensurePlotScan(row.scan_id).push(row);
  });
}

function buildPlotTraces() {
  plotTraceIndexByScanId.clear();
  let traceIndex = 0;

  return plotScanOrder.flatMap((scanId, idx) => {
    const points = (plotScanRows.get(scanId) || [])
      .slice()
      .sort((a, b) => a.index - b.index)
      .filter((row) => row.x != null && row.y != null);
    if (points.length === 0) return [];

    const age = plotScanOrder.length - 1 - idx;
    plotTraceIndexByScanId.set(scanId, traceIndex);
    traceIndex += 1;

    return [{
      x: points.map((row) => row.y),
      y: points.map((row) => row.x),
      mode: "lines+markers",
      name: `scan ${idx + 1}`,
      line: { width: 2 },
      marker: { size: 6 },
      opacity: age === 0 ? 1.0 : Math.max(0.1, 0.5 - 0.08 * age),
    }];
  });
}

function parseCsv(text) {
  const lines = text.trim().split(/\r?\n/).filter(Boolean);
  if (lines.length <= 1) return [];
  const header = lines[0].split(",");
  return lines.slice(1).map((line) => {
    const values = line.split(",");
    const row = {};
    header.forEach((key, idx) => {
      row[key] = values[idx] ?? "";
    });
    return {
      scan_id: row.scan_id,
      index: Number(row.index),
      x: row.x === "" ? null : Number(row.x),
      y: row.y === "" ? null : Number(row.y),
    };
  });
}

async function rebuildPlotFromState() {
  await Plotly.react(plotEl, buildPlotTraces(), plotLayout, plotOptions);
}

async function syncPlotFromCsv() {
  const res = await fetch("/api/scan/history.csv", { cache: "no-store" });
  const rows = parseCsv(await res.text());
  setPlotRows(rows);
  await rebuildPlotFromState();
}

socket.on("scan_started", (req) => {
  scanRunning = true;
  updateScanControls();
  void queuePlotUpdate(async () => {
    ensurePlotScan(req.scan_id);
    await rebuildPlotFromState();
  });
});

socket.on("scan_point", (pt) => {
  void queuePlotUpdate(async () => {
    ensurePlotScan(pt.scan_id).push(pt);
    if (pt.x == null || pt.y == null) {
      return;
    }

    const traceIndex = plotTraceIndexByScanId.get(pt.scan_id);
    if (traceIndex == null) {
      await rebuildPlotFromState();
      return;
    }

    await Plotly.extendTraces(plotEl, { x: [[pt.y]], y: [[pt.x]] }, [traceIndex]);
  });
});

socket.on("scan_complete", () => {
  scanRunning = false;
  updateScanControls();
  void queuePlotUpdate(syncPlotFromCsv);
  if (loopScanRequested && loopScanRequestBody) {
    void startLoopScanIteration();
  }
});

$("run-scan").addEventListener("click", async () => {
  if (scanRunning || loopScanRequested || controllerBusy || !serialConnected) {
    return;
  }
  await startScan(readScanRequestBody());
});

$("loop-scan").addEventListener("click", async () => {
  if (scanRunning || loopScanRequested || controllerBusy || !serialConnected) {
    return;
  }
  loopScanRequestBody = readScanRequestBody();
  loopScanRequested = true;
  updateScanControls();
  await startLoopScanIteration();
});

$("abort-scan").addEventListener("click", async () => {
  clearLoopScanRequest();
  const r = await post("/api/scan/abort");
  if (!r.ok) {
    $("last-error").textContent = r.error || "abort failed";
  }
});

$("clear-plot").addEventListener("click", async () => {
  const r = await post("/api/scan/history/clear");
  if (!r.ok) {
    $("last-error").textContent = r.error || "clear failed";
    return;
  }
  await queuePlotUpdate(syncPlotFromCsv);
});

void queuePlotUpdate(async () => {
  await Plotly.newPlot(plotEl, [], plotLayout, plotOptions);
  await syncPlotFromCsv();
});
void refreshSerialStatus();
void refreshSerialConsole();
updateScanControls();
