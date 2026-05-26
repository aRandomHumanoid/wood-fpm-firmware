// Wood FPM frontend.
//
// Live state via Socket.IO; scan history is persisted to CSV on the server
// and the plot is redrawn from that CSV so the browser and stored history
// stay in sync.

const socket = io({ transports: ["polling"] });
let serialConnected = false;
let serialSimulate = false;
let serialConsoleLoaded = false;
let serialConsoleEntries = [];
let controllerBusy = false;
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

// ---------- live state ----------
socket.on("state", (s) => {
  setVal("pos-x", s.x);
  setVal("pos-y", s.y);
  setVal("pos-z", s.z);
  setLed("homed-led", s.homed, false);
  updateJogMode(s.homed);
  controllerBusy = !!s.busy;
  if (jogRequestPending && controllerBusy) {
    jogRequestObservedBusy = true;
  }
  if (jogRequestPending && jogRequestObservedBusy && !controllerBusy) {
    jogRequestPending = false;
    jogRequestObservedBusy = false;
  }
  updateJogControls();
  setLed("busy-led",  s.busy,  false);
  setLed("fault-led", s.fault, s.fault);
  if (typeof s.serial_connected === "boolean") {
    serialConnected = s.serial_connected;
    updateSerialPanel();
  }
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

function filteredSerialEntries(entries) {
  if (!$("serial-filter-m114")?.checked) return entries;
  const filtered = [];
  let suppressKeepalive = false;

  entries.forEach((entry) => {
    const line = String(entry.line || "").trim();
    if (!suppressKeepalive && entry.direction === "tx" && /^M114(?:\s|;|$)/i.test(line)) {
      suppressKeepalive = true;
      return;
    }

    if (suppressKeepalive) {
      if (entry.direction === "rx") {
        const lower = line.toLowerCase();
        if (lower === "ok" || lower.startsWith("ok ") || lower.startsWith("error") || lower.startsWith("!!")) {
          suppressKeepalive = false;
        }
        return;
      }
      if (entry.direction !== "rx") {
        suppressKeepalive = false;
      }
    }

    filtered.push(entry);
  });

  return filtered;
}

function replaceSerialConsole(entries) {
  const consoleEl = $("serial-console");
  if (!consoleEl) return;
  consoleEl.innerHTML = "";
  filteredSerialEntries(entries).forEach((entry) => appendSerialEntry(entry));
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
    replaceSerialConsole(serialConsoleEntries);
    serialConsoleLoaded = true;
  } catch (err) {
    $("serial-status").textContent = err instanceof Error ? err.message : String(err);
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
  replaceSerialConsole(serialConsoleEntries);
});
$("serial-filter-m114").addEventListener("change", () => {
  replaceSerialConsole(serialConsoleEntries);
});

socket.on("serial_entry", (entry) => {
  if (!serialConsoleLoaded) return;
  serialConsoleEntries.push(entry);
  replaceSerialConsole(serialConsoleEntries);
});

// ---------- scan + plot ----------
const plotEl = $("plot");
const plotLayout = {
  paper_bgcolor: "#25252a",
  plot_bgcolor:  "#1c1c1f",
  font: { color: "#ddd" },
  margin: { l: 50, r: 20, t: 20, b: 40 },
  xaxis: { title: "Y (mm)", gridcolor: "#333" },
  yaxis: { title: "X contact (mm)", gridcolor: "#333" },
  showlegend: true,
};

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

function buildTraces(rows) {
  const scans = new Map();
  rows.forEach((row) => {
    if (!scans.has(row.scan_id)) scans.set(row.scan_id, []);
    scans.get(row.scan_id).push(row);
  });

  const ordered = Array.from(scans.entries());
  return ordered.flatMap(([scanId, rowsForScan], idx) => {
    const points = rowsForScan
      .slice()
      .sort((a, b) => a.index - b.index)
      .filter((row) => row.x != null && row.y != null);
    if (points.length === 0) return [];
    const age = ordered.length - 1 - idx;
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

async function renderPlotFromCsv() {
  try {
    const res = await fetch("/api/scan/history.csv", { cache: "no-store" });
    const rows = parseCsv(await res.text());
    await Plotly.react(plotEl, buildTraces(rows), plotLayout, { responsive: true });
  } catch (err) {
    $("last-error").textContent = err instanceof Error ? err.message : String(err);
  }
}

socket.on("scan_started", (req) => {
  void req;
  void renderPlotFromCsv();
});

socket.on("scan_point", (pt) => {
  void pt;
  void renderPlotFromCsv();
});

socket.on("scan_complete", () => { void renderPlotFromCsv(); });

$("run-scan").addEventListener("click", async () => {
  const body = {
    x_max: +$("s-xmax").value,
    probe_target_x: +$("s-probe-x").value,
    probe_speed_mm_s: +$("s-probe-speed").value,
    y_max: +$("s-ymax").value,
    y_min: +$("s-ymin").value,
    n_samples: +$("s-n").value,
  };
  const r = await post("/api/scan", body);
  if (!r.ok) $("last-error").textContent = r.error || "scan failed";
});

$("abort-scan").addEventListener("click", () => post("/api/scan/abort"));

$("clear-plot").addEventListener("click", async () => {
  const r = await post("/api/scan/history/clear");
  if (!r.ok) {
    $("last-error").textContent = r.error || "clear failed";
    return;
  }
  await renderPlotFromCsv();
});

Plotly.newPlot(plotEl, [], plotLayout, { responsive: true }).then(() => renderPlotFromCsv());
void refreshSerialStatus();
void refreshSerialConsole();
