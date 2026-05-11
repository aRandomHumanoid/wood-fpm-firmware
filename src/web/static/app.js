// Wood FPM frontend.
//
// Live state via Socket.IO; Plotly traces persist between scans with prior
// runs faded so trends stay visible.

const socket = io({ transports: ["websocket", "polling"] });

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

// ---------- live state ----------
socket.on("state", (s) => {
  setVal("pos-x", s.x);
  setVal("pos-y", s.y);
  setVal("pos-z", s.z);
  setVal("pos-th", s.theta_deg);
  setVal("pos-ph", s.phi_deg);
  setLed("probe-led", s.probe, false);
  setLed("homed-led", s.homed, false);
  setLed("busy-led",  s.busy,  false);
  setLed("fault-led", s.fault, s.fault);
  $("last-error").textContent = s.last_error || "";
});

// ---------- limits (one-shot) ----------
fetch("/api/limits").then(r => r.json()).then((L) => {
  $("lim-x").textContent = `${L.bounds.x_min} … ${L.bounds.x_max}`;
  $("lim-y").textContent = `${L.bounds.y_min} … ${L.bounds.y_max}`;
  setVal("lim-v", L.v_max_mm_s);
  setVal("lim-a", L.a_max_mm_s2);
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

document.querySelectorAll(".jog").forEach((b) => {
  b.addEventListener("click", () => {
    const step = parseFloat($("jog-step").value) || 0;
    const dx = parseFloat(b.dataset.dx) * step;
    const dy = parseFloat(b.dataset.dy) * step;
    post("/api/jog", { dx, dy });
  });
});

$("home").addEventListener("click", () => post("/api/home"));
$("estop").addEventListener("click", () => post("/api/stop"));

// ---------- scan + plot ----------
const plotEl = $("plot");
Plotly.newPlot(
  plotEl,
  [],
  {
    paper_bgcolor: "#25252a",
    plot_bgcolor:  "#1c1c1f",
    font: { color: "#ddd" },
    margin: { l: 50, r: 20, t: 20, b: 40 },
    xaxis: { title: "X (mm)", gridcolor: "#333" },
    yaxis: { title: "Y contact (mm)", gridcolor: "#333" },
    showlegend: true,
  },
  { responsive: true }
);

let activeScan = null;   // { scan_id, traceIdx, xs, ys }
let traceCount = 0;

function fadeOlderTraces() {
  const traces = plotEl.data;
  if (!traces || traces.length === 0) return;
  const update = { opacity: [] };
  const indices = [];
  traces.forEach((_, i) => {
    indices.push(i);
    const age = traces.length - 1 - i;     // 0 = newest
    const op = age === 0 ? 1.0 : Math.max(0.1, 0.5 - 0.08 * age);
    update.opacity.push(op);
  });
  Plotly.restyle(plotEl, update, indices);
}

socket.on("scan_started", (req) => {
  traceCount += 1;
  const name = `scan ${traceCount}`;
  Plotly.addTraces(plotEl, [{
    x: [], y: [],
    mode: "lines+markers",
    name,
    line:    { width: 2 },
    marker:  { size: 6 },
  }]);
  activeScan = {
    scan_id: req.scan_id,
    traceIdx: plotEl.data.length - 1,
    xs: [],
    ys: [],
  };
  fadeOlderTraces();
});

socket.on("scan_point", (pt) => {
  if (!activeScan || pt.scan_id !== activeScan.scan_id) return;
  if (pt.y == null) return;   // skip non-contacts in the plot
  activeScan.xs.push(pt.x);
  activeScan.ys.push(pt.y);
  Plotly.extendTraces(
    plotEl,
    { x: [[pt.x]], y: [[pt.y]] },
    [activeScan.traceIdx]
  );
});

socket.on("scan_complete", () => { activeScan = null; });

$("run-scan").addEventListener("click", async () => {
  const body = {
    x_min: +$("s-xmin").value,
    x_max: +$("s-xmax").value,
    y_max: +$("s-ymax").value,
    y_min: +$("s-ymin").value,
    n_samples: +$("s-n").value,
  };
  const r = await post("/api/scan", body);
  if (!r.ok) $("last-error").textContent = r.error || "scan failed";
});

$("abort-scan").addEventListener("click", () => post("/api/scan/abort"));

$("clear-plot").addEventListener("click", () => {
  Plotly.deleteTraces(plotEl, plotEl.data.map((_, i) => i));
  traceCount = 0;
});
