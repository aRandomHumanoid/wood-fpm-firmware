const plotEl = document.getElementById("plot");
const statusEl = document.getElementById("status");
const plotOptions = { responsive: true, displaylogo: false };
const plotLayout = {
  paper_bgcolor: "#21242b",
  plot_bgcolor: "#16171b",
  font: { color: "#e9edf4" },
  margin: { l: 60, r: 20, t: 20, b: 50 },
  xaxis: { title: "Y (mm)", gridcolor: "#313641" },
  yaxis: { title: "X contact (mm)", gridcolor: "#313641" },
  showlegend: true,
};

let lastVersion = null;
let refreshInFlight = false;

function formatError(err) {
  return err instanceof Error ? err.message : String(err);
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

function updateStatus(text) {
  statusEl.textContent = text;
}

function describePlot(rows) {
  const scanIds = new Set(rows.map((row) => row.scan_id));
  const updatedAt = new Date().toLocaleTimeString();
  return `${scanIds.size} scan${scanIds.size === 1 ? "" : "s"} loaded • updated ${updatedAt}`;
}

async function fetchVersion() {
  const res = await fetch("/api/scan/history/version", { cache: "no-store" });
  if (!res.ok) {
    throw new Error(`version request failed: ${res.status}`);
  }
  return res.json();
}

async function refreshPlot(force = false) {
  if (refreshInFlight) return;
  refreshInFlight = true;
  try {
    const version = await fetchVersion();
    if (!force && lastVersion && version.mtime_ns === lastVersion.mtime_ns && version.size === lastVersion.size) {
      return;
    }

    const res = await fetch("/api/scan/history.csv", { cache: "no-store" });
    if (!res.ok) {
      throw new Error(`history request failed: ${res.status}`);
    }
    const rows = parseCsv(await res.text());
    await Plotly.react(plotEl, buildTraces(rows), plotLayout, plotOptions);
    lastVersion = version;
    updateStatus(rows.length ? describePlot(rows) : "Watching scan history... no points yet");
  } catch (err) {
    updateStatus(`Refresh failed: ${formatError(err)}`);
  } finally {
    refreshInFlight = false;
  }
}

void Plotly.newPlot(plotEl, [], plotLayout, plotOptions).then(() => refreshPlot(true));
window.setInterval(() => {
  void refreshPlot(false);
}, 500);