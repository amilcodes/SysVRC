// HTML template for auton_check --html. Self-contained: no CDN, no network,
// opens from a file on any laptop at a competition. auton_check replaces the
// REPORT_DATA marker below with the run's JSON.
#pragma once

inline const char* const kReportTemplate = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>auton check</title>
<style>
  .viz-root {
    color-scheme: light;
    --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
    --grid: #e1e0d9; --axis: #c3c2b7; --ring: rgba(11,11,11,0.10);
    --s1: #2a78d6; --s2: #eb6834; --s3: #1baf7a;
    --good: #0ca30c; --serious: #ec835a; --critical: #d03b3b;
    --path: rgba(82,81,78,0.18);
  }
  @media (prefers-color-scheme: dark) {
    :root:where(:not([data-theme="light"])) .viz-root {
      color-scheme: dark;
      --page: #0d0d0d; --surface: #1a1a19; --ink: #ffffff; --ink-2: #c3c2b7; --muted: #898781;
      --grid: #2c2c2a; --axis: #383835; --ring: rgba(255,255,255,0.10);
      --s1: #3987e5; --s2: #d95926; --s3: #199e70; --path: rgba(195,194,183,0.16);
    }
  }
  :root[data-theme="dark"] .viz-root {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --ink: #ffffff; --ink-2: #c3c2b7; --muted: #898781;
    --grid: #2c2c2a; --axis: #383835; --ring: rgba(255,255,255,0.10);
    --s1: #3987e5; --s2: #d95926; --s3: #199e70; --path: rgba(195,194,183,0.16);
  }
  * { box-sizing: border-box; }
  body { margin: 0; }
  .viz-root { background: var(--page); color: var(--ink); min-height: 100vh;
    font: 14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; padding: 24px clamp(16px, 4vw, 40px) 48px; }
  .wrap { max-width: 1120px; margin: 0 auto; }
  h1 { font-size: 22px; margin: 0 0 2px; font-weight: 650; letter-spacing: -0.01em; }
  .sub { color: var(--ink-2); margin: 0 0 20px; }
  h2 { font-size: 13px; font-weight: 600; margin: 0 0 10px; color: var(--ink-2); }
  .card { background: var(--surface); border: 1px solid var(--ring); border-radius: 10px; padding: 16px; }
  .tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin-bottom: 16px; }
  .tile .k { color: var(--ink-2); font-size: 12px; }
  .tile .v { font-size: 28px; font-weight: 650; margin-top: 2px; }
  .tile .n { color: var(--muted); font-size: 12px; margin-top: 2px; }
  .grid2 { display: grid; grid-template-columns: minmax(0, 1.25fr) minmax(0, 1fr); gap: 16px; margin-bottom: 16px; }
  @media (max-width: 860px) { .grid2 { grid-template-columns: 1fr; } }
  svg { display: block; width: 100%; height: auto; overflow: visible; }
  svg text { fill: var(--muted); font: 11px system-ui, -apple-system, "Segoe UI", sans-serif; }
  svg .lbl { fill: var(--ink-2); }
  svg .halo { fill: var(--ink); paint-order: stroke; stroke: var(--surface); stroke-width: 4px; stroke-linejoin: round; font-weight: 600; }
  .legend { display: flex; flex-wrap: wrap; gap: 14px; font-size: 12px; color: var(--ink-2); margin: 0 0 8px; }
  .legend i { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 6px; vertical-align: -1px; }
  .legend i.line { width: 16px; height: 2px; border-radius: 1px; vertical-align: 3px; }
  .note { color: var(--muted); font-size: 12px; margin: 8px 0 0; }
  .tablewrap { overflow-x: auto; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; }
  th { text-align: left; font-weight: 600; color: var(--ink-2); font-size: 12px; padding: 6px 8px; border-bottom: 1px solid var(--axis); white-space: nowrap; cursor: pointer; }
  td { padding: 6px 8px; border-bottom: 1px solid var(--grid); vertical-align: middle; }
  td.num, th.num { text-align: right; font-variant-numeric: tabular-nums; }
  td code { font: 12px ui-monospace, "SF Mono", Menlo, monospace; color: var(--ink); }
  tr.sel td { background: color-mix(in srgb, var(--s3) 12%, transparent); }
  tbody tr { cursor: pointer; }
  tbody tr:hover td { background: color-mix(in srgb, var(--ink) 4%, transparent); }
  .bar { height: 6px; border-radius: 3px; background: var(--s1); min-width: 2px; }
  .flag { display: flex; gap: 8px; align-items: baseline; padding: 6px 0; border-bottom: 1px solid var(--grid); }
  .flag:last-child { border-bottom: 0; }
  .flag b { color: var(--ink); font-weight: 600; }
  .icon { width: 14px; height: 14px; flex: none; align-self: center; }
  #tip { position: fixed; pointer-events: none; background: var(--surface); color: var(--ink); border: 1px solid var(--ring);
    border-radius: 6px; padding: 6px 8px; font-size: 12px; box-shadow: 0 4px 16px rgba(0,0,0,0.12); display: none; z-index: 10; max-width: 320px; }
  .stack > * + * { margin-top: 16px; }
  .warn { display: flex; gap: 10px; align-items: flex-start; border-left: 3px solid var(--serious); margin-bottom: 16px; }
  .warn .icon { margin-top: 2px; }
</style>
</head>
<body>
<div class="viz-root">
<div class="wrap">
  <h1 id="title"></h1>
  <p class="sub" id="subtitle"></p>
  <div id="warnings"></div>
  <div class="tiles" id="tiles"></div>

  <div class="grid2">
    <div class="card">
      <h2 id="fieldTitle">Where the robot went</h2>
      <div class="legend" id="fieldLegend"></div>
      <svg id="field" role="img" aria-label="Paths and end points of every run"></svg>
      <p class="note">Click a mechanism call in the table to show where it fired in every run.</p>
    </div>
    <div class="stack">
      <div class="card">
        <h2>When the routine finished</h2>
        <svg id="hist" role="img" aria-label="Histogram of finish times"></svg>
      </div>
      <div class="card">
        <h2>What moves the end point most (p95, one factor at a time)</h2>
        <svg id="sens" role="img" aria-label="End-point spread caused by each disturbance on its own"></svg>
      </div>
      <div class="card">
        <h2>What makes it late (added to p95 finish time, one factor at a time)</h2>
        <svg id="late" role="img" aria-label="Finish time added by each disturbance on its own"></svg>
      </div>
    </div>
  </div>

  <div class="card" style="margin-bottom:16px" id="fragileCard" hidden>
    <h2>Motions that sometimes give up instead of reaching their target</h2>
    <div id="fragile"></div>
  </div>

  <div class="card" style="margin-bottom:16px">
    <h2>Mechanism calls: distance from where the nominal run fired them</h2>
    <div class="tablewrap"><table id="actions"></table></div>
  </div>

  <div class="card">
    <h2>What was varied between runs</h2>
    <div class="tablewrap"><table id="dist"></table></div>
    <p class="note">These are assumptions, not measurements. Set them to match your robot with the flags
      shown; run <code>auton_check --help</code> for the list.</p>
  </div>
</div>
<div id="tip"></div>
</div>
<script>
const D = /*REPORT_DATA*/null;
const $ = (id) => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";
const f1 = (v) => (v == null ? "-" : Number(v).toFixed(1));
const f2 = (v) => (v == null ? "-" : Number(v).toFixed(2));
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

function el(tag, attrs = {}, parent) {
  const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (parent) parent.appendChild(e);
  return e;
}
const tip = $("tip");
function showTip(ev, html) {
  tip.innerHTML = html; tip.style.display = "block";
  const w = tip.offsetWidth, h = tip.offsetHeight;
  tip.style.left = Math.min(window.innerWidth - w - 8, ev.clientX + 12) + "px";
  tip.style.top = Math.max(8, ev.clientY - h - 12) + "px";
}
function hideTip() { tip.style.display = "none"; }
function hover(node, html) {
  node.addEventListener("mousemove", (ev) => showTip(ev, typeof html === "function" ? html() : html));
  node.addEventListener("mouseleave", hideTip);
}
// Rounded top corners only, anchored to the baseline.
function barPath(x, y, w, h, r) {
  r = Math.min(r, w / 2, h);
  if (h <= 0) return "";
  return `M${x},${y + h}V${y + r}Q${x},${y} ${x + r},${y}H${x + w - r}Q${x + w},${y} ${x + w},${y + r}V${y + h}Z`;
}

// ------------------------------------------------------------------ header
const limit = D.budget;
$("title").textContent = D.name;
$("subtitle").textContent = `${D.runs} runs with the robot placed, charged and built a little differently each time. ` +
  `Compared against one undisturbed run. Seed ${D.seed}, source ${D.source}.`;
document.title = `${D.name} · auton check`;

const warnIcon = `<svg class="icon" viewBox="0 0 16 16" aria-hidden="true"><path d="M8 1.5 15 14H1z" fill="var(--serious)"/><path d="M8 6v4" stroke="#0b0b0b" stroke-width="1.6" stroke-linecap="round"/><circle cx="8" cy="12" r=".9" fill="#0b0b0b"/></svg>`;
$("warnings").innerHTML = (D.warnings || []).map((w) =>
  `<div class="card warn">${warnIcon}<div><b>Heads up.</b> ${esc(w)}</div></div>`).join("");

const worst = D.actions.slice().sort((a, b) => b.p95 - a.p95)[0];
const tiles = [
  { k: `Finishes inside ${limit} s`, v: `${Number(D.time.on_time_pct).toFixed(1)}%`,
    n: D.over_at ? `when over, usually still on ${D.over_at}` : `of ${D.runs} runs` },
  { k: "Finish time, p95", v: `${f2(D.time.p95)} s`, n: `nominal ${f2(D.nominal.motion_end)} s, worst ${f2(D.time.max)} s` },
  { k: "End point spread, p95", v: `${f1(D.end.p95)} in`, n: `heading p95 ${f1(D.end.heading_p95)}°` },
];
if (worst) tiles.push({ k: "Least repeatable call, p95", v: `${f1(worst.p95)} in`, n: worst.label + (worst.line ? ` (line ${worst.line})` : "") });
$("tiles").innerHTML = tiles.map((t) =>
  `<div class="card tile"><div class="k">${esc(t.k)}</div><div class="v">${esc(t.v)}</div><div class="n">${esc(t.n)}</div></div>`).join("");

// ------------------------------------------------------------------ histogram
(function histogram() {
  const svg = $("hist"), W = 520, H = 200, m = { l: 34, r: 12, t: 22, b: 26 };
  svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
  const xs = D.time.all;
  const lo = Math.floor(Math.min(...xs, D.nominal.motion_end) * 2) / 2;
  const hi = Math.ceil(Math.max(...xs, limit) * 2) / 2 + 0.5;
  const nb = 36, bw = (hi - lo) / nb;
  const bins = new Array(nb).fill(0);
  for (const x of xs) bins[Math.min(nb - 1, Math.floor((x - lo) / bw))]++;
  const ymax = Math.max(...bins);
  const X = (v) => m.l + ((v - lo) / (hi - lo)) * (W - m.l - m.r);
  const Y = (v) => H - m.b - (v / ymax) * (H - m.t - m.b);
  // grid + y ticks
  for (const t of [0, Math.round(ymax / 2), ymax]) {
    el("line", { x1: m.l, x2: W - m.r, y1: Y(t), y2: Y(t), stroke: "var(--grid)" }, svg);
    el("text", { x: m.l - 6, y: Y(t) + 4, "text-anchor": "end" }, svg).textContent = t;
  }
  // bars
  const step = (W - m.l - m.r) / nb;
  bins.forEach((c, i) => {
    const x = m.l + i * step + 1, w = Math.max(1, step - 2);
    const hit = el("rect", { x: m.l + i * step, y: m.t, width: step, height: H - m.t - m.b, fill: "transparent" }, svg);
    if (c) {
      const p = el("path", { d: barPath(x, Y(c), w, H - m.b - Y(c), 2), fill: "var(--s1)" }, svg);
      p.style.pointerEvents = "none";
    }
    hover(hit, `<b>${f2(lo + i * bw)}–${f2(lo + (i + 1) * bw)} s</b><br>${c} run${c === 1 ? "" : "s"}`);
  });
  el("line", { x1: m.l, x2: W - m.r, y1: H - m.b, y2: H - m.b, stroke: "var(--axis)" }, svg);
  // x ticks
  const tickStep = hi - lo > 20 ? 5 : hi - lo > 8 ? 2 : 1;
  for (let t = Math.ceil(lo / tickStep) * tickStep; t <= hi; t += tickStep)
    el("text", { x: X(t), y: H - 8, "text-anchor": "middle" }, svg).textContent = `${t}s`;
  // limit + nominal markers, labelled in text ink
  const lx = X(limit);
  el("line", { x1: lx, x2: lx, y1: m.t - 6, y2: H - m.b, stroke: "var(--critical)", "stroke-width": 1.5, "stroke-dasharray": "4 3" }, svg);
  el("text", { x: lx + 4, y: m.t - 10, "text-anchor": "start", class: "lbl" }, svg).textContent = `${limit} s limit`;
  const nx = X(D.nominal.motion_end);
  el("line", { x1: nx, x2: nx, y1: m.t - 6, y2: H - m.b, stroke: "var(--ink)", "stroke-width": 1.5 }, svg);
  const close = Math.abs(nx - lx) < 70;
  el("text", { x: nx + (close ? -4 : 4), y: m.t - 10, "text-anchor": close ? "end" : "start", class: "lbl" }, svg)
    .textContent = `nominal ${f2(D.nominal.motion_end)} s`;
  if (close) el("text", { x: lx + 4, y: m.t - 10, "text-anchor": "start", class: "lbl" }, svg);
})();

// ------------------------------------------------------------------ sensitivity
// One horizontal bar chart per question, each on its own scale (never one
// chart with two axes). Bars are rounded at the data end only.
function hbars(svgId, rows, value, label, tipText) {
  const svg = $(svgId);
  if (!rows.length) { svg.closest(".card").remove(); return; }
  const W = 520, rh = 24, m = { l: 132, r: 116, t: 4, b: 4 };
  const H = m.t + m.b + rows.length * rh;
  svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
  const vmax = Math.max(...rows.map(value), 0.01);
  rows.forEach((r, i) => {
    const y = m.t + i * rh, v = Math.max(0, value(r)), w = (v / vmax) * (W - m.l - m.r);
    el("text", { x: m.l - 8, y: y + rh / 2 + 4, "text-anchor": "end", class: "lbl" }, svg).textContent = r.factor;
    const h = 10, top = y + (rh - h) / 2, rr = Math.min(4, w / 2);
    if (w > 0.5)
      el("path", { d: `M${m.l},${top}H${m.l + w - rr}Q${m.l + w},${top} ${m.l + w},${top + rr}V${top + h - rr}Q${m.l + w},${top + h} ${m.l + w - rr},${top + h}H${m.l}Z`, fill: "var(--s1)" }, svg);
    el("text", { x: m.l + w + 6, y: y + rh / 2 + 4 }, svg).textContent = label(r);
    const hit = el("rect", { x: 0, y, width: W, height: rh, fill: "transparent" }, svg);
    hover(hit, tipText(r));
  });
  el("line", { x1: m.l, x2: m.l, y1: 0, y2: H, stroke: "var(--axis)" }, svg);
}
hbars("sens", D.sensitivity, (r) => r.p95, (r) => `${f2(r.p95)} in`,
  (r) => `<b>${esc(r.factor)}</b><br>only this varied: end point within ${f2(r.p95)} in of nominal in 95% of runs`);
hbars("late", D.sensitivity.slice().sort((a, b) => b.late_s - a.late_s), (r) => r.late_s,
  (r) => `+${f2(Math.max(0, r.late_s))} s · ${Number(r.on_time_pct).toFixed(0)}% on time`,
  (r) => `<b>${esc(r.factor)}</b><br>only this varied: p95 finish ${f2(r.time_p95)} s ` +
         `(${r.late_s >= 0 ? "+" : ""}${f2(r.late_s)} s vs nominal), on time in ${Number(r.on_time_pct).toFixed(1)}% of runs`);

// ------------------------------------------------------------------ field
let selected = worst ? D.actions.indexOf(worst) : -1;
function drawField() {
  const svg = $("field");
  svg.innerHTML = "";
  const pts = [...D.nominal.path, ...D.end.points, ...D.paths.flat()];
  if (D.walls) pts.push([-72, -72], [72, 72]);
  const act = selected >= 0 ? D.actions[selected] : null;
  if (act) pts.push(...act.points, act.nominal);
  let x0 = Math.min(...pts.map((p) => p[0])), x1 = Math.max(...pts.map((p) => p[0]));
  let y0 = Math.min(...pts.map((p) => p[1])), y1 = Math.max(...pts.map((p) => p[1]));
  const span = Math.max(x1 - x0, y1 - y0, 12) * 1.12;
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  x0 = cx - span / 2; x1 = cx + span / 2; y0 = cy - span / 2; y1 = cy + span / 2;
  const S = 560, pad = 26;
  svg.setAttribute("viewBox", `0 0 ${S} ${S}`);
  const X = (v) => pad + ((v - x0) / (x1 - x0)) * (S - 2 * pad);
  const Y = (v) => S - pad - ((v - y0) / (y1 - y0)) * (S - 2 * pad);
  // 12 in grid (half a field tile), labelled
  const g0x = Math.ceil(x0 / 12) * 12, g0y = Math.ceil(y0 / 12) * 12;
  for (let g = g0x; g <= x1; g += 12) {
    el("line", { x1: X(g), x2: X(g), y1: pad, y2: S - pad, stroke: "var(--grid)" }, svg);
    el("text", { x: X(g), y: S - 8, "text-anchor": "middle" }, svg).textContent = g;
  }
  for (let g = g0y; g <= y1; g += 12) {
    el("line", { x1: pad, x2: S - pad, y1: Y(g), y2: Y(g), stroke: "var(--grid)" }, svg);
    el("text", { x: pad - 6, y: Y(g) + 4, "text-anchor": "end" }, svg).textContent = g;
  }
  if (D.walls)
    el("rect", { x: X(-72), y: Y(72), width: X(72) - X(-72), height: Y(-72) - Y(72), fill: "none",
                 stroke: "var(--axis)", "stroke-width": 3 }, svg);
  const line = (path, attrs) => el("polyline", Object.assign({
    points: path.map((p) => `${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join(" "),
    fill: "none", "stroke-linejoin": "round", "stroke-linecap": "round" }, attrs), svg);
  for (const p of D.paths) line(p, { stroke: "var(--path)", "stroke-width": 1.5 });
  line(D.nominal.path, { stroke: "var(--s1)", "stroke-width": 2 });
  // start marker
  const st = D.nominal.path[0];
  if (st) el("circle", { cx: X(st[0]), cy: Y(st[1]), r: 5, fill: "var(--surface)", stroke: "var(--ink)", "stroke-width": 2 }, svg);
  // end points
  const g = el("g", {}, svg);
  D.end.points.forEach((p, i) => {
    const c = el("circle", { cx: X(p[0]), cy: Y(p[1]), r: 3.5, fill: "var(--s2)", stroke: "var(--surface)", "stroke-width": 1.5 }, g);
    hover(c, () => `<b>run ${i + 1} ended</b><br>(${f1(p[0])}, ${f1(p[1])}) in<br>` +
      `${f2(Math.hypot(p[0] - D.nominal.end[0], p[1] - D.nominal.end[1]))} in from nominal`);
  });
  const ne = D.nominal.end;
  el("circle", { cx: X(ne[0]), cy: Y(ne[1]), r: 6, fill: "none", stroke: "var(--ink)", "stroke-width": 2 }, svg);
  // selected mechanism call
  if (act) {
    const ga = el("g", {}, svg);
    for (const p of act.points)
      el("circle", { cx: X(p[0]), cy: Y(p[1]), r: 3.5, fill: "var(--s3)", stroke: "var(--surface)", "stroke-width": 1.5 }, ga);
    const nx = X(act.nominal[0]), ny = Y(act.nominal[1]);
    el("circle", { cx: nx, cy: ny, r: 6, fill: "none", stroke: "var(--ink)", "stroke-width": 2 }, svg);
    const t = el("text", { x: nx + 10, y: ny - 10, class: "halo" }, svg);
    t.textContent = `${act.label}  p95 ${f1(act.p95)} in`;
    if (nx + 10 + t.getComputedTextLength() > S - 4) { t.setAttribute("x", nx - 10); t.setAttribute("text-anchor", "end"); }
  }
  $("fieldLegend").innerHTML =
    `<span><i class="line" style="background:var(--s1)"></i>nominal path</span>` +
    `<span><i class="line" style="background:var(--muted)"></i>disturbed paths (first ${D.paths.length})</span>` +
    `<span><i style="background:var(--s2)"></i>where each run finished</span>` +
    (act ? `<span><i style="background:var(--s3)"></i>${esc(act.label)} fired here</span>` : "") +
    `<span><i style="background:transparent;border:2px solid var(--ink);width:10px;height:10px"></i>nominal position</span>`;
}

// ------------------------------------------------------------------ actions table
let sortKey = "p95", sortDir = -1;
function drawActions() {
  const t = $("actions");
  const rows = D.actions.map((a, i) => ({ ...a, i }));
  rows.sort((a, b) => sortDir * ((a[sortKey] > b[sortKey]) - (a[sortKey] < b[sortKey])));
  const vmax = Math.max(...rows.map((r) => r.p95), 0.01);
  const cols = [["label", "call"], ["line", "line", 1], ["t", "fires at", 1], ["p50", "p50", 1], ["p95", "p95", 1], ["max", "worst", 1]];
  t.innerHTML = `<thead><tr>${cols.map(([k, n, num]) =>
    `<th data-k="${k}" class="${num ? "num" : ""}">${n}${sortKey === k ? (sortDir < 0 ? " ↓" : " ↑") : ""}</th>`).join("")}<th></th></tr></thead>` +
    `<tbody>${rows.map((r) => `<tr data-i="${r.i}" class="${r.i === selected ? "sel" : ""}">
      <td><code>${esc(r.label)}</code></td><td class="num">${r.line || "-"}</td><td class="num">${f2(r.t)} s</td>
      <td class="num">${f2(r.p50)} in</td><td class="num">${f2(r.p95)} in</td><td class="num">${f2(r.max)} in</td>
      <td style="width:120px"><div class="bar" style="width:${(r.p95 / vmax) * 100}%"></div></td></tr>`).join("")}</tbody>`;
  t.querySelectorAll("th[data-k]").forEach((th) => th.onclick = () => {
    const k = th.dataset.k;
    sortDir = sortKey === k ? -sortDir : (k === "label" || k === "line" || k === "t" ? 1 : -1);
    sortKey = k; drawActions();
  });
  t.querySelectorAll("tbody tr").forEach((tr) => tr.onclick = () => {
    selected = Number(tr.dataset.i); drawActions(); drawField();
  });
}
if (!D.actions.length) $("actions").outerHTML = `<p class="note">This routine has no mechanism calls.</p>`;

// ------------------------------------------------------------------ fragile motions
if (D.fragile.length) {
  $("fragileCard").hidden = false;
  const icon = warnIcon;
  $("fragile").innerHTML = D.fragile.map((f) =>
    `<div class="flag">${icon}<span><b>${f1(f.pct)}% of runs</b> · <code>${esc(f.what)}</code>` +
    `${f.line ? ` at line ${f.line}` : ""} ended on a velocity/timeout exit (EZ's <code>interfered</code>)</span></div>`).join("");
}

// ------------------------------------------------------------------ disturbances
const d = D.disturbances;
$("dist").innerHTML = `<thead><tr><th>what</th><th class="num">size</th><th>flag</th></tr></thead><tbody>` + [
  ["placement, x/y", `${f2(d.place_xy)} in (1σ)`, "--place-xy"],
  ["placement, heading", `${f2(d.place_deg)}° (1σ)`, "--place-deg"],
  ["battery (speed vs full)", `${f2(d.battery[0])}–${f2(d.battery[1])}`, "--battery"],
  ["left/right drivetrain mismatch", `${Number(d.mismatch).toFixed(3)} (1σ)`, "--mismatch"],
  ["wheel slip", `${Number(d.slip[0]).toFixed(3)}–${Number(d.slip[1]).toFixed(3)}`, "--slip"],
  ["gyro drift", `${Number(d.drift).toFixed(3)} °/s (1σ)`, "--drift"],
  ["motor response time", `${f2(d.tau[0])}–${f2(d.tau[1])} s`, "--tau"],
].map((r) => `<tr><td>${r[0]}</td><td class="num">${r[1]}</td><td><code>${r[2]}</code></td></tr>`).join("") + `</tbody>`;

$("fieldTitle").textContent = D.walls ? "Where the robot went (field inches, walls drawn)"
                                      : "Where the robot went (inches, in the routine's own frame)";
drawField();
if (D.actions.length) drawActions();
</script>
</body>
</html>
)HTML";
