// field.js: draws a VRC field and robots on a <canvas>.
//
// Shared by the auton_check report and the live sim dashboard. Plain script,
// no dependencies; exposes window.VexField.
//
// Frame: the VEX GPS / path.jerryio convention. Origin at field centre,
// inches, +x toward the blue alliance station (right), +y up the page,
// heading in degrees clockwise from +y. The red alliance station is on the
// left in every layout here, matching the game manuals' top views.
//
// Element positions come from each game manual's Figure FO-1 and the
// coordinates VEX publishes for its VR playgrounds (converted from mm). They
// are meant to be right to about an inch: good for "does my path hit the
// goal", not a substitute for field specs in Appendix A.
(function () {
  "use strict";

  const TILE = 24;            // in
  const HALF = 72;            // half the 12 ft field
  const RED = "#d8423b", BLUE = "#2f7fd8";

  // ---------------------------------------------------------------- layouts
  const ring = (x, y, c) => ({ x, y, c });

  // High Stakes (2024-25), Head-to-Head match.
  const HS_MATCH = {
    name: "High Stakes",
    tape: [
      // autonomous line: a pair of lines down the middle
      { a: [-1.5, -HALF], b: [-1.5, HALF] }, { a: [1.5, -HALF], b: [1.5, HALF] },
      // starting lines, parallel to each alliance station
      { a: [-58, -HALF], b: [-58, HALF] }, { a: [58, -HALF], b: [58, HALF] },
    ],
    ladder: true,
    goals: [[-24, 24], [24, 24], [-24, -24], [24, -24], [0, -48]].map(([x, y]) => ({ x, y, shape: "hex" })),
    stakes: [
      { x: -HALF, y: 0, c: RED }, { x: HALF, y: 0, c: BLUE },
      { x: 0, y: HALF, c: "#d9b33a" }, { x: 0, y: -HALF, c: "#d9b33a" },
    ],
    corners: [
      { x: -HALF, y: HALF, sign: "−" }, { x: HALF, y: HALF, sign: "−" },
      { x: -HALF, y: -HALF, sign: "+" }, { x: HALF, y: -HALF, sign: "+" },
    ],
    rings: [
      ring(-24, 48, "b"), ring(24, 48, "r"),
      ring(-3.5, 51.5, "b"), ring(-3.5, 44.5, "b"), ring(3.5, 51.5, "r"), ring(3.5, 44.5, "r"),
      ring(-3.5, 3.5, "b"), ring(-3.5, -3.5, "b"), ring(3.5, 3.5, "r"), ring(3.5, -3.5, "r"),
      ring(-58, 0, "b"), ring(-47, 0, "r"), ring(47, 0, "b"), ring(58, 0, "r"),
      ring(-47, -48, "b"), ring(-24, -48, "b"), ring(24, -48, "r"), ring(47, -48, "r"),
      ring(-67, 67, "b"), ring(-67, -67, "b"), ring(67, 67, "r"), ring(67, -67, "r"),
    ],
  };

  // High Stakes Robot Skills layout (manual p.72, VEX VR playground).
  const HS_SKILLS = Object.assign({}, HS_MATCH, {
    name: "High Stakes skills",
    goals: [[-48, 24], [-48, -24], [48, 0], [60, 24], [60, -24]].map(([x, y]) => ({ x, y, shape: "hex" })),
    rings: [
      ...[[-48, 60], [0, 60], [-60, 48], [-48, 48], [24, 48], [-24, 24], [24, 24], [0, 0], [-24, -24],
          [24, -24], [-60, -48], [-48, -48], [24, -48], [-48, -60], [0, -60]].map(([x, y]) => ring(x, y, "r")),
      ...[[48, 60], [48, 48], [60, 48], [48, -48], [60, -48], [48, -60]].map(([x, y]) => ring(x, y, "rb")),
      ring(67, 67, "b"), ring(67, -67, "b"),
    ],
  });

  // Override (2026-27), Head-to-Head match.
  const OV_MATCH = {
    name: "Override",
    tape: [
      // the midfield square, corner-on
      { a: [0, 24], b: [24, 0] }, { a: [24, 0], b: [0, -24] }, { a: [0, -24], b: [-24, 0] }, { a: [-24, 0], b: [0, 24] },
      // diagonals from the corners into the midfield (doubled on one axis)
      { a: [-58.5, 58.5], b: [-12, 12], double: true }, { a: [12, -12], b: [58.5, -58.5], double: true },
      { a: [58.5, 58.5], b: [12, 12] }, { a: [-58.5, -58.5], b: [-12, -12] },
    ],
    zones: [
      // alliance corner zones (coloured tape)
      { c: RED, pts: [[-59, HALF], [-59, 46], [-HALF, 46]] }, { c: RED, pts: [[-59, -HALF], [-59, -46], [-HALF, -46]] },
      { c: BLUE, pts: [[59, HALF], [59, 46], [HALF, 46]] }, { c: BLUE, pts: [[59, -HALF], [59, -46], [HALF, -46]] },
    ],
    goals: [
      ...[[0, 0], [-24, 48], [-48, 24], [48, -24], [24, -48]].map(([x, y]) => ({ x, y, shape: "oct" })),
      ...[[-48, -24], [-24, -48]].map(([x, y]) => ({ x, y, shape: "oct", c: RED })),
      ...[[24, 48], [48, 24]].map(([x, y]) => ({ x, y, shape: "oct", c: BLUE })),
    ],
    toggles: [{ x: 0, y: HALF, h: true }, { x: 0, y: -HALF, h: true }, { x: -HALF, y: 0 }, { x: HALF, y: 0 }],
    loaders: [[-HALF, 58.7], [-HALF, -58.7], [HALF, 58.7], [HALF, -58.7]],
    pins: [[-24, 68.7], [24, 68.7], [-48, 48], [48, 48], [-68.7, 24], [-24, 24], [0, 24], [24, 24], [68.7, 24],
           [-24, 0], [24, 0], [-68.7, -24], [-24, -24], [0, -24], [24, -24], [68.7, -24], [-48, -48], [48, -48],
           [-24, -68.7], [24, -68.7]],
  };

  const BLANK = { name: "Tiles only" };

  const LAYOUTS = { "high-stakes": HS_MATCH, "high-stakes-skills": HS_SKILLS, override: OV_MATCH, blank: BLANK };

  // ---------------------------------------------------------------- view
  // A view maps field inches onto canvas pixels. `extent` is the half-width
  // in inches shown, wide enough to include the alliance stations.
  function view(sizePx, extent) {
    const ext = extent || 82;
    const s = sizePx / (2 * ext);
    return {
      s, size: sizePx, ext,
      px: (x, y) => [sizePx / 2 + x * s, sizePx / 2 - y * s],
      inches: (px, py) => [(px - sizePx / 2) / s, (sizePx / 2 - py) / s],
    };
  }

  // ---------------------------------------------------------------- drawing
  const C = {
    floor: "#0f1114", tile: "#4f535a", seam: "#3f4248", wall: "#c7cbd1", wallEdge: "#8d939b",
    tape: "rgba(245,245,245,0.92)", ring: { r: "#d8423b", b: "#2f7fd8" }, goal: "#d9b33a",
    ladder: "#24272c", ladderRail: "#9aa1aa", toggle: "#d24fa8", pin: "#d9b33a", loader: "#e9ecef",
  };

  function drawField(ctx, V, layoutKey) {
    const L = LAYOUTS[layoutKey] || HS_MATCH;
    const s = V.s;
    const P = V.px;
    ctx.save();
    ctx.fillStyle = C.floor;
    ctx.fillRect(0, 0, V.size, V.size);

    // alliance stations, outside the left and right walls
    for (const [side, col] of [[-1, RED], [1, BLUE]]) {
      const [x0] = P(side * (HALF + 4), 0);
      const [x1] = P(side * (HALF + 7.5), 0);
      const [, yTop] = P(0, 46), [, yBot] = P(0, -46);
      ctx.fillStyle = col;
      ctx.globalAlpha = 0.85;
      ctx.fillRect(Math.min(x0, x1), yTop, Math.abs(x1 - x0), yBot - yTop);
      ctx.globalAlpha = 1;
    }

    // tiles
    const [fx, fy] = P(-HALF, HALF);
    const fw = 2 * HALF * s;
    ctx.fillStyle = C.tile;
    ctx.fillRect(fx, fy, fw, fw);
    ctx.strokeStyle = C.seam;
    ctx.lineWidth = Math.max(1, 0.35 * s);
    for (let i = 1; i < 6; i++) {
      const g = fx + i * TILE * s;
      line(ctx, g, fy, g, fy + fw);
      const h = fy + i * TILE * s;
      line(ctx, fx, h, fx + fw, h);
    }

    // tape
    ctx.strokeStyle = C.tape;
    ctx.lineCap = "butt";
    ctx.lineWidth = Math.max(1, 1 * s);
    for (const t of L.tape || []) {
      if (t.double) {
        const dx = t.b[0] - t.a[0], dy = t.b[1] - t.a[1], n = Math.hypot(dx, dy);
        const ox = (-dy / n) * 1.6, oy = (dx / n) * 1.6;
        for (const k of [-1, 1]) segIn(ctx, P, t.a[0] + k * ox, t.a[1] + k * oy, t.b[0] + k * ox, t.b[1] + k * oy);
      } else {
        segIn(ctx, P, t.a[0], t.a[1], t.b[0], t.b[1]);
      }
    }
    for (const z of L.zones || []) {
      ctx.strokeStyle = z.c;
      ctx.lineWidth = Math.max(1.5, 1.2 * s);
      ctx.beginPath();
      z.pts.forEach(([x, y], i) => { const [px, py] = P(x, y); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
      ctx.stroke();
    }

    // corners (High Stakes): a faint triangle on the tiles, and the +/-
    // sticker where it really is, on the wall outside the corner
    for (const k of L.corners || []) {
      const sx = Math.sign(k.x), sy = Math.sign(k.y);
      ctx.fillStyle = k.sign === "+" ? "rgba(255,255,255,0.10)" : "rgba(0,0,0,0.18)";
      ctx.beginPath();
      ctx.moveTo(...P(k.x, k.y));
      ctx.lineTo(...P(k.x - sx * 15, k.y));
      ctx.lineTo(...P(k.x, k.y - sy * 15));
      ctx.closePath();
      ctx.fill();
      ctx.fillStyle = "rgba(255,255,255,0.8)";
      ctx.font = `700 ${Math.max(10, 4.5 * s)}px ui-monospace, "SF Mono", Menlo, monospace`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(k.sign, ...P(k.x + sx * 5, k.y + sy * 5));
    }

    if (L.ladder) drawLadder(ctx, P, s);

    // goals
    for (const g of L.goals || []) {
      const [px, py] = P(g.x, g.y);
      const r = 5 * s;
      ctx.beginPath();
      const n = g.shape === "hex" ? 6 : 8;
      for (let i = 0; i < n; i++) {
        const a = (Math.PI * 2 * i) / n + (n === 6 ? Math.PI / 6 : Math.PI / 8);
        const x = px + r * Math.cos(a), y = py + r * Math.sin(a);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      }
      ctx.closePath();
      ctx.fillStyle = g.c ? g.c : n === 6 ? C.goal : "#6c7280";
      ctx.globalAlpha = 0.72;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.lineWidth = Math.max(1, 0.5 * s);
      ctx.strokeStyle = "rgba(0,0,0,0.45)";
      ctx.stroke();
      ctx.fillStyle = "rgba(0,0,0,0.55)";
      ctx.beginPath();
      ctx.arc(px, py, 0.9 * s, 0, Math.PI * 2);
      ctx.fill();
    }

    // wall stakes (High Stakes)
    for (const k of L.stakes || []) {
      const [px, py] = P(k.x, k.y);
      ctx.fillStyle = k.c;
      ctx.beginPath();
      ctx.arc(px, py, 2.2 * s, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(0,0,0,0.5)";
      ctx.lineWidth = 1;
      ctx.stroke();
    }

    // rings
    for (const r of L.rings || []) {
      const [px, py] = P(r.x, r.y);
      ctx.lineWidth = 2.2 * s;
      if (r.c === "rb") {
        ctx.strokeStyle = C.ring.r;
        ctx.beginPath(); ctx.arc(px, py, 2.4 * s, Math.PI, 0); ctx.stroke();
        ctx.strokeStyle = C.ring.b;
        ctx.beginPath(); ctx.arc(px, py, 2.4 * s, 0, Math.PI); ctx.stroke();
      } else {
        ctx.strokeStyle = C.ring[r.c];
        ctx.globalAlpha = 0.85;
        ctx.beginPath(); ctx.arc(px, py, 2.4 * s, 0, Math.PI * 2); ctx.stroke();
        ctx.globalAlpha = 1;
      }
    }

    // Override: pins, toggles, loaders
    for (const [x, y] of L.pins || []) {
      const [px, py] = P(x, y);
      ctx.fillStyle = C.pin;
      ctx.globalAlpha = 0.8;
      ctx.beginPath(); ctx.arc(px, py, 1.6 * s, 0, Math.PI * 2); ctx.fill();
      ctx.globalAlpha = 1;
    }
    for (const t of L.toggles || []) {
      const [px, py] = P(t.x + (t.h ? 0 : Math.sign(t.x) * 2.5), t.y + (t.h ? Math.sign(t.y) * 2.5 : 0));
      const long = 28 * s, short = 2.5 * s;
      ctx.fillStyle = C.toggle;
      if (t.h) ctx.fillRect(px - long / 2, py - short / 2, long, short);
      else ctx.fillRect(px - short / 2, py - long / 2, short, long);
    }
    for (const [x, y] of L.loaders || []) {
      const [px, py] = P(x - Math.sign(x) * 2, y);
      ctx.fillStyle = C.loader;
      ctx.globalAlpha = 0.85;
      ctx.fillRect(px - 2 * s, py - 4 * s, 4 * s, 8 * s);
      ctx.globalAlpha = 1;
    }

    // perimeter
    ctx.strokeStyle = C.wall;
    ctx.lineWidth = Math.max(2, 2 * s);
    ctx.strokeRect(fx - s, fy - s, fw + 2 * s, fw + 2 * s);
    ctx.restore();
  }

  function drawLadder(ctx, P, s) {
    const pts = [[0, 24], [24, 0], [0, -24], [-24, 0]];
    ctx.save();
    ctx.beginPath();
    pts.forEach(([x, y], i) => { const [px, py] = P(x, y); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.closePath();
    ctx.fillStyle = "rgba(20,22,26,0.35)";
    ctx.fill();
    ctx.lineWidth = Math.max(2, 2.2 * s);
    ctx.strokeStyle = C.ladderRail;
    ctx.stroke();
    ctx.fillStyle = C.ladder;
    for (const [x, y] of pts) { ctx.beginPath(); ctx.arc(...P(x, y), 2 * s, 0, Math.PI * 2); ctx.fill(); }
    ctx.fillStyle = C.goal;                       // the High Stake on top
    ctx.beginPath(); ctx.arc(...P(0, 0), 1.6 * s, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
  }

  function line(ctx, x0, y0, x1, y1) { ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); }
  function segIn(ctx, P, x0, y0, x1, y1) { const a = P(x0, y0), b = P(x1, y1); line(ctx, a[0], a[1], b[0], b[1]); }

  // A robot footprint: square body, a thick front edge, a heading tick.
  function drawRobot(ctx, V, pose, o) {
    const opt = Object.assign({ w: 15, l: 15, fill: null, stroke: "#fff", width: 1.5, alpha: 1, front: null }, o);
    const [px, py] = V.px(pose[0], pose[1]);
    const s = V.s;
    ctx.save();
    ctx.globalAlpha = opt.alpha;
    ctx.translate(px, py);
    ctx.rotate((pose[2] * Math.PI) / 180);       // compass: clockwise from up
    const w = opt.w * s, l = opt.l * s;
    roundRect(ctx, -w / 2, -l / 2, w, l, Math.min(3 * s, 4));
    if (opt.fill) { ctx.fillStyle = opt.fill; ctx.fill(); }
    if (opt.stroke) { ctx.strokeStyle = opt.stroke; ctx.lineWidth = opt.width; ctx.stroke(); }
    if (opt.front) {
      ctx.strokeStyle = opt.front;
      ctx.lineWidth = Math.max(2, opt.width * 1.8);
      ctx.beginPath(); ctx.moveTo(-w / 2 + 2, -l / 2); ctx.lineTo(w / 2 - 2, -l / 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, -l / 2); ctx.lineTo(0, -l / 2 - 4 * s); ctx.stroke();
    }
    ctx.restore();
  }

  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  // ---------------------------------------------------------------- frames
  // Re-express a pose from a routine's own frame (whatever odom_xyt_set
  // declared) on the field, given where its start pose sits on the field.
  function codeToField(p, codeStart, place) {
    const phi = ((place[2] - codeStart[2]) * Math.PI) / 180;   // clockwise
    const dx = p[0] - codeStart[0], dy = p[1] - codeStart[1];
    return [place[0] + dx * Math.cos(phi) + dy * Math.sin(phi),
            place[1] - dx * Math.sin(phi) + dy * Math.cos(phi),
            p[2] + (place[2] - codeStart[2])];
  }

  // The same routine run from the other alliance: reflect across the field's
  // centre line. EZ autons mirror this way when every turn is `* sgn`.
  const mirror = (p) => [-p[0], p[1], -p[2]];

  // ---------------------------------------------------------------- units
  function fmtLen(inches, units, digits) {
    if (units === "tiles") return (inches / TILE).toFixed(digits == null ? 2 : digits) + "t";
    return inches.toFixed(digits == null ? 1 : digits) + "″";
  }
  const wrapDeg = (a) => { a = ((a + 180) % 360 + 360) % 360 - 180; return a <= -180 ? 180 : a; };

  window.VexField = {
    TILE, HALF, RED, BLUE, LAYOUTS, view, drawField, drawRobot, codeToField, mirror, fmtLen, wrapDeg,
  };
})();
