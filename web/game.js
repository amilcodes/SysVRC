// game.js: what a routine does to the game elements, and what it scores.
//
// Runs over a robot trajectory that's already been simulated (auton_check's
// clean run and its varied runs), so it's cheap enough to redo every time the
// start position is dragged. Mechanisms come from robot.json (see
// docs/robot_spec.md): where the intake grabs, where the clamp holds a goal,
// how far the arm reaches, and which action("...") lines drive them.
//
// High Stakes only for now. Deliberately rough: the robot drives through goals
// and rings instead of pushing them, rings teleport up the conveyor at a fixed
// rate, the arm swings at a fixed speed. Good for "does the clamp fire with a
// goal under it" and "does this score 7 or 3", not for ring physics.
//
// Frame: field inches, +x toward blue, +y up, heading clockwise from +y
// (same as field.js). Robot frame: +x right, +y forward, origin at the centre
// of the drivetrain.
(function (root) {
  "use strict";

  const HALF = 72;
  const RING_ON = 0.2;          // |intake speed| below this doesn't move rings
  const DT = 0.02;              // s
  const GOAL_R = 5;             // mobile goal base, centre to flat-ish (in)
  const RING_R = 3.5;           // ring outer radius (7 in across)

  // ---------------------------------------------------------------- field
  // Starting elements per layout. Rings carry a stack (bottom first).
  const LAYOUTS = {
    "high-stakes": {
      // same stacks as field.js (Figure FO-2), bottom ring first
      rings: [
        [-24, 48, "rb"], [24, 48, "br"],
        [-3.5, 51.5, "rb"], [-3.5, 44.5, "rb"], [3.5, 51.5, "br"], [3.5, 44.5, "br"],
        [-3.5, 3.5, "b"], [-3.5, -3.5, "b"], [3.5, 3.5, "r"], [3.5, -3.5, "r"],
        [-58, 0, "b"], [-47, 0, "br"], [47, 0, "rb"], [58, 0, "r"],
        [-47, -48, "b"], [-24, -48, "rb"], [24, -48, "br"], [47, -48, "r"],
        [-67, 67, "rbrb"], [-67, -67, "rbrb"], [67, 67, "brbr"], [67, -67, "brbr"],
      ],
      goals: [[-24, 24], [24, 24], [-24, -24], [24, -24], [0, -48]],
      stakes: [
        { x: -HALF, y: 0, only: "r", cap: 2 }, { x: HALF, y: 0, only: "b", cap: 2 },
        { x: 0, y: HALF, cap: 6 }, { x: 0, y: -HALF, cap: 6 },
      ],
      corners: [[-HALF, HALF, -1], [HALF, HALF, -1], [-HALF, -HALF, 1], [HALF, -HALF, 1]],
      startingLine: 58,
      match: true,
    },
    "high-stakes-skills": {
      rings: [
        ...[[-48, 60], [0, 60], [-60, 48], [-48, 48], [24, 48], [-24, 24], [24, 24], [0, 0], [-24, -24],
            [24, -24], [-60, -48], [-48, -48], [24, -48], [-48, -60], [0, -60]].map(([x, y]) => [x, y, "r"]),
        ...[[48, 60], [48, 48], [60, 48], [48, -48], [60, -48], [48, -60]].map(([x, y]) => [x, y, "br"]),
        [67, 67, "b"], [67, -67, "b"],
      ],
      goals: [[-48, 24], [-48, -24], [48, 0], [60, 24], [60, -24]],
      stakes: [
        { x: -HALF, y: 0, only: "r", cap: 2 }, { x: HALF, y: 0, only: "b", cap: 2 },
        { x: 0, y: HALF, cap: 6 }, { x: 0, y: -HALF, cap: 6 },
      ],
      corners: [[-HALF, HALF, -1], [HALF, HALF, -1], [-HALF, -HALF, 1], [HALF, -HALF, 1]],
      startingLine: 58,
      match: false,
    },
  };
  const supported = (key) => Object.prototype.hasOwnProperty.call(LAYOUTS, key);

  // ---------------------------------------------------------------- geometry
  function toRobot(pose, x, y) {           // field point -> robot frame
    const th = (pose[2] * Math.PI) / 180, dx = x - pose[0], dy = y - pose[1];
    return [dx * Math.cos(th) - dy * Math.sin(th), dx * Math.sin(th) + dy * Math.cos(th)];
  }
  function toField(pose, rx, ry) {         // robot frame point -> field
    const th = (pose[2] * Math.PI) / 180;
    return [pose[0] + rx * Math.cos(th) + ry * Math.sin(th), pose[1] - rx * Math.sin(th) + ry * Math.cos(th)];
  }
  const inBox = (p, z, pad) => p[0] >= z[0][0] - pad && p[0] <= z[1][0] + pad && p[1] >= z[0][1] - pad && p[1] <= z[1][1] + pad;
  const boxCentre = (z) => [(z[0][0] + z[1][0]) / 2, (z[0][1] + z[1][1]) / 2];
  const normBox = (z) => [[Math.min(z[0][0], z[1][0]), Math.min(z[0][1], z[1][1])], [Math.max(z[0][0], z[1][0]), Math.max(z[0][1], z[1][1])]];

  // Is a goal "Placed" in a corner? Its centre inside the corner triangle
  // (legs of about 17 in, which is where a goal base sits once it's pushed in).
  function cornerOf(L, x, y) {
    for (const [cx, cy, sign] of L.corners) if (Math.abs(cx - x) + Math.abs(cy - y) < 17) return sign;
    return 0;
  }

  // ---------------------------------------------------------------- robot
  // Fill in what robot.json leaves out, so a half-finished spec still runs.
  function compile(spec) {
    spec = spec || {};
    const fp = spec.footprint || {};
    const w = fp.width || 15, l = fp.length || 15;
    const outline = Array.isArray(spec.outline) && spec.outline.length >= 3 ? spec.outline
      : [[-w / 2, -l / 2], [w / 2, -l / 2], [w / 2, l / 2], [-w / 2, l / 2]];
    const mechs = {};
    for (const m of spec.mechanisms || []) if (m && m.id) mechs[m.id] = m;
    const byKind = (k) => Object.values(mechs).find((m) => m.kind === k);

    const intake = byKind("intake");
    const clamp = byKind("goal_clamp");
    const arm = byKind("wall_stake_arm");
    const sorter = byKind("color_sort");
    const R = {
      name: spec.name || "robot", w, l, outline,
      intake: intake && {
        id: intake.id,
        zone: normBox(intake.zone || [[-w / 2 + 2, l / 2 - 2], [w / 2 - 2, l / 2 + 3]]),
        capacity: intake.capacity || 2,
        transfer: intake.transfer_s || 0.6,       // s for a ring to go from the floor to the top
        pickGap: intake.pickup_gap_s || 0.15,
      },
      clamp: clamp && {
        id: clamp.id,
        zone: normBox(clamp.zone || [[-4, -l / 2 - 4], [4, -l / 2 + 2]]),
        closedWhen: clamp.closed_when === undefined ? true : !!clamp.closed_when,
      },
      arm: arm && {
        id: arm.id,
        states: arm.states || {},
        load: arm.load_state || "PROPPED",
        scoreDeg: arm.score_deg || 120,
        dps: arm.deg_per_s || 280,
        reach: normBox(arm.reach || [[-4, l / 2], [4, l / 2 + 11]]),
        start: arm.start,
      },
      sorter: sorter && { id: sorter.id, start: !!sorter.start },
      mechs,
      bindings: [],
    };
    // Explicit bindings first, then the obvious ones from each mechanism's
    // names in the code (intake.move(127), mogoClamp.toggle(), ...).
    for (const b of spec.bindings || []) {
      try { R.bindings.push({ re: new RegExp(b.match), mech: b.mech, op: b.do, value: b.value, scale: b.scale || 1 }); }
      catch (e) { /* a bad regex in robot.json just doesn't bind */ }
    }
    const esc = (s) => s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
    for (const m of Object.values(mechs)) {
      for (const n of m.code_names || []) {
        const N = esc(n);
        if (m.actuator === "pneumatic") {
          R.bindings.push({ re: new RegExp(`^${N}\\.toggle\\(\\)$`), mech: m.id, op: "toggle" });
          R.bindings.push({ re: new RegExp(`^${N}\\.(extend|retract)\\(\\)$`), mech: m.id, op: "set", value: "$1" });
          R.bindings.push({ re: new RegExp(`^${N}\\.set_value\\((.+)\\)$`), mech: m.id, op: "set", value: "$1" });
        } else {
          R.bindings.push({ re: new RegExp(`^${N}\\.move\\((.+)\\)$`), mech: m.id, op: "speed", value: "$1", scale: 127 });
          R.bindings.push({ re: new RegExp(`^${N}\\.move_voltage\\((.+)\\)$`), mech: m.id, op: "speed", value: "$1", scale: 12000 });
          R.bindings.push({ re: new RegExp(`^${N}\\.move_velocity\\((.+)\\)$`), mech: m.id, op: "speed", value: "$1", scale: m.cartridge_rpm || 600 });
          R.bindings.push({ re: new RegExp(`^${N}\\.brake\\(\\)$`), mech: m.id, op: "speed", value: "0" });
        }
      }
    }
    return R;
  }

  // Numbers in action text: "127", "-127", "45 + 10", "12000 * 0.8". Anything
  // fancier isn't a number we can know ahead of time.
  function evalNumber(s) {
    s = String(s).trim();
    if (!/^[-+*/(). \d]+$/.test(s)) return NaN;
    let i = 0;
    const peek = () => s[i], ws = () => { while (s[i] === " ") i++; };
    function atom() {
      ws();
      if (peek() === "(") { i++; const v = expr(); ws(); i++; return v; }
      if (peek() === "-") { i++; return -atom(); }
      if (peek() === "+") { i++; return atom(); }
      const m = /^\d+(\.\d+)?|^\.\d+/.exec(s.slice(i));
      if (!m) return NaN;
      i += m[0].length;
      return parseFloat(m[0]);
    }
    function term() { let v = atom(); for (ws(); peek() === "*" || peek() === "/"; ws()) { const o = s[i++]; const r = atom(); v = o === "*" ? v * r : v / r; } return v; }
    function expr() { let v = term(); for (ws(); peek() === "+" || peek() === "-"; ws()) { const o = s[i++]; const r = term(); v = o === "+" ? v + r : v - r; } return v; }
    const v = expr();
    ws();
    return i === s.length ? v : NaN;
  }

  // action("...") lines -> mechanism events, in time order. `times[k]` is when
  // action k fired in this run (-1 if it never did).
  function events(R, actionTexts, times) {
    const out = [];
    actionTexts.forEach((text, k) => {
      const t = times[k];
      if (!(t >= 0)) return;
      for (const b of R.bindings) {
        const m = b.re.exec(text);
        if (!m) continue;
        let v = b.value === undefined ? undefined : String(b.value).replace(/\$(\d)/g, (_, n) => m[+n] || "");
        out.push({ t, k, text, mech: b.mech, op: b.op, value: v, scale: b.scale });
        break;
      }
    });
    out.sort((a, b) => a.t - b.t || a.k - b.k);
    return out;
  }

  // ---------------------------------------------------------------- the sim
  // poseAt(t) -> [x, y, heading] in field inches. Returns a log of where every
  // element was and when, what each mechanism call did, and the score.
  function run({ layout, robot: R, alliance, poseAt, events: evs, end, budget, preload = true }) {
    const L = LAYOUTS[layout];
    if (!L) return null;
    const us = alliance === "blue" ? "b" : "r";
    const side = us === "b" ? 1 : -1;

    // elements ------------------------------------------------------------
    const rings = [];   // { c, track: [{t, at, x, y, ref}] }
    const stacks = L.rings.map(([x, y, cs]) => {
      const ids = [];
      for (const c of cs) { ids.push(rings.length); rings.push({ c, track: [{ t: 0, at: "field", x, y }] }); }
      return { x, y, ids };          // bottom first; the top is ids[ids.length - 1]
    });
    const goals = L.goals.map(([x, y]) => ({ x, y, rings: [], held: false, track: [{ t: 0, held: false, x, y }] }));
    const stakes = L.stakes.map((s) => ({ ...s, rings: [] }));
    const put = (id, t, at, x, y, ref) => rings[id].track.push({ t, at, x, y, ref });

    // robot ---------------------------------------------------------------
    const st = {
      intake: 0, queue: [],          // queue: { id, p } p = 0 floor .. 1 top
      lastPick: -1, until: 0, untilSeen: 0,
      clampClosed: false, goal: -1,
      armDeg: 0, armTarget: 0, armRing: -1, armScoredThisSwing: false,
      sort: !!(R.sorter && R.sorter.start), flags: {},
    };
    const outcome = {};              // action index -> { ok, text }
    let intakeEvent = -1;            // the call that last started the intake
    const picked = {};               // action index -> rings picked while it was in charge

    const armAngle = (v) => {
      if (!R.arm) return 0;
      if (v in R.arm.states) return R.arm.states[v];
      const n = evalNumber(v);
      return Number.isFinite(n) ? n : null;
    };

    function apply(ev, t, pose) {
      const set = (ok, text) => { outcome[ev.k] = { ok, text }; };
      const mech = R.mechs[ev.mech] || {};
      if (R.intake && ev.mech === R.intake.id) {
        if (ev.op === "speed") {
          const n = evalNumber(ev.value);
          if (!Number.isFinite(n)) { set(null, "speed unknown"); return; }
          st.intake = Math.max(-1, Math.min(1, n / (ev.scale || 1)));
          if (st.intake > RING_ON) { intakeEvent = ev.k; picked[ev.k] = 0; set(null, "intake on"); }
          else set(null, st.intake < -RING_ON ? "outtake" : "intake off");
        } else if (ev.op === "until") {
          st.until = Math.max(1, evalNumber(ev.value) || 1); st.untilSeen = 0; st.intake = 1;
          intakeEvent = ev.k; picked[ev.k] = 0; set(null, `hold ${st.until}`);
        } else if (ev.op === "release") { st.until = 0; set(null, ""); }
        return;
      }
      if (R.clamp && ev.mech === R.clamp.id) {
        let on = st.clampClosed;
        if (ev.op === "toggle") on = !on;
        else if (ev.op === "set") {
          const v = String(ev.value).trim();
          const ext = v === "extend" || v === "true" || v === "1" || v === "HIGH";
          on = ext === R.clamp.closedWhen;
        }
        if (on && !st.clampClosed) {
          // grab the nearest free goal under the clamp
          const c = boxCentre(R.clamp.zone);
          let best = -1, bestD = Infinity, nearest = Infinity;
          goals.forEach((g, i) => {
            if (g.held) return;
            const p = toRobot(pose, g.x, g.y);
            const d = Math.hypot(p[0] - c[0], p[1] - c[1]);
            nearest = Math.min(nearest, d);
            if (inBox(p, R.clamp.zone, 1.0) && d < bestD) { best = i; bestD = d; }
          });
          if (best >= 0) {
            st.goal = best; goals[best].held = true;
            set(true, "clamped");
          } else {
            set(false, nearest < 30 ? `missed, goal ${nearest.toFixed(1)}″ away` : "nothing to clamp");
          }
        } else if (!on && st.clampClosed) {
          if (st.goal >= 0) {
            const g = goals[st.goal];
            g.held = false;
            g.track.push({ t, held: false, x: g.x, y: g.y });
            const k = cornerOf(L, g.x, g.y);
            set(null, k > 0 ? "dropped in + corner" : k < 0 ? "dropped in − corner" : "released");
          } else set(null, "opened");
          st.goal = -1;
        } else set(null, on ? "already closed" : "already open");
        st.clampClosed = on;
        return;
      }
      if (R.arm && ev.mech === R.arm.id) {
        if (ev.op === "state" || ev.op === "angle") {
          const a = armAngle(ev.value);
          if (a == null) { set(null, "angle unknown"); return; }
          st.armTarget = a;
          if (ev.reset) st.armDeg = a;
          st.armEvent = ev.k;
          set(null, a >= R.arm.scoreDeg ? (st.armRing >= 0 ? "swinging" : "empty") : "");
        } else if (ev.op === "speed") {
          const n = evalNumber(ev.value);
          if (Number.isFinite(n)) st.armTarget = n > 0 ? 200 : 0;
        }
        return;
      }
      if (R.sorter && ev.mech === R.sorter.id) {
        if (ev.op === "set") st.sort = /^(true|1|on)$/.test(String(ev.value).trim());
        else if (ev.op === "toggle") st.sort = !st.sort;
        else if (ev.op === "until") { st.until = Math.max(1, evalNumber(ev.value) || 1); st.untilSeen = 0; if (R.intake) { st.intake = 1; intakeEvent = ev.k; picked[ev.k] = 0; } }
        else if (ev.op === "release") st.until = 0;
        set(null, "");
        return;
      }
      // doinkers, lifts and anything else: remembered, no effect on elements
      if (ev.op === "toggle") st.flags[ev.mech] = !st.flags[ev.mech];
      else if (ev.op === "set") st.flags[ev.mech] = /^(true|1|extend|HIGH)$/.test(String(ev.value).trim());
      set(null, mech.kind || "");
    }

    // Calls before the robot has moved (the first tick or two) describe how
    // it's set up, e.g. `LBState = PROPPED` for the preload: they apply
    // instantly. Then the preload goes in the arm if it starts at its load
    // angle, else in the intake.
    const p0 = poseAt(0);
    if (R.arm && R.arm.start !== undefined) {
      const a0 = armAngle(String(R.arm.start));
      if (a0 != null) st.armDeg = st.armTarget = a0;
    }
    let evi = 0;
    while (evi < evs.length && evs[evi].t <= 0.03) apply({ ...evs[evi], reset: true }, 0, p0), evi++;
    if (R.arm) st.armDeg = st.armTarget;
    if (preload) {
      const id = rings.length;
      rings.push({ c: us, preload: true, track: [{ t: 0, at: "robot" }] });
      const loadDeg = R.arm ? armAngle(R.arm.load) : null;
      if (R.arm && loadDeg != null && Math.abs(st.armDeg - loadDeg) < 15) { st.armRing = id; put(id, 0, "arm"); }
      else if (R.intake) st.queue.push({ id, p: 0.5 });
    }

    const flung = (id, t, pose, why) => {
      const [x, y] = toField(pose, 0, -R.l / 2 - 6);
      put(id, t, "out", x, y, why);
    };

    const stopAt = Math.max(end, 0);
    for (let t = DT; t <= stopAt + 1e-9; t += DT) {
      const pose = poseAt(t);
      while (evi < evs.length && evs[evi].t <= t) apply(evs[evi++], t, pose);

      // The chassis shoves loose goals out of its way, except into the clamp
      // opening, where a goal slides in until it's seated: that's how a robot
      // backing into a goal lines it up before the clamp fires.
      for (let gi = 0; gi < goals.length; gi++) {
        const g = goals[gi];
        if (g.held) continue;
        const p = toRobot(pose, g.x, g.y);
        const hx = R.w / 2 + GOAL_R, hy = R.l / 2 + GOAL_R;
        if (Math.abs(p[0]) >= hx || Math.abs(p[1]) >= hy) continue;
        let q = null;
        const z = R.clamp && R.clamp.zone, c = z && boxCentre(z);
        const inSlot = z && p[0] >= z[0][0] && p[0] <= z[1][0] && Math.sign(p[1]) === Math.sign(c[1]);
        if (inSlot) {
          // seat it at the clamp, no further
          if (Math.abs(p[1]) < Math.abs(c[1])) q = [p[0], c[1]];
        } else {
          const ox = hx - Math.abs(p[0]), oy = hy - Math.abs(p[1]);
          q = ox < oy ? [Math.sign(p[0] || 1) * hx, p[1]] : [p[0], Math.sign(p[1] || 1) * hy];
        }
        if (!q) continue;
        let [nx, ny] = toField(pose, q[0], q[1]);
        const lim = HALF - GOAL_R;
        nx = Math.max(-lim, Math.min(lim, nx));
        ny = Math.max(-lim, Math.min(lim, ny));
        if (Math.hypot(nx - g.x, ny - g.y) > 0.05) { g.x = nx; g.y = ny; g.track.push({ t, held: false, x: nx, y: ny }); }
      }

      // held goal rides along
      if (st.goal >= 0) {
        const c = boxCentre(R.clamp.zone);
        const [gx, gy] = toField(pose, c[0], c[1]);
        const g = goals[st.goal];
        g.x = gx; g.y = gy;
        g.track.push({ t, held: true, x: gx, y: gy });
      }

      if (R.intake) {
        // pick up: the top ring of any stack under the intake
        if (st.intake > RING_ON && st.queue.length < R.intake.capacity && t - st.lastPick >= R.intake.pickGap) {
          for (const s of stacks) {
            if (!s.ids.length) continue;
            // the intake gets a ring as soon as any of it is in the mouth
            if (!inBox(toRobot(pose, s.x, s.y), R.intake.zone, RING_R * 0.7)) continue;
            const id = s.ids.pop();
            st.queue.push({ id, p: 0 });
            put(id, t, "robot");
            st.lastPick = t;
            if (intakeEvent >= 0) picked[intakeEvent] = (picked[intakeEvent] || 0) + 1;
            break;
          }
        }
        // conveyor
        if (Math.abs(st.intake) > RING_ON) {
          const dp = (st.intake * DT) / R.intake.transfer;
          for (let i = 0; i < st.queue.length; i++) {
            const q = st.queue[i];
            q.p = Math.min(1, q.p + dp);
            if (q.p <= 0) {                              // spat out the front
              const [x, y] = toField(pose, 0, R.l / 2 + 4);
              const s = { x, y, ids: [q.id] };
              stacks.push(s);
              put(q.id, t, "field", x, y);
              st.queue.splice(i--, 1);
            }
          }
        }
        // the ring at the top goes somewhere
        const top = st.queue[0];
        if (top && top.p >= 1) {
          const c = rings[top.id].c;
          const loadDeg = R.arm ? armAngle(R.arm.load) : null;
          if (st.until > 0 && c === us) {
            st.untilSeen++;
            if (st.untilSeen >= st.until) { st.intake = 0; st.until = 0; }   // stop and keep it
            top.p = 0.999;
          } else if (R.arm && st.armRing < 0 && loadDeg != null && Math.abs(st.armDeg - loadDeg) < 15) {
            st.armRing = top.id; put(top.id, t, "arm"); st.queue.shift();
          } else if (st.sort && c !== us) {
            flung(top.id, t, pose, "sorted"); st.queue.shift();
          } else if (st.goal >= 0 && goals[st.goal].rings.length < 6) {
            goals[st.goal].rings.push(top.id); put(top.id, t, "goal", 0, 0, st.goal); st.queue.shift();
          } else if (st.goal < 0) {
            flung(top.id, t, pose, "no goal"); st.queue.shift();
          } else {
            flung(top.id, t, pose, "goal full"); st.queue.shift();
          }
        }
      }

      if (R.arm) {
        const step = R.arm.dps * DT;
        const before = st.armDeg;
        st.armDeg += Math.max(-step, Math.min(step, st.armTarget - st.armDeg));
        if (before < R.arm.scoreDeg && st.armDeg >= R.arm.scoreDeg && st.armRing >= 0) {
          // swinging up through the stake: is one in reach?
          let hit = -1;
          stakes.forEach((s, i) => {
            if (hit < 0 && inBox(toRobot(pose, s.x, s.y), R.arm.reach, 1.0)) hit = i;
          });
          const ev = st.armEvent;
          if (hit >= 0 && stakes[hit].rings.length < stakes[hit].cap) {
            stakes[hit].rings.push(st.armRing); put(st.armRing, t, "stake", stakes[hit].x, stakes[hit].y, hit);
            if (ev != null) outcome[ev] = { ok: true, text: hit < 2 ? "alliance stake" : "wall stake" };
          } else {
            const [x, y] = toField(pose, 0, R.l / 2 + 6);
            put(st.armRing, t, "out", x, y, "missed stake");
            if (ev != null) outcome[ev] = { ok: false, text: "no stake in reach" };
          }
          st.armRing = -1;
        }
      }
    }

    for (const [k, n] of Object.entries(picked)) {
      const o = outcome[k] || { ok: null, text: "" };
      outcome[k] = { ok: n > 0 ? true : o.ok, text: n > 0 ? `+${n} ring${n > 1 ? "s" : ""}` : o.text === "intake on" ? "no rings" : o.text };
    }

    const result = { layout, alliance: us, rings, goals, stakes, outcome, robot: R, end: stopAt };
    result.final = scoreAt(result, Math.min(budget || stopAt, stopAt), poseAt);
    return result;
  }

  // ---------------------------------------------------------------- state & score
  function ringAt(ring, t) {
    let s = ring.track[0];
    for (const e of ring.track) { if (e.t <= t + 1e-9) s = e; else break; }
    return s;
  }
  function goalAt(goal, t) {
    // tracks are appended in time order; binary search the last entry <= t
    const tr = goal.track;
    let lo = 0, hi = tr.length - 1;
    while (lo < hi) { const m = (lo + hi + 1) >> 1; if (tr[m].t <= t + 1e-9) lo = m; else hi = m - 1; }
    return tr[lo];
  }

  // Everything drawable at time t: loose rings, goals with their rings, stakes.
  function stateAt(res, t) {
    const loose = new Map();           // "x,y" -> { x, y, cs: [] }
    const goals = res.goals.map((g) => { const s = goalAt(g, t); return { x: s.x, y: s.y, held: s.held, rings: [] }; });
    const stakes = res.stakes.map((s) => ({ x: s.x, y: s.y, rings: [] }));
    const inRobot = [];
    let armRing = null;
    for (const r of res.rings) {
      const s = ringAt(r, t);
      if (s.at === "field") {
        const key = s.x.toFixed(2) + "," + s.y.toFixed(2);
        if (!loose.has(key)) loose.set(key, { x: s.x, y: s.y, cs: [] });
        loose.get(key).cs.push(r.c);
      } else if (s.at === "goal") goals[s.ref].rings.push(r.c);
      else if (s.at === "stake") stakes[s.ref].rings.push(r.c);
      else if (s.at === "robot") inRobot.push(r.c);
      else if (s.at === "arm") armRing = r.c;
    }
    // ring order on a goal/stake = order they were scored
    const order = (list, kind) => {
      list.forEach((el, i) => {
        const ids = res.rings.map((r, id) => [r, id]).filter(([r]) => { const s = ringAt(r, t); return s.at === kind && s.ref === i; });
        ids.sort((a, b) => ringAt(a[0], t).t - ringAt(b[0], t).t);
        el.rings = ids.map(([r]) => r.c);
      });
    };
    order(goals, "goal");
    order(stakes, "stake");
    return { loose: [...loose.values()], goals, stakes, inRobot, armRing };
  }

  // Points for both colours at time t, plus the auton win point checklist.
  function scoreAt(res, t, poseAt) {
    const L = LAYOUTS[res.layout];
    const S = stateAt(res, t);
    const pts = { r: 0, b: 0 }, neg = { r: 0, b: 0 };
    const counted = { r: 0, b: 0 };
    const stakeHas = { r: [], b: [] };   // stakes with >= 1 ring of that colour: positions
    const value = (cs, mult) => {
      const v = { r: 0, b: 0 };
      cs.forEach((c, i) => { v[c] += (i === cs.length - 1 ? 3 : 1) * mult; });
      return v;
    };
    S.goals.forEach((g) => {
      if (!g.rings.length) return;
      const corner = g.held ? 0 : cornerOf(L, g.x, g.y);
      const v = value(g.rings, corner > 0 ? 2 : 1);
      for (const c of ["r", "b"]) {
        if (corner < 0) neg[c] += v[c]; else pts[c] += v[c];
        if (g.rings.includes(c)) stakeHas[c].push(g.x);
      }
      g.rings.forEach((c) => counted[c]++);
    });
    S.stakes.forEach((s, i) => {
      const own = L.stakes[i].only;
      const cs = own ? s.rings.filter((c) => c === own) : s.rings;
      if (!cs.length) return;
      const v = value(cs, 1);
      for (const c of ["r", "b"]) { pts[c] += v[c]; if (cs.includes(c)) stakeHas[c].push(s.x); }
      cs.forEach((c) => counted[c]++);
    });
    for (const c of ["r", "b"]) pts[c] = Math.max(0, pts[c] - neg[c]);

    const us = res.alliance, side = us === "b" ? 1 : -1;
    const out = { us: pts[us], them: pts[us === "b" ? "r" : "b"], rings: counted[us], points: pts };
    if (L.match && poseAt) {
      const pose = poseAt(t);
      const pts2 = res.robot.outline.map(([x, y]) => toField(pose, x, y));
      const xs = pts2.map((p) => p[0] * side);
      const onLadder = pts2.some(([x, y]) => Math.abs(x) + Math.abs(y) <= 25) ||
        (res.robot.arm && [[0, res.robot.arm.reach[1][1]]].map(([x, y]) => toField(pose, x, y)).some(([x, y]) => Math.abs(x) + Math.abs(y) <= 25));
      out.awp = {
        rings: counted[us] >= 3,
        stakes: stakeHas[us].filter((x) => x * side > 0).length >= 2,
        line: !(Math.min(...xs) <= L.startingLine + 1 && Math.max(...xs) >= L.startingLine - 1),
        ladder: onLadder,
      };
      out.awp.all = out.awp.rings && out.awp.stakes && out.awp.line && out.awp.ladder;
    }
    return out;
  }

  const api = { LAYOUTS, supported, compile, events, run, stateAt, scoreAt, evalNumber, toRobot, toField };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.VexGame = api;
})(typeof window !== "undefined" ? window : globalThis);
