// node --test web/tests
// The High Stakes element sim in web/game.js, on hand-made trajectories.
"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const path = require("node:path");

const G = require(path.join(__dirname, "..", "game.js"));
global.window = {};
require(path.join(__dirname, "..", "field.js"));
const F = global.window.VexField;

// A robot that faces +y and drives along waypoints [t, x, y, heading].
function track(points) {
  return (t) => {
    if (t <= points[0][0]) return points[0].slice(1);
    for (let i = 1; i < points.length; i++) {
      const [t0, x0, y0, h0] = points[i - 1], [t1, x1, y1, h1] = points[i];
      if (t <= t1) { const u = (t - t0) / (t1 - t0); return [x0 + (x1 - x0) * u, y0 + (y1 - y0) * u, h0 + (h1 - h0) * u]; }
    }
    return points[points.length - 1].slice(1);
  };
}
const SPEC = {
  footprint: { width: 15, length: 15 },
  mechanisms: [
    { id: "intake", kind: "intake", code_names: ["intake"], zone: [[-5, 6.5], [5, 11]], capacity: 2, transfer_s: 0.5 },
    { id: "clamp", kind: "goal_clamp", actuator: "pneumatic", code_names: ["clamp"], zone: [[-4, -11.5], [4, -5.5]] },
    { id: "lb", kind: "wall_stake_arm", states: { REST: 0, LOAD: 27, SCORE: 177 }, load_state: "LOAD", score_deg: 150,
      deg_per_s: 400, reach: [[-4, 7], [4, 17]] },
  ],
  bindings: [{ match: "^lb\\((\\w+)\\)$", mech: "lb", do: "state", value: "$1" }],
};
const R = G.compile(SPEC);
const run = (o) => G.run({ layout: "high-stakes", robot: R, alliance: "red", budget: 15, preload: false, ...o });
const evs = (list) => G.events(R, list.map((x) => x[1]), list.map((x) => x[0]));

test("the sim's starting layout is the one field.js draws", () => {
  for (const key of ["high-stakes", "high-stakes-skills"]) {
    const a = G.LAYOUTS[key].rings.map(([x, y, c]) => `${x},${y},${c}`).sort();
    const b = F.LAYOUTS[key].rings.map((r) => `${r.x},${r.y},${r.c}`).sort();
    assert.deepEqual(a, b, key);
    assert.deepEqual(G.LAYOUTS[key].goals.map(String).sort(), F.LAYOUTS[key].goals.map((g) => `${g.x},${g.y}`).sort());
  }
  // the manual: 44 rings on the field at the start of a match (48 with preloads)
  assert.equal(G.LAYOUTS["high-stakes"].rings.reduce((n, r) => n + r[2].length, 0), 44);
});

test("numbers in action text", () => {
  assert.equal(G.evalNumber("127"), 127);
  assert.equal(G.evalNumber("-127"), -127);
  assert.equal(G.evalNumber("45 + 10"), 55);
  assert.equal(G.evalNumber("12000 * (1 - 0.25)"), 9000);
  assert.ok(Number.isNaN(G.evalNumber("isBlue ? 127 : -127")));
});

test("bindings: code names and explicit patterns", () => {
  const e = evs([[1, "intake.move(127)"], [2, "clamp.toggle()"], [3, "lb(SCORE)"], [4, "wave()"]]);
  assert.deepEqual(e.map((x) => [x.mech, x.op]), [["intake", "speed"], ["clamp", "toggle"], ["lb", "state"]]);
});

test("driving over a ring picks it up only with the intake running", () => {
  // the stack of one red ring at (58, 0)... use the single blue at (-58, 0): drive through it, facing +y
  const path = track([[0, -58, -20, 0], [2, -58, 10, 0]]);
  const off = run({ poseAt: path, events: [], end: 2 });
  assert.equal(off.final.rings, 0);
  assert.ok(G.stateAt(off, 2).loose.some((r) => r.x === -58 && r.y === 0));
  const on = run({ poseAt: path, events: evs([[0.01, "intake.move(127)"]]), end: 2 });
  assert.ok(!G.stateAt(on, 2).loose.some((r) => r.x === -58 && r.y === 0));
  assert.equal(on.outcome[0].text, "+1 ring");
  // with no goal clamped, a hook conveyor flings it off the top
  const ring = on.rings.find((r) => r.track[0].x === -58 && r.track[0].y === 0);
  assert.deepEqual(ring.track.map((e) => e.at), ["field", "robot", "out"]);
  assert.equal(ring.track[2].ref, "no goal");
});

test("backing into a goal seats it in the clamp; it rides along and scores", () => {
  // goal at (-24, -24); robot faces +y (clamp at the back) and backs down onto it, clamps,
  // then intakes the 2-stack at (-24, -48)? It can't: wrong way. Instead feed it a preload.
  const path = track([[0, -24, 0, 0], [1.5, -24, -16, 0], [3, -24, 0, 0]]);
  const res = G.run({ layout: "high-stakes", robot: R, alliance: "red", budget: 15, preload: true, poseAt: path,
                      events: evs([[1.6, "clamp.toggle()"], [1.7, "intake.move(127)"]]), end: 4 });
  assert.equal(res.outcome[0].ok, true, JSON.stringify(res.outcome[0]));
  const s = G.stateAt(res, 4);
  const held = s.goals.find((g) => g.held);
  assert.ok(held, "goal is held");
  assert.ok(Math.abs(held.y - (0 - 8.5)) < 1.5, `goal rides at the clamp, y=${held.y}`);
  assert.deepEqual(held.rings, ["r"]);                     // the preload went up the conveyor onto it
  assert.equal(res.final.us, 3);                           // a lone ring is the top ring
});

test("a clamp that fires with no goal under it says how far off it was", () => {
  const res = run({ poseAt: track([[0, -40, 0, 0], [1, -40, -5, 0]]), events: evs([[1, "clamp.toggle()"]]), end: 1 });
  assert.equal(res.outcome[0].ok, false);
  assert.match(res.outcome[0].text, /missed|nothing/);
});

test("scoring: top ring is 3, the rest 1; corners double or cancel", () => {
  const res = run({ poseAt: () => [0, 0, 0], events: [], end: 0 });
  // put rings on goals by hand: goal 0 gets r, r, r (5 points for red)
  const put = (gi, colours, t = 0) => colours.forEach((c) => {
    res.rings.push({ c, track: [{ t, at: "goal", ref: gi, x: 0, y: 0 }] });
  });
  put(0, ["r", "r", "r"]);
  assert.equal(G.scoreAt(res, 1).points.r, 5);
  // move goal 0 into a positive corner (bottom left) -> doubled
  res.goals[0].track.push({ t: 2, held: false, x: -66, y: -66 });
  assert.equal(G.scoreAt(res, 3).points.r, 10);
  // a second goal in a negative corner takes its value off the rest
  put(1, ["r"], 4);
  res.goals[1].track.push({ t: 4, held: false, x: 66, y: 66 });
  assert.equal(G.scoreAt(res, 5).points.r, 10 - 3);
  // blue on top of red: red ring is 1, blue top is 3
  put(2, ["r", "b"], 6);
  const p = G.scoreAt(res, 7).points;
  assert.equal(p.b, 3);
  assert.equal(p.r, 7 + 1);
});

test("the lady brown scores the preload on the alliance stake when it's in reach", () => {
  // red alliance stake at (-72, 0); robot faces it (heading -90) with its front 8 in away
  const at = [-72 + 7.5 + 8, 0, -90];
  const ok = G.run({ layout: "high-stakes", robot: R, alliance: "red", budget: 15, preload: true, poseAt: () => at,
                     events: evs([[0.008, "lb(LOAD)"], [0.5, "lb(SCORE)"]]), end: 2 });
  assert.equal(ok.outcome[1].ok, true, JSON.stringify(ok.outcome));
  assert.equal(G.stateAt(ok, 2).stakes[0].rings.length, 1);
  assert.equal(ok.final.us, 3);
  const far = G.run({ layout: "high-stakes", robot: R, alliance: "red", budget: 15, preload: true, poseAt: () => [-30, 0, -90],
                      events: evs([[0.008, "lb(LOAD)"], [0.5, "lb(SCORE)"]]), end: 2 });
  assert.equal(far.outcome[1].ok, false);
});

test("auton win point checklist", () => {
  // parked touching the ladder, off the starting line, nothing scored
  const res = run({ poseAt: () => [-20, 0, 90], events: [], end: 1 });
  assert.equal(res.final.awp.ladder, true);
  assert.equal(res.final.awp.line, true);
  assert.equal(res.final.awp.rings, false);
  assert.equal(res.final.awp.all, false);
  // straddling the starting line
  const line = run({ poseAt: () => [-58, 30, 0], events: [], end: 1 });
  assert.equal(line.final.awp.line, false);
});
