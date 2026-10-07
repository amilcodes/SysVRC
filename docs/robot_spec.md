# robot.json

One robot, as the sim needs it. Written by the [robot studio](robot_studio.md), fine to edit by hand. `robots/ours/robot.json` is ours.

Read by `core/include/visbot/robot_spec.hpp` (the drivetrain, for auton_check, the plant and the controller) and `web/game.js` (mechanisms and bindings, for the element sim). The schema itself is `SCHEMA` in `tools/robot_studio/spec.py`; that's what the AI fills in and what files are validated against.

**Robot frame**: inches, origin on the floor under the middle of the drivetrain, +x right, +y forward, +z up. Zones are boxes seen from above, `[[x0, y0], [x1, y1]]`.

## Drivetrain

```json
"footprint": {"width": 15.0, "length": 15.0, "height": 18.0},
"outline": [[-7.5, -7.5], [7.5, -7.5], [7.5, 7.5], [-7.5, 7.5]],
"mass_lb": 16.5,
"drive": {"type": "tank", "wheel_diameter": 3.25, "wheels_per_side": 3, "motors_per_side": 3,
          "motor": "11W", "cartridge_rpm": 600, "ratio": 0.75, "wheel_rpm": 450,
          "track_width": 12.5, "wheelbase": 11.0},
"code": {"wheel_diameter": 3.25, "wheel_rpm": 450, "cartridge_rpm": 600, "where": "globals.cpp:48"}
```

* `drive` is the robot as built. `ratio` is wheel turns per motor turn (36T driving 48T = 0.75). `wheel_rpm` is `cartridge_rpm * ratio` and is filled in if missing.
* `code` is what your code tells the drive library. The controller's odometry uses it, and the plant uses `drive`. When they differ, odometry drifts exactly the way it would on the robot. Leave `code` out if they match.
* `tau_s` (optional) is the drive's response time. Without it, it's estimated from mass, motor count and top speed (0.12 s for a 15 lb six-motor 450 rpm drive; heavier, faster or fewer motors means slower).
* `outline` is the top-view shape drawn in the report and the dashboard. Without it, a `width x length` box.

## Mechanisms

```json
{"id": "intake", "kind": "intake", "code_names": ["intake"], "zone": [[-5, 6.5], [5, 11]],
 "capacity": 2, "transfer_s": 0.6},
{"id": "clamp", "kind": "goal_clamp", "actuator": "pneumatic", "code_names": ["mogoClamp"],
 "closed_when": true, "zone": [[-4, -11.5], [4, -5.5]]},
{"id": "ladybrown", "kind": "wall_stake_arm", "code_names": ["ladybrown1", "ladybrown2"],
 "states": {"REST": 0, "PROPPED": 27, "EXTENDED": 177}, "load_state": "PROPPED",
 "score_deg": 150, "deg_per_s": 300, "reach": [[-4, 7], [4, 17]]}
```

| kind | what the element sim does with it |
|---|---|
| `intake` | Takes the top ring of any stack under `zone` while running forward, holding up to `capacity`. Each ring takes `transfer_s` to reach the top, then goes to the arm if it's at `load_state`, otherwise onto the clamped goal, otherwise flung. Reverse spits rings back out the front. |
| `goal_clamp` | On closing, holds the goal whose stake is in `zone` (the chassis pushes goals into it on the way). `closed_when` is the piston value that means clamped. |
| `wall_stake_arm` | `states` are arm angles in degrees. Swinging up through `score_deg` with a ring scores it on any wall stake in `reach`. |
| `color_sort` | `do: set` turns flinging the wrong colour on and off. `do: until` holds the next N alliance rings in the intake, then stops it. |
| `doinker`, `lift`, `hang`, `flywheel`, `catapult`, `other` | Tracked but no effect on elements yet. |

`code_names` are the objects that drive it. Plain calls on them bind automatically: `obj.move(n)`, `obj.move_voltage(n)` and `obj.move_velocity(n)` set the speed, and `obj.toggle()`, `obj.extend()`, `obj.retract()` and `obj.set_value(b)` drive a piston.

## Bindings

For everything else, like team helpers and state machines:

```json
{"match": "^setIntake\\((.+)\\)$", "mech": "intake", "do": "speed", "value": "$1", "scale": 127},
{"match": "^ChangeLBState\\((\\w+)\\)$", "mech": "ladybrown", "do": "state", "value": "$1"}
```

`match` is a JavaScript regex over the text inside `action("...")`, `value` a literal or `$1`-style capture. `do` is one of:

* `speed`: value / `scale`, -1..1
* `toggle`
* `set`: true/false/extend/retract
* `state` or `angle`: a name from `states`, or degrees
* `until`, `release`

A value like `45 + 10` is evaluated; anything that depends on the robot at runtime (`isBlue ? 127 : -127`) isn't.

Calls that happen in the first couple of ticks (`LBState = PROPPED` before the routine moves) describe how the robot starts and apply instantly. `"preload": true` (the default) puts one alliance ring in the arm if it starts at its load angle, otherwise in the intake.

## Also there

`name`, `schema`, `source` (which CAD and code it came from, and the frame the studio found), `evidence` (where each value came from: `cad`, `code`, `both` or `guess`), `questions` (what to check), and `confidence`. The sim ignores them; they're for whoever edits the file next.
