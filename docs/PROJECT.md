# Robotic arm — ROS 2 project record

The working document for this arm's ROS 2 stack. Read this first.

It says what exists, what state it is in, what was found wrong and how it was
fixed, and what still needs measuring. It is the counterpart to
`~/Downloads/DS51150_test/START_HERE.md`, which covers the firmware and
electronics; this one covers everything from `/joint_states` upwards.

**Last updated:** 2026-11-16

---

## 1. What this is

A 6-DOF articulated arm with a single-finger gripper, driven by ROS 2 Jazzy and
MoveIt 2. Current capability: **plan a collision-free path and move to a goal
pose on the real hardware**. More capabilities to follow.

| | |
|---|---|
| ROS | Jazzy on Ubuntu 24.04.5 |
| Robot model | derived from a Fusion 360 export |
| Reach | tool0 radius 0.64 m, height −0.03 to 0.97 m, at the current joint limits |
| Execution | real hardware (Mega + ESP32), or `mock_components/GenericSystem` |

### Joint → actuator map

Verified on the arm 2026-09-18 (§4.14). The authoritative record is
`arm_description/config/actuators.yaml`.

| Joint | Role | Actuator | Board | Feedback, **as actually wired** | direction |
|---|---|---|---|---|---|
| `joint1` | base / waist | DS51150 **#1** | Mega 2560, PWM D9 | pot → ADS1115 **0x4A** | +1 |
| `joint2` | shoulder | DS51150 **#2** | Mega 2560, PWM D10 | pot → ADS1115 **0x48** | +1 |
| `joint3` | elbow | DS51150 **#3** | Mega 2560, PWM D11 | pot → ADS1115 **0x49** | +1 |
| `joint4` | wrist roll | ST3215 id 1 | Waveshare ESP32 | internal encoder | **−1** |
| `joint5` | wrist pitch | ST3215 id 2 | Waveshare ESP32 | internal encoder | **−1** |
| `joint6` | tool roll | ST3215 id 3 | Waveshare ESP32 | internal encoder | **−1** |
| `Slider_39` | gripper finger | N20 + lead screw | Mega → DRV8871 | INA219 current (planned) | — |

The pot column differs from the design (0x48 / 0x49 / 0x4A for base / shoulder /
elbow): the three pot cables are rotated one position on the sensing board.

The DS51150s are **physically marked #1/#2/#3 and are not interchangeable** —
their calibration constants differ. Assignment is by backlash, lowest where the
lever arm is longest. See `DS51150_test/docs/SERVOS.md`.

The gripper is a lead screw turned by the N20, moving one finger against a fixed
one. **Not in use yet** — no controller commands it, and the hardware interface
keeps it on loopback (`use_fake_gripper:=true`) even when the arm is real.

### Boards currently connected

| Device | Port | Stable path |
|---|---|---|
| Arduino Mega 2560 | `/dev/ttyACM0` | `/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_85339313233351213111-if00` |
| Waveshare ESP32 (CP2102) | `/dev/ttyUSB0` | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |

Use the `by-id` paths, not `ttyACM0`/`ttyUSB0`. The numbers are assigned in
enumeration order and will swap the first time the boards are plugged in in a
different order — at which point the arm takes wrist commands on its base servos.

---

## 2. Packages

```
src/
  arm_description/      URDF/xacro, meshes, ros2_control, joint limits
  arm_hardware/         the ros2_control hardware interface for both boards
  arm_moveit_config/    SRDF, kinematics, planners, collision matrix, RViz
  arm_demos/            move_to, cartesian_to: plan and execute from the CLI
firmware/               ESP32 wrist firmware, and the board's stock backup
tools/                  bench and bring-up scripts that do not need ROS
docs/PROJECT.md         this file
```

The Fusion export is kept untouched as the CAD record; `arm_description` is a
copy with fixes, and every hand edit is listed in
`arm_description/CAD_PROVENANCE.md`.

**One rule worth keeping:** joint limits live in exactly one file,
`arm_description/config/joint_parameters.yaml`. The xacro reads it, and
`arm_moveit_config/config/joint_limits.yaml` mirrors it for MoveIt. Never edit a
limit anywhere else.

---

## 3. How to run it

```bash
colcon build --symlink-install && source install/setup.bash

# full stack: description + controllers + MoveIt + RViz
ros2 launch arm_moveit_config demo.launch.py

# just look at the model, with a slider per joint
ros2 launch arm_description display.launch.py

# move it
ros2 run arm_demos move_to --named ready
ros2 run arm_demos move_to --named home
ros2 run arm_demos move_to --pose 0.25 0.0 0.25          # tool0, approach down
ros2 run arm_demos move_to --pose 0.25 0.0 0.25 --plan-only
ros2 run arm_demos move_to --joints 0.2 -0.6 1.7 -0.2 -0.4 -1.7

# checks
colcon test --packages-select arm_description && colcon test-result --verbose
ros2 control list_controllers
```

In RViz: the **MotionPlanning** panel, planning group `arm`. Drag the orange
interactive marker to a goal, then Plan, then Execute.

### Verified working

With the captured joint zero:

- Controllers activate: `joint_state_broadcaster`, `arm_controller`,
  `gripper_controller`, all `active`.
- `home` is an upright vertical column, shoulder z=0.159 to tool z=0.970,
  collision-free.
- Joint-space goals plan and execute: `home` ↔ `ready`, about 9 s.
- **8 of 8** top-down Cartesian goals across the front of the workspace plan and
  execute, 3.3–10.2 s.
- IK benchmark: 60 random reachable poses, seeded from `ready` —
  **100% solved, median 4–5 ms, p95 under 50 ms**.
- **28 tests pass** across the packages (`colcon test`), including
  consistency checks that fail the build if duplicated values drift — each one
  verified by deliberately breaking it.

RViz shows the arm and the MotionPlanning panel; drag the marker, Plan, Execute.

---

## 4. Problems found, and what was done about them

Kept in full, because most of these are traps that will recur on the next CAD
re-export or the next arm.

### 4.1 Collision geometry enclosed as little as 7% of each part — SAFETY

**The most serious thing found.** The Fusion exporter emits one axis-aligned box
per link as collision geometry. Those boxes do not contain the parts. Measured
directly, every visual vertex tested against the collision hull's half-spaces:

| link | visual geometry outside its collision box | worst protrusion |
|---|---|---|
| `Link1_Shoulder` | 62.9 % | **107.0 mm** |
| `Link6_Gripper` | 48.7 % | 99.7 mm |
| `Link3_ForeArm` | 32.6 % | 66.3 mm |
| `Link5_Wrist2` | 30.3 % | 43.4 mm |
| `Link2_UpperArm` | 30.1 % | 96.3 mm |
| `Link4_Wrist1` | 20.3 % | 47.8 mm |
| `base_link` | 0.1 % | 0.4 mm (fine) |
| `Finger_Moving` | 0.0 % | 0.0 mm (fine) |

`Link4_Wrist1` and `Link5_Wrist2` had been given *identical* boxes, which cannot
be right for two different parts.

This is not conservative, it is wrong in the dangerous direction: MoveIt reports
a path as clear while the links pass through each other, and the first you learn
of it is the arm folding into itself under 150 kg-cm of torque.

**Fixed** by regenerating collision geometry as convex hulls of the visual
meshes: `arm_description/scripts/make_collision_meshes.py`. It audits first and
only rewrites links that fail, so it is safe to re-run after any CAD change.

```bash
python3 src/arm_description/scripts/make_collision_meshes.py --audit   # report
python3 src/arm_description/scripts/make_collision_meshes.py           # fix
```

**How much this mattered:** recomputing the self-collision matrix with correct
geometry moved four link pairs from "can never touch" to genuinely colliding —
`Link4_Wrist1`↔`Link6_Gripper` collides in **17%** of random configurations and
`Link3_ForeArm`↔`Link5_Wrist2` in **19.5%**. With the exporter's boxes, MoveIt
had been told to never check those pairs at all.

**Still open:** the `Link6_Gripper` hull bridges the gap between the jaws. Right
trade today (conservative, and the gripper is unused); wrong trade for grasping,
because an object between the fingers will read as a collision. Replace with a
convex decomposition (VHACD/CoACD) or hand-authored primitives before grasp work.

### 4.2 The robot was lying on its side

The exporter leaves `base_link` in the Fusion design frame, in which the waist
axis points along **+Y**, not +Z. Loaded as exported, RViz's ground plane cuts
through the robot and gravity would pull it sideways in any physics sim.

**Fixed** with a `world` → `base_link` fixed joint, derived rather than guessed:
the rotation maps `joint1`'s axis onto world +Z and `joint2`'s onto world −Y.
Those two axes are perpendicular to within **0.0°** in the CAD — which confirms
the model is internally consistent and only the *root* frame was misaligned. The
z offset is the lowest point of `base_link`'s collision mesh, so the base sits
flat on z=0.

A further 180° of yaw was added so the arm reaches along **+X**. The elbow only
folds one way — the mirrored fold is a hard self-collision — and without the flip
the entire workspace sits behind the robot at negative X.

A plain 90° rotation would have left the arm tilted by 1.14°, which is 15 mm of
error at full reach.

**And the base was 217 mm off-centre.** The CAD's `base_link` origin is not the
middle of the pedestal, so the robot stood well to one side of the point it
rotates about, and every Cartesian goal had to carry that offset in your head.
The waist axis and the pedestal's footprint centroid agree to **0.0 mm**, so one
correction fixes both: the waist axis is now the world Z axis.

Values in `joint_parameters.yaml` under `base_mount`.

### 4.3 Every joint was unlimited

All six revolute joints were exported as `continuous` — Fusion has no joint-limit
concept. A continuous joint tells MoveIt the servo can rotate forever, so it
plans poses the arm cannot reach and the goal simply never arrives.

**Fixed:** all joints are now `revolute` with limits from
`joint_parameters.yaml`, sourced from the firmware where that exists:

| joint | limit | velocity | provenance |
|---|---|---|---|
| `joint1` | ±120° | 40 °/s | **measured** — `SOFT_LIMIT_*_DEG`, servo #1 |
| `joint2` | ±90° | 40 °/s | **measured** — mechanically limited by the structure |
| `joint3` | ±120° | 40 °/s | *provisional* |
| `joint4` | ±90° | 60 °/s | *provisional* |
| `joint5` | ±90° | 60 °/s | *provisional* |
| `joint6` | ±180° | 60 °/s | *provisional* |
| `Slider_39` | −65…−10 mm | 20 mm/s | CAD |

40 °/s is not a guess and should not be raised casually: it is the speed at which
following error stays inside the firmware's own 15° watchdog, given the measured
213 ms effective lag. Command faster and the Mega will relax the servo mid-move.

### 4.4 Two `ros2_control` blocks, one name, five of seven joints

The exporter emitted a `<ros2_control>` block in the top-level file *and* another
in the assembly macro. Both were named `arm_final_system`; `controller_manager`
rejects duplicate component names. Between them they covered `joint1`–`joint5`
and `joint6`/`Slider_39` — so no single component described the whole arm.

**Fixed:** one block in `urdf/arm.ros2_control.xacro`, all seven joints, with a
`use_mock_hardware` switch so mock and real hardware present an identical
interface to everything above.

### 4.5 The gripper started outside its own joint limits

`Slider_39`'s range is **entirely negative** (−65 mm to −10 mm) straight from the
CAD. The exporter still set its mock `initial_value` to `0.0`, which is out of
bounds. A robot whose start state is invalid cannot plan at all, and the error
points at the planner rather than at the initial value.

**Fixed:** starts at −0.065 (open). A regression test now asserts every mock
initial value lies inside that joint's limits.

### 4.6 pick_ik returned NO_IK_SOLUTION for a pose the arm was sitting in

Configured with `minimal_displacement_weight` and `avoid_joint_limits_weight`
set, and `cost_threshold: 0.001`, **every** IK call failed — including IK for the
arm's own current pose, which is how it was spotted.

Cause: those weights add cost functions whose value is non-zero even at a
geometrically perfect solution. `cost_threshold` is an **absolute** cost bound,
so 0.001 sits below the floor the weights create and rejects every valid
solution. Raising it to 100 restored 4/4 immediately.

Rule: enabling any pick_ik cost weight means `cost_threshold` must be raised
above the floor those weights create.

### 4.7 The IK solver assumption was wrong — KDL beats pick_ik here

The starting assumption was that KDL would struggle, because this arm has no
exact spherical wrist (joint4/joint5 intersect to within 1.1 mm, but
joint5/joint6 miss by **11.7 mm**) and the usual advice is that offset wrists
need an optimiser.

Wrong for this arm. Benchmark — 60 random valid configurations, FK to a tool
pose, IK asked to recover it seeded from `ready`:

| solver | solved | median | p95 | max |
|---|---|---|---|---|
| **KDL, 0.5 s** | **100 %** | **4.0 ms** | 26 ms | 128 ms |
| KDL, 0.2 s | 98 % | 2.1 ms | 42 ms | 82 ms |
| pick_ik global, 0.2 s | 92 % | 10.0 ms | 147 ms | 169 ms |
| pick_ik global, 0.2 s, 4 threads | 80 % | 37.8 ms | 118 ms | 209 ms |
| pick_ik global, 0.05 s | 85 % | 9.2 ms | 33 ms | 45 ms |

Six DOF and a short chain suit a Jacobian method; the memetic optimiser is
solving a much harder problem than this arm poses and paying for it. **KDL at
0.5 s is the configuration.**

Same lesson as the servo bring-up: when a prediction and a measurement disagree,
the measurement wins.

### 4.8 An empty RViz window with no error anywhere

The hand-written `moveit.rviz` contained `{X: 0; Y: 0; Z: 0}` — semicolons where
YAML flow mappings need commas. RViz's response to an unparseable config is to
load **nothing** and report nothing: the window opens, it is empty, and there is
no message to say why.

**Fixed** by rebuilding the config from MoveIt's own Setup Assistant template
(`moveit_setup_app_plugins/templates/config/moveit.rviz`) and adding
`test_configs_parse.py`, which parses every YAML and `.rviz` in the package and
asserts the Fixed Frame is a frame that exists.

### 4.9 `--` inside an XML comment

`<!-- ... --samples 1500 ... -->` is not well-formed XML; `--` may not appear
inside a comment. This broke both a xacro and the SRDF, with an error that points
at a line number and not at the cause. Worth knowing because command-line
examples in comments hit it constantly.

### 4.10 The joint zero was a bent pose — and it was costing most of the workspace

The exporter puts q=0 wherever the CAD assembly happened to be. For this arm that
was a bent pose: the forearm sat **190 mm** off the shoulder axis and the tool
pointed **20.1° off vertical**. Home looked wrong in RViz, which is how it was
noticed.

It was not only cosmetic. The joint limits (±120°, ±90°) are symmetric about
zero, so a bent zero puts the *whole usable range* in the wrong place physically.
Top-down Cartesian goals were failing with NO_IK_SOLUTION at positions the arm
can clearly reach — measured, **0 of 4** test goals planned.

**Fixed** by baking a per-joint zero offset into the six joint origins. Every
joint rotates about its own local +Z, so shifting a zero by `delta` is exactly
`new_origin = cad_origin * Rz(delta)` — no mesh moves, no joint translation
changes, no limit value changes.

After the change, with the final captured zero, **8 of 8** top-down goals plan
and execute, in 3.3–10.2 s. Same planner, same solver, same limits — only the
zero moved.

| goal (tool0, approach down) | old zero | upright zero |
|---|---|---|
| (0.25, 0.00, 0.25) | fail | OK (the `ready` pose itself) |
| (0.30, 0.00, 0.22) | fail | 94 waypoints |
| (0.22, 0.10, 0.25) | fail | 96 |
| (0.25, −0.12, 0.28) | fail | 96 |
| (0.35, 0.00, 0.20) | fail | 98 |
| (0.20, 0.00, 0.35) | fail | 103 |
| (0.28, −0.08, 0.18) | not tried | 69 |
| (0.32, 0.06, 0.24) | not tried | 34 |

**The trade: maximum reach went down.** tool0's horizontal reach dropped from
0.81 m to **0.64 m**, and the lowest reachable point rose from z=−0.17 m to
z=−0.03 m. With home upright, the shoulder's ±90° takes the upper arm to within
**0.6° of horizontal** on each side and no further. Under the old zero the same
±90° was centred on a bent pose, so on one side the shoulder could swing below
horizontal. Planning in front of the robot got much better; reaching far out and
down got worse.

That same number doubles as a **cheap check on the zero at the machine**.
`FeedbackConfig.h` describes the shoulder's ±90° as a *structural* limit. If
mid-travel really is upright, the structure should stop the upper arm at almost
exactly horizontal on both sides. If it stops well above or below horizontal,
the zero is wrong.

It also changed the collision picture completely: the number of link pairs that
ever collide dropped from **19 to 6**, because the arm can no longer fold as far
into itself within its limits.

### 4.11 Half the zero can be solved for; the other half has to be looked at

The vertical stacking was solved numerically — least squares driving every link
origin onto the vertical axis through the shoulder and the tool's approach axis
onto +Z, checked collision-free.

That solve cannot fix the whole pose, and it is worth understanding why.
**`joint4` and `joint6` are roll joints about the arm's own axis.** Turning them
moves no link origin and barely tilts the tool, so they contribute almost nothing
to the objective and the optimiser leaves them wherever it likes. The arm comes
out vertical with the wrist facing an arbitrary direction — which is exactly what
was spotted on screen: "J4 is not proper upright, J5 will also rotate a bit".

No amount of better optimisation fixes that, because the information is not in
the model. Which way the wrist should face at zero is a fact about the built
machine.

So the workflow is split, and `scripts/capture_zero.py` exists for the second
half:

```bash
ros2 launch arm_description display.launch.py        # a slider per joint
# drag until it looks right at the machine
ros2 run arm_description capture_zero.py             # report
ros2 run arm_description capture_zero.py --apply     # write it
colcon build --packages-select arm_description
ros2 run arm_moveit_config analyse_self_collision.py # the limits moved
# then re-derive 'ready'
```

Offsets accumulate in `joint_parameters.yaml` relative to the CAD values, so
repeated captures cannot drift.

**The claim this makes about the hardware.** The limits are symmetric about the
servo's own mid-travel (`CAL_US_AT_ZERO = 1500 µs`). Baking these offsets asserts
that mid-travel on every servo *is* this pose. That is a statement about how the
horns are mounted — and the 18T spline only gets you within ±10° of any angle, so
it has to be made true during the re-zeroing `DS51150_test/START_HERE.md` already
lists as outstanding. If the built arm turns out not to match, change the offsets
here, regenerate, and shift the limits by the same amount, because then zero and
mid-travel really are different poses.

### 4.12 Top-down goals: what still limits them

Even with a good zero, poses where this arm can point its tool straight down are
a narrow basin — only **0.8%** of random configurations point the tool within
10° of straight down (measured with the current zero). Widening joint4/joint5
to the ST3215's full ±180° barely helps (**0.81% → 1.18%**), so this is a
solver-seeding problem, not a limits problem.

That is why `ready` is defined as a pose that is *itself* exactly vertical: it
seeds IK inside the basin. It is not a tidy waypoint, it is load-bearing.

If a top-down goal still fails, loosen the orientation tolerance rather than
fighting it — and prefer `--orientation approach` (pins the approach direction,
leaves spin about it free) over `--orientation full`. Full 6-DOF orientation
goals are rarely solvable on this arm.

`ros2 run arm_moveit_config workspace_map.py` prints a reachability map marking
where a downward approach is possible.

### 4.13 Two failures that do not point at their cause

Worth knowing because both waste time:

- **`--` inside an XML comment** is illegal. `<!-- ... --samples 1500 ... -->`
  broke both a xacro and the SRDF, with an error giving a line number and no
  hint. Command-line examples in comments hit this constantly.
- **`Command()` in a launch file needs `ParameterValue(..., value_type=str)`.**
  Without it, launch tries to parse the xacro output as YAML and reports
  "Unable to parse the value of parameter robot_description as yaml". A URDF is
  not YAML; the message never says the type hint is missing.
- **A script without `chmod +x` silently disappears from `ros2 run`.** With
  `--symlink-install` the installed file is a symlink that keeps the source
  permissions. Three documented commands did not exist for this reason until a
  verification sweep checked every one.
- **Two publishers on `/robot_description`.** MoveIt's
  `publish_robot_description=True` makes `move_group` publish it alongside
  `robot_state_publisher`. Subscribers take whichever arrives first — the
  controller manager logs "ResourceManager has already loaded a urdf". Harmless
  while both copies agree, a timing-dependent bug the day they don't. Now off;
  `robot_state_publisher` is the one source.

### 4.14 First motor test: the pot cables are rotated, and a joint jumped

**What was done.** Every motor moved one at a time, a known amount, while the
arm was watched: which joint moved, and which way against the URDF's prediction
for +q. Purpose-built firmware for both boards (safe at boot, dead-man stop,
tracking/load trips, fenced relative moves, 10 °/s), driven by
`tools/motor_test.py`. The ESP32's previous firmware (Waveshare's ServoDriver
demo) was backed up in full first: `firmware/esp32_stock_backup/`.

**The incident.** The first DS51150 test enabled "the base" using the pot on
0x48 — the documented pairing — and sent D9 the pulse for that pot's angle,
+3°. D9 does drive the base, but the base was really at +26°: its pot is on
0x4A. **The base swung about 23° at full servo speed.** The tracking trip
relaxed everything about 0.7 s later, because the pot being watched never
moved. No damage.

**What made it possible.** Two assumptions, both mine:

- *That the documented pot-to-pin pairing was true.* On a hobby servo the first
  pulse is where it goes, at full speed; if the angle it was computed from
  belongs to another servo, it jumps.
- *That a pose check at the arm would catch a swap.* It could not: a base turned
  26° looks exactly as normal as one at 0°. The check only validates joints
  whose angle you can see.

It was also first diagnosed wrongly — as "D9 drives the elbow" — because the
pot data alone only shows *which pot* moved, not *which joint*. The watcher at
the arm corrected it.

**What changed.**

- The firmware now watches every *other* pot while one joint is enabled and
  relaxes everything if one moves more than 2°, naming the joint that really
  moved. A mismatch now costs 2°, not 23°.
- Pin-to-joint pairing is set at run time (`w <joint> <pin>`), so testing does
  not require rewiring.
- The remaining pairings were tested in an order where no guess could jump more
  than 2.5°: after the base, the two remaining pots read within 2.5° of each
  other.

**The actual wiring.** PWM pins and servo placement are exactly as documented.
The pot cables are rotated one position on J1/J2/J3: base → 0x4A, shoulder →
0x48, elbow → 0x49. Once read that way, every pot-rail fingerprint matches the
servo `SERVOS.md` puts in that slot — which also resolved an apparent "servos
in the wrong slots" reading from earlier in the day.

**Directions.** Base, shoulder, elbow and tool roll turn the way the URDF says;
**wrist roll and wrist pitch are reversed**. That is normal, and it is fixed in
`actuators.yaml`, never by flipping a URDF axis.

**A lesson in method.** The base's direction was first recorded as −1 from the
unplanned swing, then shown to be +1 by a clean, deliberate move. The fast,
large motion was easy to misread; it had only ever been marked provisional.

**Under load, feedforward alone stalls in a ~2° band.** A +5° elbow test moved
it +2.2° and it never returned; at +15° it moved +13.3° and returned within
1.2°. The base (no gravity load) returned within 0.2°. The test firmware has no
integral term; the real three-joint firmware needs stage 5's PI loop, retuned
loaded — as `RESULTS.md` already predicted.

**Afterwards, the base's wiper line opened.** An hour after it tracked a
+10° move to 0.2°, the base's pot read −178.6° — physically impossible (travel
is ±133°). The raw channels on ADS 0x4A showed why: wiper 3.52 V against a pot
rail of 3.33 V. A wiper cannot exceed its own rail; the buffer input is
floating. Rail and pot-low on the same cable were fine, so it is one conductor.
The firmware's angle-plausibility check refused to enable it, but a floating
wiper can also settle at a *believable* voltage, so a direct check was added:
a wiper outside 0–100 % of its rail is named as a wire fault and the joint
cannot be enabled. Open at the time of writing.

**Software now follows the as-built wiring.** `motor_test.cpp` uses the
verified table, and `FeedbackConfig.h` — used by every single-joint stage
sketch — was corrected: it paired D9 (the base's pin) with 0x48 (the
shoulder's pot), so uploading stage 5 to the assembled arm would have
reproduced the jump and then closed a loop on a pot that never moves.

**The sensing board is not wired the way `CIRCUIT.md` draws it.** The base's
buffer is MCP6004 channel **A** (pins 3 / 2 / 1) feeding ADS 0x4A, not channel C
as documented. Chasing the fault above using the documented channel measured a
different joint's signal and pointed at the wrong component; the real suspect is
the 100 Ω between the connector and pin 3. Check the channel before probing.

**Wrist servos.** IDs 1/2/3 were already set, all at mid-travel (2046–2050 of
4095), full-range position mode, 11.3–11.4 V. Each +30° move landed within 0.5°
and returned within 0.5°; peak load 6–18 % of stall.

### 4.15 Three-joint firmware works; the Mega's USB link corrupts ~1% of lines

`DS51150_test/src/arm3.cpp` (env `arm3`) runs stage 5's cascade on all three
DS51150s with the as-built wiring, plus a host protocol for ros2_control.
Verified on the arm 2026-09-18/27:

- **Hold.** All three enabled and held for 5 s: drift −0.21 / 0.00 / −0.01 deg,
  worst error 0.23 deg. Well inside the arm's 0.5–1.1 deg backlash. It also
  reproduced a bench finding: servo #2 (shoulder) has the smallest spread
  (0.09 deg), matching `SERVOS.md`'s "servo #2 does not hunt".
- **The PI term is what the loaded joints needed.** The elbow, which stalled
  2.8 deg short with feedforward alone and never returned, now moves +9.60 for
  +10 commanded and returns within 0.21 deg. Base and shoulder likewise.
- **Coordinated streaming works.** All three following a sine at once, tracking
  error peaking 0.6–1.4 deg at 5 deg/s.

**The serial link corrupts about 1% of lines, but only while transmitting.**
Measured with no motion at all: stream 50 Hz with the status stream on, 0.6%
of commands arrive mangled; with the status stream OFF, **zero** out of 159.
The garbage contains fragments of the Mega's own status lines, so the
ATmega16U2 USB bridge mixes directions under load. Nothing to do with wiring.

Mangled lines usually fail to parse, which is luck rather than safety: a
corruption that flips a digit inside a valid setpoint would be acted on. Both
directions now carry an XOR checksum (`...*3F`), so a corrupted line is dropped
and counted (`rx_bad` in `?`) instead. Unchecksummed lines are still accepted so
a human can type commands. After the change: 617 commands accepted, 9 rejected,
556/557 status frames good, no faults.

**Loop headroom is the next constraint.** With three ADCs and a 50 Hz ASCII
status stream the firmware reports `loop=301Hz` and a worst-case control block
of 3196 of 5000 us — dominated by `Serial.print`, exactly as stage 5 found on
one joint. Fine at 50 Hz; too tight for a 100 Hz stream to ROS. Fix when
writing the hardware interface: a binary status frame (~30 bytes, no float
formatting) instead of ASCII.

### 4.16 Binary status frame: 13x less CPU, and the corruption went with it

The ASCII status line cost too much to stream at 100 Hz (§4.15). Replaced by a
fixed 34-byte binary frame — sync `A5 5A`, timestamp, state, fault, then
measured / setpoint / target as centi-degrees and three pulse widths, with an
XOR checksum. `B <hz>` turns it on; `F <hz>` still gives the ASCII line for
humans.

| | ASCII 50 Hz | binary 100 Hz |
|---|---|---|
| worst control block | 3196 µs of 5000 | **236 µs** |
| loop rate | 301 Hz | ~390 Hz |
| corrupted status frames | ~1 in 557 | **0 in 1236** |
| corrupted commands | 9 in 617 | **0 in 536** |

Fewer bytes on the wire also fixed the *inbound* corruption, which the
checksums were catching but which is better not to have. Verified with the
servos driven, so the Servo library's timer ISR was active: 98.9 frames/s,
zero bad checksums, no faults.

Two things fixed along the way, both worth remembering:

- **Deadlines must advance on a fixed grid.** Setting the next deadline to
  `now + period` makes every jitter permanent: a 100 Hz request measured
  **78.7 Hz**, which is 10 ms plus the average wait for the next control tick.
  Advancing `next += period` (resynchronising only if a whole period was
  missed) gives 100.0 Hz.
- **The excitation refresh costs exactly one frame each time.** It busy-waits
  ~7 ms. At one refresh per 333 ms, 2.5% of frames were late; at one per 1.5 s,
  0.7%. The rail is stable to 0.008%, so seconds between refreshes is plenty.
  The real fix, if the stream ever needs to be hard real-time, is a
  non-blocking refresh state machine in `FeedbackSensor`.

### 4.17 ROS drives the real arm

`arm_hardware` is a ros2_control `SystemInterface` that speaks to both boards
and applies the per-joint mapping from `actuators.yaml`, which the xacro turns
into `<param>` entries. The URDF stays the description of the MECHANISM; the
wiring, the reversed joints and the zeros live in one place outside it.

    q_urdf = direction * (motor_angle - motor_angle_at_zero)

Verified on the arm 2026-09-27: both boards connect, all six joints enable and
hold, three controllers activate, no errors over 40 s at 100 Hz.

Notes worth keeping:

- **Commands are seeded from the measured position on activation.** Without
  that, the first `write()` would send whatever the command interfaces hold —
  zero — and the arm would lunge to its home pose at full speed.
- **A firmware fault becomes a controller error**, not a silently limp arm:
  `read()` returns ERROR, which deactivates the hardware. Same for a stale
  link (no status frame for 250 ms).
- **The ESP32 got production firmware too** (`firmware/esp32_st3215`, env
  `arm`): three servos, streaming targets in counts, binary status, dead-man,
  and a collision trip from the servos' own load reading. It does NOT close a
  loop around the ST3215's internal one — that would fight it, exactly as
  stage 5 warns about the DS51150s. Measured: enable with no jump, moves
  landing within 1–5 counts (0.1–0.4°), 303 frames with zero bad checksums.
- **`sg` strips `LD_LIBRARY_PATH`** because it is setuid, so ROS commands run
  through it must re-source the setup files. This cost a confusing
  "librcl_action.so: cannot open shared object file" that had nothing to do
  with the code.

**Calibrating the zeros, visually.** Every `motor_angle_at_zero` started as an
assumption: that the servo's mid-travel is the URDF zero. With live positions
reaching ROS the check is direct — put RViz beside the arm and look.

First pass, 2026-09-27: five of six matched by eye. Only the tool roll was
visibly wrong — RViz said −34.5° where the real joint sat at −8°. The fix is
arithmetic once the error is known, because the reading and the zero differ by
a constant:

    zero_new = counts_now - direction * q_true * counts_per_rad
             = 1655 - (-8 deg in rad) * 651.899
             = 1746 counts        (mid-travel 2048 was 302 counts out)

`joint6` now reads −8.00°. The accuracy of this method is the accuracy of the
eye judging the real angle — a degree or two, which is the same order as the
arm's own backlash (0.5–1.1°), so there is little point chasing further without
an angle gauge.

The other five are "looks right", which is weaker for the two wrist joints than
it sounds: a roll joint's angle is hard to judge by eye. Worth revisiting if
Cartesian moves land consistently off in one direction.

### 4.18 Jitter while holding: an integrator limit cycle, not servo dither

Under ros2_control the joints visibly jittered while holding. Measured rather
than guessed:

| joint | peak-to-peak | sd | commanded variation |
|---|---|---|---|
| **base** | **0.90°** | 0.224° | **0.000°** |
| shoulder | 0.07° | 0.014° | 0.000° |
| elbow | 0.08° | 0.013° | 0.000° |
| wrist | 0.00–0.09° (one encoder count) | — | 0.000° |

The command was perfectly still, so nothing in ROS was causing it. A spectrum of
the base's motion settled it: energy concentrated at **0.24–0.36 Hz**, with 40x
more below 3 Hz than above 10. That is NOT the servo's own deadband dither,
which `RESULTS.md` measured by FFT as broadband **22–46 Hz**. It is an
**integrator limit cycle**: Ki winds up while the output shaft sits inside its
backlash and cannot respond, until the accumulated term overwhelms the deadband,
the joint jumps past, and the cycle reverses.

The base is worst because it is the one joint with **no gravity load**, so there
is no droop for the integral to work against — exactly the case `RESULTS.md`
flagged when it noted Ki=8 drove error to zero but nearly doubled the jitter.

**Fix:** do not integrate inside the joint's own measured backlash (0.52 / 0.77 /
1.08°, per `SERVOS.md`). That is already the hard accuracy floor — no gain beats
it — so integrating there buys oscillation and nothing else. The accumulated
integral is kept, not reset, so droop compensation on the loaded joints
survives.

**Verified 2026-09-27**, holding all three joints for 20 s (999 frames):

| joint | before | after | that joint's backlash |
|---|---|---|---|
| **base** | **0.90°** p-p | **0.36°** p-p | 0.52° |
| shoulder | 0.07° | 0.31° | 0.77° |
| elbow | 0.08° | 0.24° | 1.08° |

Every joint now sits **inside its own backlash**, which is the hard floor: what
is left is the gearbox, not the loop. Drift over the 20 s was at most 0.29°
(shoulder), which is the deadband declining to chase noise rather than the loop
losing the position - the trade that was intended. The shoulder and elbow
numbers rose from 0.07-0.08° because they are no longer being held by a wound-up
integrator; that is the same trade and still well inside their backlash.

### 4.19 The gripper remount left joint6 standing on the servo's count seam

The gripper was unmounted and remounted, which invalidated joint6's calibrated
zero (1746 counts). Re-reading the joint gave something stranger than a shifted
zero:

    scan at 22:04  ->  id 3  pos 4093
    scan at 22:06  ->  id 3  pos    1

It had not moved. The ST3215 reports a single turn as 0..4095 counts, and the
joint was sitting **exactly on the 0/4095 seam**, so the reading flipped across
the wrap with a hair of movement. That is the worst place for a closed-loop
joint to live:

* the reported angle jumps 360° for a few thousandths of a turn, so any error
  term computed from it is meaningless;
* a goal position **cannot cross the seam at all** - the servo has no
  representation for the far side;
* and had the zero simply been moved on paper to 3794 (1746 + 2048, the 180°
  remount), positive travel would have run out of counts after **+26°**,
  against the ±180° the URDF allows.

**Fix: re-index the servo's own zero**, rather than paper over it in the URDF.
Feetech's SDK does this by writing 128 to the torque register
(`SMS_STS::CalibrationOfs`): the servo computes an internal offset so that where
it stands right now reads 2048, and stores it in EEPROM. Added as
`motor_test.py esp-centre <id>`, in the **test** firmware and not the production
one - it writes EEPROM, and a stray command that silently destroys a calibration
is not something the production firmware should be able to do.

Result: joint6 reads 2048 where it stands, the seam is 180° away, the full
travel is usable, and `motor_counts_at_zero: 2048` is now a clean number.

Two things worth knowing about this:

* **The zero lives in the servo, not in this repo.** If that servo is swapped or
  factory-reset, `actuators.yaml` is wrong until `esp-centre` is run again. Said
  so in the file, at the number.
* **The write "failed" but had worked.** `writeRegs` reports failure when no ack
  arrives inside `transact`'s 4 ms window, and an EEPROM write takes longer than
  that, so the ack was simply late. The position read back as exactly 2048.
  `centre()` now ignores that ack and judges by reading the position back -
  verification by outcome, not by acknowledgement.

### 4.20 Three faults found by bringing the stack up against dead hardware

The boards were reconnected after a LiPo charge but the pack was not plugged in
yet. The Mega reported `-148.76 -305.68 55.69 deg` - impossible angles, which
per `START_HERE.md` is the signature of unpowered pot excitation - and the
ESP32's servos answered `0 0 0 counts`. Both firmwares correctly refused to
enable. Then the stack fell over, and the way it fell over exposed three bugs
that had nothing to do with the battery.

**1. A refused activation aborted the controller manager.** `ros2_control`
*throws* if a component cannot reach its initial state, and the default initial
state is `active`:

    terminate called after throwing an instance of 'std::runtime_error'
      what(): Failed to set the initial state of the component : arm_system to active
    process has died ... exit code -6

leaving the spawners looping on a service that would never appear. A flat
battery should produce an error message, not a crash. The component is now
listed under `hardware_components_initial_state.inactive`, so startup
*configures* it - which opens both ports and verifies both boards answer - and
`demo.launch.py` *activates* it as an explicit step, with the controller
spawners chained behind that. This is also the better arrangement on its own
merits: **activation is what energises the arm**, and that should be a decision,
not a side effect of having run a launch file.

**2. Unsigned lines could move the arm.** Both firmwares accept a line with no
`*XX` checksum, so that a human can type commands into a serial monitor. But
the corruption this link actually suffers **drops bytes** (§4.15), and dropping
the tail of `G 1 2 3*7F` leaves a perfectly valid `G 1 2`. The host tool made
this worse by sending its 10 Hz heartbeat unsigned - visible in the logs as

    !! unknown command: [,1h]

i.e. a mangled heartbeat arriving as a command. Now: the host checksums
**every** line, and the firmwares accept an unsigned line only for things that
cannot move the arm (`?`, `h`, `scan`, telemetry). Stopping is always accepted
whatever shape it arrives in - refusing a stop is the one failure that is never
safe. After reflashing, `rx_bad=0` and the garbled-command lines are gone.

**3. The gripper published a NaN joint position.** The loopback joint's state
follows its command, and command interfaces start as NaN, so before any
controller wrote one the arm published `Slider_39: nan`. That NaN reached the TF
tree and RViz dropped the link:

    TF_NAN_INPUT: Ignoring transform for child_frame_id "Finger_Moving" ...
    visual: position of link Finger_Moving contains NaNs. Skipping render

The URDF already declared the right answer - `<state_interface><param
name="initial_value">-0.065`, the finger fully open - but `initial_value` is
applied automatically only by the mock plugin; a real component has to honour it
itself. It now does, in `on_configure`, and `read()` ignores a non-finite
command instead of publishing it. Note that 0.0 would **not** have been a safe
fallback: the gripper's travel is entirely negative, [-0.065, -0.010], so zero
is outside its limits and the planner rejects the whole state (§4.5 again).

### 4.21 "Still jittering, and it moves like an old man"

Both complaints were real, both were measurable, and chasing them turned up
something underneath that neither was about.

**Jitter.** §4.18's integrator deadband was not enough. Measured from
`/joint_states` while holding (`tools/jitter.py`), the base was back at 0.640°
p-p at 0.35 Hz - slightly LARGER than its 0.52° deadband, which is the signature
of a cycle sustained by the *proportional* term inside the band. Freezing only
the integral still left kp and kd reacting to an error the mechanism cannot
answer. Two fixes, one per board:

* **Mega: a hold band.** When the commanded trajectory is stationary and the
  error is inside the joint's backlash, hold the last command and do nothing at
  all. Self-limiting, because droop on the loaded joints grows the error past
  the backlash and un-freezes the loop by itself.
* **ESP32: stop re-issuing a goal the servo already has.** The firmware rewrote
  all three goal positions every 10 ms whether or not they had changed, and each
  write restarts the ST3215's internal move planner. The wrist was being kicked
  100 times a second into re-planning a move it had already finished.

| joint | §4.18 bench | under ROS, before | after | its backlash |
|---|---|---|---|---|
| joint1 base | 0.36° | 0.640° | **0.350°** | 0.52° |
| joint2 shoulder | 0.31° | 0.290° | **0.130°** | 0.77° |
| joint3 elbow | 0.24° | 0.180° | **0.110°** | 1.08° |
| joint4 | — | 0.264° | 0.264° (sd 0.25 counts) | 1 count |
| joint5 | — | 0.439° | **0.000°** | 1 count |
| joint6 | — | 0.615° | **0.000°** | 1 count |

Two wrist joints are now *perfectly* still, and every other joint is at or
inside its own mechanical floor. What remains is the gearbox.

**Smoothness.** The cause was not a speed setting. The Mega ran its OWN
trajectory generator - an exponential approach with `TRAJ_APPROACH_W = 2.5`,
a **400 ms time constant** - on top of the trajectory MoveIt had already
time-parameterised. Every move therefore lagged the plan by a third of a second
and arrived asymptotically. That design was right when the host sent a STEP
("go to 30°"); it became wrong the moment ros2_control began streaming a
finished trajectory at 100 Hz.

The fix is to feed the host's own velocity forward - measured from how fast IT
is moving the target - so the proportional term only has to close the residual.
A streamed trajectory is then followed with no systematic lag, while a step
target still arrives on the exponential and still cannot overshoot.

**And a warning heeded rather than overridden.** The first version of this also
raised `MAX_ACC_DEG_S2` 80 -> 200 and `TRAJ_APPROACH_W` 2.5 -> 12. Both were
wrong, and `FeedbackConfig.h` said so in a comment written from a measurement:
200 °/s² stops the setpoint from 40 °/s in 0.2 s, *faster* than the servo's own
213 ms, so the setpoint halts while the servo is still moving and it coasts
past. `w = 12` is an 83 ms time constant - the same mistake in a different
variable, for any step target. Final values: **`w = 5.0`** (200 ms, matched to
the plant) and **`MAX_ACC_DEG_S2 = 120`** (a ceiling just above the URDF's 80,
so the firmware stops silently rounding down a valid plan without becoming the
thing that overshoots). The lesson is the general one: a constant with a
measurement written next to it is not a tuning knob.

**joint6's direction was wrong, and "VERIFIED" did not make it right.**
`actuators.yaml` recorded `direction: +1` as VERIFIED on 2026-09-18, from
watching a single relative move. Under ros2_control the model and the machine
turned opposite ways. `tools/axis_check.py` settles this from the URDF instead
of by eye - it prints each joint's axis in world frame at the home pose, where
joint6's is `[-0.020 -0.033 +0.999]`, so URDF +q6 is counter-clockwise seen from
above. Commanding +q6 drove the counts UP and the tool the other way, so rising
counts are clockwise and the mapping is **-1**. Joints 4 and 5 were recorded the
same way on the same day and are both -1; joint6 being +1 was the odd one out,
which is the kind of thing a computed cross-check catches and a careful eye does
not. Its zero was calibrated separately: **1629 counts**, confirmed against the
model at the home pose.

**One goal tolerance does not fit six joints.** Moving to `ready` at 50% aborted
with `GOAL_TOLERANCE_VIOLATED`: joint4 settled **1.36° short** and stayed there
for the whole 2 s window, against a 0.017 rad (1°) tolerance copied across every
joint. An ST3215 driven by goal-position writes has a static error under load -
its internal loop stops when its own deadband is satisfied and nothing on this
side can push further. The wrist joints now get 0.035 rad (2°), which is a
measured hardware limit and not a preference: the way to earn a tighter number
is to tune the servos' own PID registers and re-measure.

**Underneath all of it: the battery.** Fast moves kept dying with
`ESP32 fault on servo 3: a servo stopped answering the bus`. The first response
was to widen the tolerance - `BUS_MISS_TRIP` from 3 polls (30 ms) to 20
(200 ms), justified on its own terms, since an ST3215 holds its goal internally
and losing telemetry is not losing control. It tripped again anyway, so the
dropout was real, and always on servo 3 - the last servo in the daisy chain,
where the voltage drop is worst.

So the supply voltage went into the status frame, which the ST3215s report and
nothing was reading. With the arm merely **holding**, not moving:

    arm enabled and holding; servo supply 11.3 V (lowest 10.4 V)
    servo supply down to 9.7 V ...
    servo supply down to 9.5 V ...

11.3 V at rest, **9.5 V under a static hold** - 3.17 V/cell on a 3S pack, at or
below the safe cutoff. A servo that browns out reboots, and a rebooting servo is
indistinguishable from one that has "stopped answering". Every aborted fast move
was this. The arm was parked upright and relaxed rather than discharged further,
and a 30% move immediately before that completed cleanly, which fits: the
failures were at 50-100%, where current draw peaks.

Worth being explicit that the instinct here was wrong twice over - first
widening a fault threshold, then reaching for tolerances - and what settled it
was instrumenting the one quantity nobody was looking at.

**A mistake of mine, recorded because it cost a fault.** Mid-session I ran a
serial probe against `/dev/ttyUSB0` while ros2_control owned that port. It stole
the ESP32's status bytes and reset the board, producing `no status from the ESP32
for 260 ms — link lost` and a clean, correct emergency deactivation that looked
for all the world like a hardware fault. While the stack is up, joint positions
come from `/joint_states` and nothing else may open those ports.

### 4.22 The rail collapsed to 4.9 V, and two protections that were missing

With a charged pack (12.1 V at hold, against 11.3 V the day before) the fast-move
test finally ran. The 100% move home succeeded in 1.30 s. Then:

    t+85  arm_controller: State tolerances failed for joint 1:
          Position Error: -0.175162, Position Tolerance: 0.175000   (10.036 deg)
          Aborted due to path tolerance violation
    t+153 servo supply down to 5.1 V
    t+153 servo supply down to 4.9 V
    t+154 ESP32 fault on servo 2: a servo stopped answering the bus

and afterwards **nothing downstream of the pack had power at all**: the pots read
`exc BAD wiper BAD` at impossible angles (-5261, -5569, -5670 deg) and the
ST3215s stopped answering too. That last pairing is the useful one - the pots run
on the 5 V buck and the ST3215s on ~12 V straight off the pack, so both being
dead puts the failure **upstream of the buck**, in the pack, its protection, a
fuse or the main switch, rather than in the sensing rail.

**Which came first is not established, and the logs cannot settle it.** The
voltage warning only fires below 10.5 V, so there are no samples between 12.1 V
and 5.1 V:

| | explains joint1's 10 deg lag | explains the collapse |
|---|---|---|
| supply failed first | yes - a servo starved of volts cannot hold | yes, directly |
| base pot glitched first | yes - a phantom error drives the servo hard | yes - a stalled DS51150 draws enough to pull the rail down |

The base wiper's un-reflowed 100 R joint (§4.14, still open) makes the second
plausible; a buck or pack at its limit under a 50% move makes the first. What
distinguishes them: if the supply is dead or thermally tripped it was the supply;
if it recovers and the base glitches again under motion it was the pot.

Two protections were missing, and both are now in - because whichever cause it
was, the response should not have been a puzzling bus dropout a minute later.

* **Undervoltage relax (ESP32, `F_VOLTS`).** Holding torque against a collapsing
  rail is the worst use of it: servos draw more as voltage falls, pulling it down
  further, and the first thing to break is a servo browning out - which arrives
  as "stopped answering the bus" and hides the cause. Below **9.5 V** (3.17 V/cell
  on 3S, already the cutoff) the wrist now lets go and says why.
* **Tracking fault 15 deg -> 8 deg (Mega).** 15 deg was sized for an arm with no
  velocity feedforward, where the expected following error was ~4.2 deg at
  40 deg/s. With the host's velocity now fed forward (§4.21) the measured error is
  under 2 deg, so 15 had stopped being headroom and become a blind spot - and
  the 10 deg stall lived inside it. ros2_control gave up at its own 10 deg path
  tolerance while the firmware kept driving. 8 deg puts the firmware's release
  BELOW the point where the layer above it stops trusting the motion, which is
  the right order, and is still 4x the worst error measured.

The general shape of this is worth keeping: the layer that can actually let go of
the hardware should give up before the layer that can only report.

### 4.23 The wrist was hunting, the base was limit-cycling, and neither was the software above them

"The wrist is jittering very very much when the arm moved." Both halves of that
turned out to be servos fighting their own loops, and the fixes were in the
servos, not in ROS.

**Why the earlier "0.000 deg" was not a lie but was worthless.** §4.21 measured the
wrist perfectly still - with the arm at HOME, where the wrist hangs straight down
and carries nothing. In the `ready` pose, with the gripper's weight off-axis:

| joint | holding at `home` | holding at `ready` |
|---|---|---|
| joint4 | 0.264° | 0.703° |
| joint5 | 0.000° | 0.088° |
| joint6 | **0.000°** | **1.494°** |

A hold test at the pose that unloads the joint measures nothing about the joint.
Measure where it is loaded.

**The cause: the servos' insensitive area was 1 count.** Reading their own
registers (new `Q` command) gave, for all three:

    kp=32, kd=32, ki=0, dead_cw=1, dead_ccw=1

One count is 0.088°. Holding the gripper off-axis, each servo was correcting
every single count, overshooting, and correcting back - a hunt at 7.85 Hz. The
ESP32 sends them *nothing* while holding (§4.21's write-on-change), so this was
entirely internal to the servos.

Widening the insensitive area to **3 counts (0.26°)** fixed it. Same pose, nothing
else changed:

| joint | dead = 1 | dead = 3 |
|---|---|---|
| joint4 | 0.703° | **0.176°** |
| joint5 | 0.088° | **0.000°** |
| joint6 | **1.494°** | **0.176°** |

And during a move, where the complaint originated (`tools/moving_ripple.py`,
velocity ripple about its own moving average):

| joint | dead = 1 | dead = 3 |
|---|---|---|
| joint4 | 10.80 °/s | **4.98 °/s** |
| joint6 | **16.83 °/s** | **3.97 °/s** |

A wrong guess first: the speed cap had been raised 600 -> 900 in §4.21, so it
looked like the obvious culprit. Dropping it to 400 changed the hunt not at all
(1.49° vs 1.23°). It stayed at 400 anyway - the trajectory needs about 1 count per
10 ms tick, which 400 steps/s exceeds forty times over, and a lower cap draws less
peak current from a supply that has none to spare.

**The base was a second, separate limit cycle - and `kd` was zero.** All three
DS51150 joints ran with `kd = 0.0`: pure P+I, nothing opposing velocity. Worse,
the amplitude scales with the supply, because more volts mean more authority
inside the backlash where the output shaft cannot respond at all:

| supply | joint1 p-p holding at home |
|---|---|
| ~11 V, sagging (2026-09-27) | 0.390° |
| 11.8-12.1 V, healthy (2026-09-28) | **1.240°** |

So §4.21's hold band had been flattered by a dying battery. `kd = 1.5` on the base
alone: **1.240° -> 0.290° p-p**, sd 0.247 -> 0.045, and the dominant frequency moved
0.34 Hz -> 0.80 Hz, which is a broken cycle rather than a slower one. Damping was
the right lever precisely because it costs no accuracy, unlike widening the hold
band. Joints 2 and 3 are left at kd = 0 for now: they carry gravity loads, their
integrals have droop to work against, and they measure 0.28° and 0.12°.

**Every joint now holds inside 0.3° peak-to-peak**, loaded or not, and four
consecutive 50% moves ran with no fault, no abort and no tolerance violation.

**Three bugs of mine that cost most of the afternoon**, all of them in the
scaffolding rather than the robot:

1. **Activation read stale frames, per board.** A stack that is killed rather than
   interrupted leaves the ESP32 enabled with no host, so it trips its dead-man and
   sits in a fault. `on_activate` sends "c" to clear it - but then read frames the
   board had queued BEFORE the clear, and reported `ESP32 refused to enable:
   dead-man` about a board it had just fixed. The first fix flushed once at the top
   of `on_activate`, which does not work: activation runs the Mega pass first and
   may spend two seconds in its handshake, during which the ESP32 queues another
   two hundred stale frames. The flush has to be per board, immediately before
   that board's clear.
2. **Three orphaned `move_group` processes.** Repeated launch cycles left them
   running, and a stale one with an outdated planning scene answered the action:
   `Ignoring unexpected goal response. There may be more than one action server`,
   surfacing as `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE`. Hours were spent
   treating that as a robot fault.
3. **Commanding moves before the controller existed.** `arm_enabled and holding` is
   printed by the hardware; the controllers are spawned several seconds LATER,
   chained behind it. Moves issued in between fail with `Failed to send trajectory
   part 1 of 1 to controller arm_controller`, which reads like hardware and is not.

`tools/start_stack.sh` and `tools/stop_stack.sh` now exist so none of these
recur: stop SIGINTs first so `on_deactivate` reaches both boards, and start waits
for `arm_controller` to be *active*, not merely for the arm to be enabled. Both
match processes by absolute executable path, because `pkill -f "ros2 launch
arm_moveit_config"` also matches the shell running the script and kills the
caller - which happened three times.

### 4.24 The control side is finished; the power path is not

2026-09-29, with the gated `kd` flashed (see below) and a charged pack. Every
control-side complaint from the previous two days is now measured as fixed, and
everything that still fails, fails for one reason.

**Holding** (`tools/jitter.py`, arm at home):

| joint | kd = 0 | kd ungated | **kd gated** |
|---|---|---|---|
| joint1 base | 1.240° | 0.290° | **0.180°** |
| joint2 | 0.760° | 0.280° | **0.130°** |
| joint3 | 0.120° | 0.120° | 0.150° |
| joint4 / 5 / 6 | — | 0.176 / 0 / 0 | **0.088 / 0.000 / 0.000** |

**Moving**: 30% and 50% moves complete, every joint reads "smooth", and the
base's following error is **0.90° max, 0.23° mean** - against the 11.1° lag that
was aborting every move the day before. Six Cartesian poses across the workspace
all executed.

**Why `kd` ended up gated.** The term has one job: damp the base's hold-time
limit cycle. Letting it act during motion failed twice, in opposite directions:

| form | during a move | while holding |
|---|---|---|
| `-kd * measVel` | opposes commanded motion: 60 us against travel at 40 deg/s, 11.1° lag, path-tolerance abort | correct |
| `-kd * (measVel - setVel)` | no lag, but amplifies the velocity noise of a real pot signal - the motors visibly oscillated | correct |
| **`stopped ? -kd * measVel : 0`** | contributes nothing; trajectories behave as before it existed | correct |

The lesson is narrow and worth keeping: a term added to fix behaviour at rest
should be switched off in motion unless it has been measured in motion.

**Goal tolerances were sized from measurement, not copied.** A move to `home`
aborted with `GOAL_TOLERANCE_VIOLATED` at 1.12° against 0.97°, with the arm
visibly at home - and joint3's backlash (1.08°) already exceeded its own
tolerance, so its goals could never have been relied on. The hold band stops
correcting inside the backlash by design, and the approach leaves up to another
backlash of residual, so ~2x backlash is the honest bound: joint1 1.5°,
joint2 2.1°, joint3 2.6°, wrist 2.0°.

**And then everything failed at once, eight times out of eight.** Not the
controller, not the tolerances:

    servo supply down to 10.0 V
    servo supply down to 7.7 V
    Deactivating following hardware components ... [ arm_system ]

| condition | rail |
|---|---|
| at rest, recovering instantly | 12.0-12.2 V |
| moving into a gravity-loaded pose | **7.7 V** |

A pack that snaps back to 12.2 V the moment the load is removed but collapses to
7.7 V under it is **series resistance**, not depleted capacity. At a plausible
5-10 A that is 0.4-0.9 Ohm somewhere between the cells and the servos, where a
healthy path is under 0.05. It is also getting worse across the session - 9.8 V
under the same kind of move earlier in the day - which is what a resistive joint
does as it heats.

The undervoltage release was lowered 9.5 V -> 9.0 V in the course of this, and
the reasoning is worth keeping straight because the number was doing two jobs at
once. As **servo** protection it was far too conservative: an ST3215 is rated
6-12.6 V and is nowhere near browning out at 9.5 V. As **pack** protection,
3.0 V/cell under load is the normal cutoff. 9.0 V serves both and still catches
the collapse that prompted it (the rail once fell to 4.9 V). It buys working
headroom; it does not make the sag acceptable.

**Localised, 2026-09-29.** The frame carried only the LOWEST of the three servo
voltages, which cannot distinguish a sagging source from a drop along the wrist's
daisy chain. Sending all three settled it in a single move:

    servo supply sagging: id1 11.9 V  id2 12.0 V  id3 10.2 V
    servo supply sagging: id1 11.2 V  id2 11.3 V  id3  7.6 V   (heavier move)

id1 and id2 track each other within 0.1 V; id3 drops **3.6 V** below them. The
resistance is therefore between id2 and id3 - that cable segment, its crimps, or
id3's own connector - and not in the pack, the leads to the arm, or anything
shared. Two things fall out of it:

* A multimeter reads 12.2 V "at the servos" because every easily probed point is
  **upstream** of the fault.
* Every `servo stopped answering the bus` trip through this whole investigation
  named **id3**. It was browning out, rebooting, and going silent - which is
  exactly what a servo does at the far end of a resistive supply.

id3 is also the joint whose gripper was remounted, which is when the trips began,
so the cable was plausibly strained then.

**2026-09-30: id3 stopped answering altogether.** At rest, no load, from the
moment the board boots:

    S,1,1113,...,12.0 V      id1 healthy
    S,2,2537,...,12.2 V      id2 healthy
    S,3,0,0,0,0,4095,0.0     id3 no reply at all
    ## enabled=0 fault=2 id=3   miss=0/0/125

125 consecutive missed reads at 100 Hz, repeatable across reboots, with one
detail that matters: **id3's "0.0 V" is not a measurement of zero volts.** It is
the absence of a reply - the field is simply never filled - so it says nothing
about power, data or the servo individually. Occasionally id3 answers once during
configure and then goes silent, which is an intermittent contact mostly open
rather than a clean break.

The arm now refuses to energise, correctly: `ESP32 refused to enable: a servo
stopped answering the bus`.

**A wrong turn of mine, recorded because the reasoning was seductive.** When the
per-servo telemetry showed `id3 10.3 V / load 24`, I argued that a resistive
connection could not drop 1.5 V at 2.4% of stall, since V = I*R needs current -
and backed away from the bad-connection diagnosis I had just made. That objection
was wrong. A failing contact is not milliohms like a good crimp; it is ohms, so a
few hundred milliamps is all it takes to lose volts across it. The original call
was right, the arithmetic behind doubting it used the wrong order of magnitude
for R, and the connection then failed completely a day later - which is what a
degrading contact does.

**2026-09-30, after the repair: two different faults, one fixed.** The supply
problem is genuinely gone. Under the same loaded move that used to trip
everything:

    SUPPLY 11.9  12.0  12.1 V  |  LOAD -44  176  0

id3 within 0.2 V of its neighbours and now the highest of the three, where it had
been 3.6 V below them. Five moves ran clean, including a 78° wrist sweep.

Then id3 stopped answering again mid-sweep - and the manner of it rules out the
supply:

    SUPPLY 11.8 12.0 12.0 V | LOAD -124  96   0
    SUPPLY 11.8 12.0 12.0 V | LOAD  -88 128 -28
    ESP32 fault on servo 3: a servo stopped answering the bus

**12.0 V, 0.26 s before silence.** A resistive joint announces itself by sagging
under load, as this one did the day before at 10.3 and then 7.6 V. A conductor
that simply opens does not: the voltage is right up until the moment there is no
reply at all. Both dropouts happened while the arm was moving, and never at rest -
`--watch` sees zero missed reads in 20 s of standing still. That is a broken
strand or a pulled crimp flexing, and it needs the segment replaced rather than
re-seated.

**CONFIRMED by inspection, 2026-09-30: the fault is in the wire.** The telemetry
called it correctly - a resistive supply connection first, then an open under flex
- and both were in the same cable to id3.

Worth separating clearly, because conflating them cost time: **id3 has had two
faults in three days** - a resistive supply connection (fixed, and it was the
cause of the 7.6 V sag and the undervoltage trips) and an intermittent open under
flex (still present, and the cause of every "stopped answering the bus"). The
per-servo voltage telemetry is what distinguishes them, and neither is visible
from a meter reading at rest.

**Nothing further should be tuned on this arm until that cable is replaced.**
The control loops are measurably done, and every remaining failure is this.

### 4.25 Masses, a measured limit, and why the arm cannot be moved by hand

**The links are printed plastic, not steel.** The Fusion export carried its
default material, so every mass and inertia was ~13x too heavy: 24.94 kg for the
arm, 9.03 kg for `base_link` alone, on a machine driven by hobby servos. The CAD
numbers stay exactly as exported - they are the geometric record - and are scaled
by `real_density / cad_density` from a `materials` block in
`joint_parameters.yaml`. Mass and inertia are both linear in density for fixed
geometry, so one ratio corrects all 49 numbers, including the gripper link in
`Component92_step.urdf.xacro` which is a separate file and was missed on the
first pass (the total gave it away: 2.515 kg where the links file alone predicted
1.906).

| link | as "steel" | as printed plastic |
|---|---|---|
| base_link | 9.034 kg | **0.691 kg** |
| Link1_Shoulder | 6.768 | 0.517 |
| Link2_UpperArm | 5.986 | 0.458 |
| **total** | **24.942 kg** | **1.953 kg** |

`real_density` is 600 kg/m^3, an estimate for a functional print at a few
perimeters and 20-25% infill (solid PLA is ~1240, ABS ~1040). It is still an
estimate - **weighing one printed link settles it**, and the file says so at the
number. Nothing in kinematic planning uses these; gravity compensation, torque
estimates and any dynamics simulation do, and so does anyone who reads 25 kg and
believes it.

**These joints cannot be measured by hand, and asking was a mistake.** The first
instruction given for measuring real travel was to move each joint by hand with
torque off. Both boards did report `enabled=0` with no command going out - the arm
was genuinely limp - and it still would not move: a DS51150 is geared for
150 kg-cm and is simply not back-drivable. Pushing one by hand strips gears or
breaks the pot. The hardware was answering the question correctly and the
instruction was wrong.

**So the limit is measured under power, by walking the joint to the onset of
resistance** (`motor_test.py arm3-probe`). The joint is stepped outward 1.5 deg at
a time and the TRACKING ERROR watched: while it is free, measured follows setpoint
within a few tenths; when it meets a stop, the setpoint goes on and the
measurement does not. Stopping at that onset rather than at a stall means the
answer arrives with its margin already in it, and nothing is ever forced - the
firmware's own tracking fault sits at 8 deg, four times the threshold used.

What it found for **joint3**, where the limit had been a guess copied from
another joint:

| direction | guessed | measured stop | |
|---|---|---|---|
| toward + | 120° | **+126.6°** | 6.6° of travel was being thrown away |
| toward - | 120° | **-119.7°** | the guess was nearly exact |

The first probe run, with the firmware still clamped at +/-120, reached both ends
with a tracking error never above **0.41 deg** - which is itself the proof that
the guess and not the mechanism was the limit. The firmware's soft limit was then
opened to the servo's own pulse range (+/-133) to find the truth, and set to the
stops afterwards: the URDF (-115.5 / +122.5, 4 deg of margin) is the authority on
what may be commanded, and the firmware (-119.5 / +126.5) is the backstop for a
command that should never have arrived. The asymmetry is real.

**And widening a limit cost a collision exemption.** Re-running
`analyse_self_collision.py` over the wider range found **Finger_Moving <->
Link1_Shoulder** colliding - a configuration the old limit could not express - so
its "Never" exemption was removed. Without that step MoveIt would have planned the
finger through the shoulder. This is the reason that script must be re-run after
any limit change: a joint that reaches further can reach into the arm.

Workspace with the measured limit: reach radius 0.004-0.644 m, height
-0.018-0.969 m, and still only **2.5%** of sampled poses can point the tool within
20 deg of straight down - the top-down envelope of 4.12 is unchanged by this and
remains the arm's real geometric weakness.

**Also this session:** the gripper controller is off by default
(`gripper_controller:=true` to restore it) now that it only drives a loopback; the
RViz display states its three robots explicitly instead of inheriting defaults
(solid = the real arm, orange = the goal, faint = the plan, no trail); and the
gripper's loopback value was set to the midpoint of its range so the live robot
and the orange goal robot stop disagreeing by 27 mm of finger travel - MoveIt puts
a joint whose range excludes zero at its midpoint, and matching that convention is
cheaper than explaining the difference.

### 4.26 The shoulder servo stopped responding, and how one number proved it

A full sweep went 0 for 11 where the day before it had run clean. The per-joint
following error named the culprit immediately - at 15% speed, with a peak
commanded velocity of 2.6 deg/s:

| joint | max error |
|---|---|
| joint1 | 0.32° |
| **joint2** | **10.00°** |
| joint3 | 1.17° |
| joint4/5/6 | 0.79 / 1.01 / 0.40° |

At 2.6 deg/s a healthy servo has no excuse for 10 deg. But position alone cannot
distinguish "the joint is not being driven" from "the joint is being driven and is
not moving", and those need completely different fixes - so the probe was taught
to print the **commanded pulse width** alongside, which settles it:

    target  -2.59   meas -4.58   behind +1.94   cmd 1495 us
    target  -0.59   meas -4.58   behind +3.94   cmd 1531 us
    target  +1.41   meas -4.58   behind +5.94   cmd 1572 us

The loop drove the command 1495 -> 1572 us, about **14 deg of commanded
deflection**, and the measurement did not shift by 0.01 deg. The base and elbow had
both responded to the same code path minutes earlier, so the Mega's output is
sound: **that servo is not responding to its command** - its power, its signal
lead to D10, or the servo itself.

Worth noting what the sensor did here, because it looks like a sensor fault and is
not one. The pot reported `wiper ok` and a rock-steady angle throughout. That is
correct: the pot is wired separately to the sensing board, so it keeps reading the
true position of a joint that genuinely is not moving. A servo with no power still
has a working pot - which is exactly why the two have to be reasoned about
separately.

**Also fixed, from the same logs:** activation enables one board at a time, and the
Mega's handshake can take up to `handshake_timeout_ms`. An ESP32 still enabled from
a previous session gets nothing during that window and trips its own dead-man -
`ESP32 fault on servo 0: dead-man` - which is the board being right and the
sequence being wrong. Both boards are now relaxed before either is enabled.

### 4.27 Replacing a servo means re-measuring one number, and only one

The shoulder servo was replaced (4.26: it stopped responding to a command that
drove its pulse 14 deg out, with the pot reading correctly throughout). The
replacement works - closed-loop tracking within 0.64 deg to +20 deg and 0.44 deg
to -20 deg, where the old one would not move at all - but it did not work until
one constant was re-measured, and it is worth being precise about which.

The sensing is a tap off the servo's OWN internal potentiometer, so:

    deg = CAL_DEG_PER_UNIT * (sense_unit - unitAtZero)

`CAL_DEG_PER_UNIT` is shared - `FeedbackConfig.h` established that only the
offsets differ between these servos - and `unitAtZero` belongs to the individual
servo. Swap the servo and that one number is wrong, which is why the joint read
**+158.36 deg** afterwards and the firmware refused to enable with "measured angle
is not believable". That refusal is correct and it is also circular: the servo
cannot be driven to be calibrated, and cannot be calibrated without being driven.

`cal_hold` (env `cal`) breaks the circle. It is a standalone sketch that parks the
servo at one compiled-in pulse width from `setup()` and prints the settled
reading, so it needs no valid zero and survives the Mega's reset-on-connect.
Pointed at D10/0x48:

    HOLD,1500,49902.2,0.6
    HOLD,1500,49902.4,0.3
    HOLD,1500,49902.5,0.9

| | old servo | new servo |
|---|---|---|
| pot rail fingerprint | 26576 | **26335** |
| `unitAtZero` | 49690.880 | **49902.400** |

The rail fingerprint is how the swap announced itself in the first place - rails
are per-servo, and `SERVOS.md` records them for exactly this kind of
identification. The offset moved 211.5 units, which at -0.00319 deg/unit is
**-0.675 deg**: the horn went back on within a degree of where the old one sat.

`usPerDeg` was carried over at 7.4615 rather than re-swept, on the file's own
evidence that the scale is shared (the three originals span 7.4615-7.50408, 0.6%).
That is an assumption, not a measurement, and it is the thing to re-check if this
joint ever tracks poorly at speed - a sweep needs an external angle reference,
which is why it was not done now.

**Practical note for reassembly, which is cheaper than calibrating later.** After
`cal_hold` the servo sits exactly at mid-travel. Bolting its link on in the
UPRIGHT pose at that moment makes motor-zero equal URDF-zero, and
`actuators.yaml` needs no offset at all. Attach it anywhere else and the offset
has to be recovered by eye, the way joint6's was (4.19, 4.21). The same applies
to joints 4 and 5, whose zeros are still provisional at a plain 2048.

### 4.30 Making a posed arm the home pose - in the actuators, not the URDF

The arm was reassembled with links bolted on at arbitrary angles, so it no longer
stood in the model's zero pose. Fixed by posing it by eye in RViz and recording
where each MOTOR sits. Result: all six joints read home within 0.94 deg, and
`joint_check.py` gives **6 of 6 at 0.19-0.75 deg** - the best tracking recorded
here.

**Which zero to move, because there are two and only one is right.** The repo has
`capture_zero.py`, which rewrites the URDF's joint origins and accumulates into
`zero_offsets_rad`. That is for when the CAD pose itself is wrong, and it is the
wrong instrument for a mounting offset: it would redefine the model to match
however the links happened to be bolted on, undoing "upright at zero" (4.10),
changing what every joint limit means physically, and forcing the self-collision
matrix to be regenerated. A mounting offset belongs in `actuators.yaml`, whose
entire job is to say where the motor sits when the URDF joint reads zero. The
model stays canonical and nothing downstream is invalidated. `tools/set_zeros.py`
was written for that side.

**For the wrist, re-index the servo instead of recording an offset.** joint6 was
posed 340 counts from its servo's centre. Recording 1708 as its zero would have
worked arithmetically and cost 150 deg of travel: 1708 is only 340 counts from
the 4095 ceiling, which parks the joint beside the 0/4095 seam - exactly the trap
of 4.19, where a goal cannot cross the seam and the reading flips 360 deg next to
it. Instead the servo's own zero was re-indexed onto the posed orientation (new
`C <id>` command, Feetech's write-128-to-register-40), so it reads 2048 there:

    S,1,2047    S,2,2048    S,3,2048

`zero_counts` stays a clean 2048 for all three, travel stays symmetric, and
**actuators.yaml needed no change at all**.

**What was deliberately NOT captured.** joints 1-3 read +0.05, +0.30 and +0.69 deg
at the posed pose - inside their own droop and backlash. Capturing those as the
new zero would have baked in whatever the arm happened to be doing at that
instant, which is the mistake `set_zeros.py` prints a caution about. Their zeros
were already right to within the accuracy of the method, so they were left alone.
The method is eyeball-accurate, 1-2 deg; it is not a jig.

**A bug worth recording, because the tool reported the opposite of the truth.**
Three EEPROM writes all succeeded and all three were reported as
`!! expected ~2048 - the offset did NOT take`. The verification read was coded as
`readRegs(...) == 2`, and `readRegs` in this firmware returns **bool**, not a byte
count - so the check was always false. What caught it was not re-reading the code
but asking the hardware: the positions read 2048, so the writes had worked. When a
tool says a write failed, confirm against the thing itself before believing it -
and prefer judging by outcome over judging by acknowledgement, which is the same
lesson as 4.19's late EEPROM ack.

**Standing caveat:** all three wrist zeros now live in the SERVOS' EEPROM. Swap or
factory-reset a wrist servo and its zero is gone, with `actuators.yaml` still
claiming 2048. The fix is to re-run the re-centre, not to edit the file.

### 4.31 A stale device path pointed the wrist driver at the Arduino

The stack stopped starting after twelve days idle. Two causes, and the first is a
mistake I made deliberately and documented at the time.

**The ESP32's device path was socket-dependent, and the board moved.** 4.21 chose
a by-path name for it on the grounds that the CP2102's serial is the generic
"0001" and therefore not unique - and the comment there said plainly "move it to
a different socket and this must be updated". It was moved (physical port 11
instead of 13, and a kernel update also added parallel `usbv2` names), so the
configured path simply stopped existing.

That much was foreseen. What was not: the stale name had been **reassigned to the
other board**.

    pci-0000:00:14.0-usb-0:11:1.0-port0  ->  /dev/ttyUSB0   (the ESP32, now here)
    pci-0000:00:14.0-usb-0:13:1.0        ->  /dev/ttyACM0   (the MEGA)

So `tools/wrist_status.py`, `tools/wrist_tune.py` and `motor_test.py`'s ESP_PORT -
all three still carrying the old literal - were opening the **Arduino** and asking
it ST3215 questions. They returned nothing and looked like a dead wrist bus. A
path that merely breaks is a good failure; a path that silently resolves to
different hardware is not, and that asymmetry is what made by-path the wrong
trade rather than just a fragile one.

Everything now matches by **device identity**: the Mega by its own serial, the
ESP32 by its by-id name. The original caveat is real but narrower than the failure
it caused - the CP2102 name is unique only while it is the only CP2102 attached -
and if a second one ever appears the answer is a udev rule for a stable
`/dev/arm-wrist`, not a return to socket-dependent paths.

**Second cause, and it is the familiar one.** During one start attempt, id1
stopped answering the bus at zero load, while merely configured and not
energised - the open-contact signature of 4.28. It cleared on retry and the arm
then ran clean, all six joints tracking within 0.80 deg. Note which servo: since
the chain was re-routed (4.29), **id1 is the far end**, and every fault this chain
has produced has been at whichever servo is furthest along it - id3 twice, then
id2 reading low, now id1. That is the same conclusion from a third direction:
daisy-chained power is the fault, not any particular servo or cable.

**Self-inflicted, worth noting:** editing those three tools, a quoted string was
split mid-literal and all three files were left unparseable. The syntax check run
immediately afterwards caught it before any of them were used. Checking a file
parses after a mechanical edit costs a second and the alternative is a confusing
hardware-shaped error.

### 4.32 Making startup survive the fault it cannot fix

"The stack is not starting" had two causes over this stretch, and only one of them
was ever a software bug.

**Cause 1, fixed for good: device naming.** Covered in 4.31 - both boards are now
matched by device identity rather than by socket, so replugging cannot point a
driver at the wrong board. One hole remains and is documented at the constant: the
CP2102's serial is the generic "0001", so its name is unique only while it is the
only CP2102 attached. If a second ever appears, the answer is a udev rule for a
stable `/dev/arm-wrist`, not a return to socket-dependent paths.

**Cause 2, which software cannot fix: the daisy-chained wrist power.** The servo
furthest along the chain intermittently stops answering the bus for a moment, at
zero load. It has moved as the wiring changed - id3 twice, then id2 reading low,
now id1 - which is what identifies the chain rather than any servo or cable
(4.28, 4.29, 4.31). The fix is to feed the three servos' power in parallel and
daisy-chain only the data line. That is open question #0 and it is hardware.

**What was fixed permanently is the CONSEQUENCE.** A momentary dropout during the
enable handshake used to fail the whole activation, which took the controllers
down with it and left the operator re-running the launch - so a transient
presented as a hard failure, and the real fault stayed hidden behind it.
`on_activate` now retries the handshake up to three times per board. Each attempt
flushes that board's queued frames, clears its latched fault and re-enables from
a known state, so a retry is a genuine second try rather than a hopeful repeat.
A persistent fault still fails, with its reason, just three attempts later.

**Verified against a real refusal, not assumed.** The obvious way to exercise the
retry - shrinking `handshake_timeout_ms` - does not work, because that timeout is
shared with `on_configure`: at 1 ms configure failed first and activation was
never reached, and at 12 and 40 ms the boards still reported enabled inside one
frame period, so the first attempt kept succeeding. Both layers pass or fail
together on that parameter. So the ESP32 was temporarily flashed with a bogus
servo id (`IDS = {1, 2, 9}`), which makes the firmware genuinely refuse:

    WARN  ESP32 refused to enable (a servo stopped answering the bus) - attempt 1 of 3, retrying
    WARN  ESP32 refused to enable (a servo stopped answering the bus) - attempt 2 of 3, retrying
    ERROR ESP32 refused to enable after 3 attempts: a servo stopped answering the bus

Then restored, reflashed, and confirmed: normal startup uses **zero** retries and
all six joints track within 0.92 deg. Worth the two flashes - the stale-frame bug
(4.28) and the `readRegs == 2` bug (4.30) were both in error paths that had never
been run, and both reported the opposite of the truth.

### 4.33 "Laggy and out of sync" was quantised motion, on both boards

The complaint was that moves are not smooth and the joints do not move together.
The measurement said it was not lag: following error never exceeded 1.9 deg. What
it was shows up in commanded versus MEASURED peak velocity during a 30% move:

| joint | commanded peak | measured peak |
|---|---|---|
| joint1 | 0.1 deg/s | 7.0 |
| joint2 | 3.8 deg/s | 33.0 |
| joint4 | 8.9 deg/s | 53.3 |
| joint6 | 0.1 deg/s | 35.3 |

Up to 9x the commanded speed, in bursts, then nothing - the motion was quantised
into micro-jumps of 0.26-0.45 deg per 10 ms sample, against a trajectory asking
for about 0.04 deg per cycle.

**The metric was validated before acting on it.** A 0.45 deg step inside one
sample implies an acceleration the servo should not manage, so the obvious
suspect was jitter in `/joint_states` timestamps inflating dq/dt. Computing the
velocity both ways - from stamps, and from the sample index at a nominal 100 Hz -
gave the same answer, with inter-sample dt measured at **10.00 ms mean, 9.6-10.5
ms range, and zero samples closer than 2 ms**. The steps were real.

**Fix 1, the Mega: integrate normally while moving.** The integrator deadband from
4.18 exists to stop a limit cycle while HOLDING - inside the backlash the output
shaft cannot respond, so integrating there winds up until the joint jumps past.
It was applying during motion too, where none of that reasoning holds: with only
kp acting (6 us/deg, so 3 us at half a degree) the command sat below the servo's
own stiction, the joint did not move, the error grew to the edge of the backlash,
the integrator switched on and the joint jumped. Gating it on `stopped` **halved
the mean following error** - joint2 0.39 -> 0.20 deg, joint3 0.74 -> 0.31 deg -
and reduced ripple on both.

This is the third time the same mistake has been found in a different variable:
a term added to fix behaviour at rest must be switched off during motion unless it
has been measured during motion (see also the kd gating in 4.23-4.24).

**Fix 2, the wrist: less aggressive reaction.** The ST3215s' own Kp was 32. At 16:

| joint | kp = 32 | kp = 16 |
|---|---|---|
| joint4 ripple | 6.12 deg/s | **2.92** |
| joint6 ripple | 5.50, "vibrating in place" | **0.78, "smooth"** |

joint6 travels barely 1 deg during that move, so what it was doing was reacting to
the arm shaking it, not traversing roughly - and halving Kp cut that by 86%.

**A hypothesis of mine that the measurements refuted.** The wrist's 0.26 deg
granularity was exactly its 3-count deadband, so the deadband looked like the
cause. Measured at 1, 2 and 3 counts over the same move, ripple was identical
within run-to-run noise (joint4 5.54 / 5.91 / 6.12; joint6 8.94 / 5.57 / 5.50 -
and at 1 count joint6 was WORSE, because a tighter deadband means more reaction to
being shaken). The deadband was not it; Kp was. Restored to 3, which has the best
hold evidence behind it (4.23).

Also fixed: the ESP32 now computes each goal's speed from the motion actually
being asked for rather than writing the cap every cycle (the cap darts across each
10 ms segment and waits out the tick). It made no measurable difference on its own
but removes a real stepping mechanism, and the cap at 800 steps/s is now a ceiling
for catching up rather than the operating speed.

**And tuning stopped because the measurements stopped being trustworthy.** With
identical settings, joint3's ripple ranged 1.90 to 5.04 deg/s across runs, and the
undervoltage release began firing mid-measurement. The pack fell from 11.7-12.0 V
resting this morning to **10.7-11.2 V**, and a sagging supply means less servo
torque, which means more stiction stepping - so the later runs were measuring the
battery, not the controller. Nothing further should be tuned for smoothness until
the pack is charged and the wrist's power is fed in parallel (open question #0).

### 4.34 Straight-line motion, and the limits that blocked it

**IK and path planning already existed** - `move_to --pose X Y Z` has solved IK
with KDL and planned a collision-free path with OMPL since 4.7, and dragging
RViz's marker is the same pipeline. What was missing was control over the SHAPE
of the path. `move_to` takes whatever OMPL invents in joint space, which is right
for "get over there" and wrong for approaching something, where coming straight
down onto a part is the whole point.

`arm_demos cartesian_to` adds that, via MoveIt's `compute_cartesian_path`:

    ros2 run arm_demos cartesian_to --relative 0 0 -0.08        # 8 cm straight down
    ros2 run arm_demos cartesian_to --pose 0.30 0.05 0.25       # line to a point
    ros2 run arm_demos cartesian_to --relative 0 0 -0.10 --plan-only

Verified on the real arm: 50 mm down, 50 mm up, 60 mm in +X and 40 mm in +Y, each
100% solved and executed.

**The fraction is the whole contract.** A straight line is a far stronger demand
than "any path" - it can leave the workspace part-way, cross a singularity, or be
blocked, and unlike joint-space planning it cannot go around anything. The service
reports how much of the line it solved, and a partial path is NOT a partial
success: it stops short of where you asked, which for an approach move means
stopping above the part instead of on it. So it refuses below `--min-fraction`
(0.95 default) and says how far short it would have stopped. A rotating move
tested at 11.1% was refused - which is 4.12's top-down envelope showing up exactly
where it should.

`--jump` defaults to 5.0 rather than 0. MoveIt accepts 0 to disable the IK jump
check, and on real hardware that is how a joint slams across its range mid-line
near a singularity.

**Two bugs found while building it, both in "it looked like it worked".**

* `start_state.is_diff` was unset. An empty RobotState with that flag false is
  not "start from where the arm is" - it is read as a fully specified state with
  every joint at zero. The first call happened to succeed because the mock arm
  really was at zero; every later one solved 0% of the line, because it was
  planning from a pose the arm was not in. `move_to` sets the same flag, which is
  where the fix came from.
* The TF read accepted the first transform that resolved. That returned the tool
  at (0.494, -0.002, 0.624) when it was actually at (0.037, -0.018, 0.969) - a
  pose assembled from a partly-populated tree. It now lets the buffer settle and
  rejects a transform older than a second, because a stale pose is worse than no
  pose: it is precisely the input a Cartesian path must not be given.

Several 0% results along the way were NOT bugs but correct answers from a bad
start pose, which is worth knowing before chasing one.

**And on real hardware it was blocked by something else entirely.** Every plan
failed with `UNKNOWN(-26)` - MoveIt 2's **START_STATE_INVALID**:

| joint | at | limit |
|---|---|---|
| joint4 | **+90.09°** | ±90° |
| joint5 | **+90.18°** | ±90° |

Both a tenth of a degree outside their limits, which invalidates the whole start
state and refuses every plan, Cartesian or not. Those +/-90 values are the
PROVISIONAL guesses of open question #1 - deliberately conservative, never
measured - and the arm simply goes further than them. That question has now gone
from "costs workspace" to "intermittently bricks planning", which makes measuring
the wrist travel the next thing to do rather than a nice-to-have.

### 4.35 The base's pot wiper went open - and it looked like a joint limit

The stack stopped activating with:

    Mega fault on joint 1: pot wiper out of range (wire open) - every servo relaxed

and the base reporting **-121.94 deg**, outside its +/-120 limit. The obvious
reading - "the arm is parked past its limit, like joint4/joint5 in 4.34" - was
wrong, and what gave it away was that the number kept changing without the arm
moving:

    -121.94  ->  -133.73  ->  -179.59  ->  -180.22

Nothing was enabled between those readings; the only events were firmware
flashes, and both firmwares boot with every servo limp. A joint angle cannot do
that. A sensor reading can.

**Measured electrically** (env `diag`, which reads every ADS1115 pin directly),
on the base's chip at 0x4A:

| pin | reads | should be |
|---|---|---|
| A1 pot_low | 0.0001 V | ~0 V, correct |
| A2 pot_high (excitation) | 3.3184 V | top of the divider, correct |
| **A0 buffered wiper** | **3.5251 V** | somewhere BETWEEN A1 and A2 |

The wiper reads **0.2 V above the top of its own divider**, which no working
potentiometer can do: the wiper is a tap between pot_low and pot_high and cannot
leave that range. A floating op-amp input saturates to a rail instead, and
3.525 V out of a 5 V-supplied MCP6004 is a saturated output. The excitation is
healthy, all three ADS1115s answer on I2C, so the fault is the wiper path alone -
the tap from the base servo's internal pot, through the 100 R, to MCP6004 pin 3.

**This was a known open item and it has now failed completely.** 4.14 recorded the
base wiper as intermittent, with the 100 R at MCP6004 pin 3 never reflowed (it
also measured 110.7 R rather than 100). Intermittent became open.

Two things worth keeping from how this went wrong:

* **The firmware was right and the interpretation was wrong.** The wiper check
  exists precisely for this and it fired correctly; the mistake was reading its
  consequence (an implausible angle) as a position rather than as a sensor
  failure. The tell is cheap and general: an angle that changes while the arm
  demonstrably cannot move is never an angle.
* **Refusing to enable saved the joint.** The false -121.94 deg produced a
  commanded pulse of ~585 us, close to the servo's 500 us floor, which would have
  driven the base hard into its end stop on the strength of a dead sensor. The
  health check refused, which is the entire reason it is there.

Repair: reflow or replace that 100 R and check continuity from the base servo's
wiper tap to MCP6004 pin 3. The arm's actual base position is UNKNOWN until it
reads again, and is probably fine - nothing drove it anywhere.

### 4.36 The wrist chain's bad link, located to one connector

An execution aborted mid-move. The chain of log lines reads as a controller
failure - `Controller 'arm_controller' failed with error INVALID_GOAL: Current
goal cancelled during deactivate transition`, then `Failed to validate
trajectory: couldn't receive full current joint state within 1s` - but all of
that is consequence. The cause is one line above the deactivation:

    servo supply sagging: id1 7.6 V / load 36   id2 7.3 V / load -91   id3 12.6 V / load -54
    ESP32 fault on servo 1: supply below 9.0 V

**id3 held 12.6 V while id1 and id2 both collapsed to ~7.3 V.** Since the board's
lead now goes to id3 first (4.29), id3 is nearest the supply - so the source was
delivering full voltage and the entire 5 V was lost below it. Both downstream
servos dropping by the same amount means the resistance sits in the one link they
share: **id3's outgoing segment and its connectors**. At the loads shown (under
9% of stall) a 5 V drop implies several ohms, which is a bad contact rather than
undersized wire.

This finally pins down what has moved around the chain for a fortnight - id3
twice, then id2 reading low, then id1 - to a single segment, and it also
exonerates the pack, which has been suspected repeatedly.

**The diagnostic was lying, and that is fixed.** The fault message asserted "the
pack sags under load; check leads and connectors", which is what sent an earlier
session to measure the battery. It now prints the three voltages and reads the
gradient: if all three fall together the loss is upstream (pack, main lead,
shared connector); if they do not, the supply is sound and the resistance is in
the link shared by whichever servos dropped. Same data, one inference, stated
where it is needed rather than left to be recalled.

### 4.37 The wrist is limited by its cable, and the probe found that out by snapping it

Probing joint4's travel (`wrist_tune.py --probe 1 170`) found no mechanical
resistance at all - free to **+150.12°** with the servo's own load never above
43/1000, against a guessed limit of 90°. Then the servo stopped answering the
bus and stayed dead.

It was not a bus fault. **The cable had run out of length and broken.** Rolling
joint4 winds the bundle feeding everything distal to it, and the probe drove
straight past what the harness could take.

**This is the root cause of the whole fortnight of wrist dropouts.** They moved
around - id3 twice, then id2 reading low, then id1 - and survived three separate
repairs, which never made sense as three independent bad crimps. One limit being
exceeded explains all of it: every time the wrist rolled far, the harness
strained and whichever connection was tightest let go. The per-servo voltage
gradients that localised "the bad link" were real measurements of a symptom whose
cause was the joint limits allowing more rotation than the wiring had slack for.

**The probe cannot find this limit safely, by construction.** It stops at the
onset of resistance, and a cable going taut produces almost none - it simply
breaks. So the tool now **clamps to the limits in joint_parameters.yaml** and
refuses to go past them without `--beyond-limits`, which prints why. A limit that
nothing mechanical enforces is still a limit when something else does.

And the limits themselves now say so. They were marked `[PROVISIONAL]`, which is
an open invitation to "improve" them by measuring - which is precisely the mistake
made here. joint4 is `[HARNESS]` with the reasoning at the number, including that
free travel is >= 150° and unusable. joint5 is harness-sensitive but less so,
since pitching bends the bundle rather than winding it. joint6 genuinely has
+/-180° today, because it rolls the tool flange rather than the servo bus - and a
note records that this stops being true the moment the gripper is wired, when the
N20's leads will be wound by joint6 exactly as the bus is by joint4.

The lesson generalises past this arm: "measure the real travel" was good advice
for joint3, where the mechanism is the limit, and actively destructive here. Before
probing a joint, the question to answer is WHAT limits it - and if the answer is
the harness, the measurement is of cable slack, with the joint stationary.

### 4.38 Rebuilding the wrist harness: longer is not the fix, shape is

Two faults tracked separately for a fortnight share one cause, and one harness
job fixes both:

| Fault | Cause |
|---|---|
| Cable snapped at joint4; dropouts wandering id3 -> id2 -> id1 (4.37) | joint4 winds the bundle feeding everything above it, with no slack to give |
| Pack cuts out under full-arm load; the far end of the chain dips to 9.3 V (4.29) | wrist power **daisy-chained** through each servo's connectors |

The reasoning below matters more than the parts list, because three previous
repairs replaced parts and changed nothing.

**1. Extra length must be a HELICAL SERVICE LOOP around the roll axis, not slack
in a straight run.** This is the part that is easy to get wrong. A straight run
with slack flops, snags, and still concentrates the whole twist at one point -
which is why all three previous failures were AT CONNECTIONS rather than
mid-length. A loop coiled on the axis winds and unwinds like a spring,
distributing the twist over its full length. Take-up needed is about
`theta * r`: at +/-180 deg on a 20 mm coil radius that is ~63 mm, so fit ~100 mm
and coil it as wide as the wrist cavity allows - a bigger radius is less strain
per turn.

**2. Star the power from a DISTRIBUTION BLOCK; daisy-chain only the data.** V+
and GND get their own pair from the block to EACH servo, so no servo's current
flows through another's connectors. The ST3215 signal line is low-current and
stays chained. This removes the series resistance behind both the 9.3 V dip and
the pack's protection tripping: id1's stall current stops dropping volts across
two sets of id3's crimps. This was already open question #0j; the rebuild is the
occasion to do it.

Decided 2026-11-02, and NOT "split it at the board". Waveshare's own wiki says
the board's 5.5x2.1 mm DC port "will directly power the servo" - servo current
passes through the barrel jack, the board traces and one XH2.54 connector. Three
ST3215s stall at **2.7 A each** (Waveshare), so the peak is ~8 A through a jack
and a connector each rated nearer 3 A. Waveshare publishes **no board current
rating at all**, which is itself the answer: the board is a signal source that
happens to pass power, and it is the bottleneck. A distributor note says only to
"consider whether the power supply solution is enough when using too many
servos".

Nor is it "three wires straight from the battery terminals". Battery posts are
not a distribution point: that gives no single place to fuse, and no defined
single ground node.

So the topology is:

    pack --[main fuse]-- distribution block
                             |-- pair --> board DC jack   (ESP32 + bus driver only, ~0.2 A)
                             |-- pair --> id1  V+/GND
                             |-- pair --> id2  V+/GND
                             |-- pair --> id3  V+/GND
    board servo connector: SIG (+GND) --> id1 --> id2 --> id3   (signal only, NO V+)

**The ground rule is the part that can bite.** Each ST3215 connector has only
three pins (V+, GND, SIG), and the servo's two connectors are internally
parallel, so a servo's power return and its signal reference are THE SAME PIN.
The UART signal from the ESP32 is referenced to the BOARD's ground. If servo
current runs down one ground path while the board references another, the drop
between them appears as a shifted reference on the bus - corrupted data on a bus
that has been flaky for a fortnight, i.e. a new fault that looks exactly like the
old one. Hence a single block: every ground meets at one node, so there is no
offset to develop. Leave the board's V+ out of the servo harness so the board
carries no servo current, and keep its DC lead short and the same gauge as the
rest, because that lead is now the signal's reference path.

Verify it the same way the fault was found: `ARM_LOG_SUPPLY=1` and
`grep "servo supply" /tmp/arm_stack.log` through a loaded sweep. All three servos
should now stay within a few tenths of each other. A gradient that persists means
the remaining resistance is upstream of the block, not in the wrist.

**3. Silicone-insulated stranded, and gauge the power up.** PVC and solid core
work-harden under repeated twist and fail later, invisibly - the worst failure
mode for something buried in a wrist. Silicone high-flex, **18-20 AWG power**,
26-28 AWG signal.

**4. Strain-relieve BOTH ends of the rotating span.** Anchor the bundle just
before it enters the roll and just after it leaves, so the twist lands on the
free coil rather than on a crimp.

**Then measure, and do not probe.** `wrist_tune.py --limit-here <id>` was added
for this: torque off, the joint posed BY HAND to where the loop just goes taut,
and the tool records that position, converts it through `actuators.yaml` and
takes 5 deg of margin before rounding toward zero. It is the deliberate inverse
of `--probe` - the human finds the limit, because a person can feel a bundle go
taut and a load register cannot (4.37).

It refuses to record anything from a servo that is not answering, and that guard
earns its place: a servo missing from a status frame reports **position 0**, and
0 counts converts to a plausible-looking **0 deg**. Without the guard, a dead bus
would have proposed a limit of 0 deg - a number that looks like a measurement and
would have left the joint no travel at all. It also refuses if the joint moved
more than 1 deg while reading, because the whole value of this method over the
probe is that a human stopped it in the right place.

Order of operations, and it only works in this order: **fit the loop, measure the
slack, then raise the limit.** Raising the limit first is what snapped the cable.

### 4.39 The rewire worked - and the first clean reading found a latent bug

2026-11-02, harness rebuilt with the star power feed of 4.38. `wrist_status.py`,
unloaded:

| | before (4.29, loaded) | after |
|---|---|---|
| supply, id1/id2/id3 | 11.2 / 11.3 / **7.6** V | **12.4 / 12.5 / 12.6** V |
| spread across the chain | 3.6 V | **0.2 V** |
| missed reads | climbing, wandering between ids | **0 / 0 / 0** |
| fault | `2` (servo stopped answering) | **`0`** |

A 0.2 V spread is what a star feed is supposed to look like. The gradient that
three separate repairs could not shift is gone, which also retires the theory
that any one cable or crimp was ever the fault: it was the topology.

**Then the first trustworthy reading in a fortnight found something else.**
Converting the live counts through `actuators.yaml` put joint6 at **-159.7°**,
only 230 counts from the 0/4095 seam. Checking why led to the real problem -
not today's pose, but the LIMITS:

    joint6 lower -180 deg  ->  count 4096.00   OUTSIDE 0..4095
    joint6 upper +180 deg  ->  count    -0.00  OUTSIDE 0..4095

Zero is count 2048 with 651.899 counts/rad, so +/-180° is 2048.3 counts from
centre and the servo only has 2047 either side. **Both configured limits were
unreachable, and the firmware clamps them onto the seam itself.** A goal at the
limit therefore parks this joint exactly on the wrap; a hair past it and the
position jumps ~360°, the loop sees an enormous error, and the joint is lost -
which is precisely how joint6 was lost in 4.19, recovered only by rewriting the
servo's EEPROM.

It had been latent since the +/-180° was written, because nothing had ever
commanded joint6 near its limit. Now at +/-175°: 57 counts of margin at each
end, against a ~23-count goal tolerance - the same better-than-2x reasoning as
joint3's 4° off its measured stops.

**The lesson is the method, not the number.** A round number in radians is not
necessarily a reachable number in counts. Every limit on a counts-based actuator
should be converted into counts and checked against the device's own range,
because the conversion is where zero offset, direction and counts/rad all come
together - and nothing in ros2_control or MoveIt will complain. They will happily
plan to a limit the hardware cannot represent.

Narrowing a limit cannot reach a collision that the wider limit did not, so the
collision matrix stays valid without regenerating it - the re-run rule in
`arm.srdf` is about WIDENING.

### 4.40 The elbow's pot is noisy, and that is what keeps stopping the arm

2026-11-11. The arm aborted twice in one session and the two failures looked
unrelated:

    1794381308  tolerances: State tolerances failed for joint 2 (= joint3, 0-based)
                Position Error: -0.177490  Tolerance: 0.175000      -> PATH_TOLERANCE_VIOLATED
    1794381597  Mega fault on joint 3: tracking error (jam, collision, or lost
                power) - every servo relaxed                        -> hardware deactivated

Both are joint3. Both are about **10°** (10.17°, then an apparent 10.28° of
movement). That is one fault, not two.

**The measurement that settled it.** `motor_test.py arm3-monitor 90` watches all
three pots and never enables a servo, so the arm is provably stationary:

| joint | spread over 90 s, torque OFF, nothing moving |
|---|---|
| base | 0.03° |
| shoulder | 0.03° |
| **elbow** | **4.11°** (-8.39 .. -4.28) |

The elbow's position signal wanders 4° with the arm standing still - **137x its
neighbours**. The glitch counter stayed at 0 only because its threshold is a 5°
jump between consecutive frames, and this noise sits just under it.

**Three checks ruled out the alternatives before concluding it is the sensor:**

* *Not torque.* Static gravity torque about joint3 in the fault pose is
  **0.32 N.m** - it carries 0.288 kg - which is 7% of even the pessimistic
  4.9 N.m rating (open question #2). The elbow is nowhere near its limit.
* *Not supply.* The 2 Hz trace through the fault shows 12.2 / 12.4 / 12.5 V
  rock-steady with all loads at 0. (Caveat: that is the WRIST's telemetry. The
  Mega's own servo supply is uninstrumented - see the new open question.)
* *Not software asymmetry.* All three pots are read through the same
  `j.sense.begin(j.addr)` with the same gain and data rate, and the same
  `kp 6.0 ki 3.0`. Only the I2C address differs (base 0x4A, shoulder 0x48,
  elbow 0x49), and `arm3-status` reports `ads ok exc ok wiper ok` for all three -
  that check tests for rails and opens, not for noise.

So the fault is the elbow's ANALOG CHANNEL: its pot wiper, the wiring to it, or
its MCP6004 buffer channel. This is the same circuit that failed on the BASE in
4.35, where the wiper went open and needed a 100 ohm reflowed at MCP6004 pin 3 -
a different channel of the same part. A partly-broken wiper connection leaves the
buffer input at high impedance, and a high-impedance input wanders exactly like
this.

**Why it presents as "intermittent" and keeps coming back.** The noise is
stochastic, so it crosses a threshold only sometimes - and the firmware's PI loop
*acts* on it, driving the servo to chase phantom error, so the joint genuinely
walks and `|set - meas|` grows until the 8°/800 ms watchdog trips. The arm then
relaxes, which looks like a jam, a collision or lost power, because those are the
three things the firmware can legitimately blame.

**What this corrects.** Earlier the same session the path tolerance was re-derived
from 0.175 to 0.350 rad, because 0.175 rad (10.03°) left only ~2° once the
firmware's designed 8° approach lag was accounted for. That derivation stands on
its own and is worth keeping - but it was NOT the root cause of the abort, and
saying so would have shipped a tolerance change as a fix for a broken sensor. The
10.17° was 8° of designed lag plus the elbow's noise.

**The lesson.** Two aborts with different wording from different subsystems
(`tolerances` from ros2_control, `Mega fault` from the firmware) described one
sensor. What connected them was that both numbers were ~10°, and what proved it
was measuring the joint with NOTHING ENABLED - the cheapest test available, and
the one that cannot be confused by a control loop reacting to its own bad input.

### 4.41 id1's leg measured: ~11 ohm, and a latched fault that killed the stack before any move

Two more findings on 2026-11-11, one hardware and one mine.

**id1's series resistance, from telemetry alone.** The sag warning fired three
times across two sessions with the SAME shape, and the third made it quantifiable
because the load was zero:

    id1 10.4 V / load 0    id2 12.4 V / load 0    id3 12.5 V / load 0

A 2.05 V deficit against its healthy neighbours while drawing nothing but idle
current. The ST3215 idles at 180 mA (Waveshare), so

    R = 2.05 V / 0.18 A = ~11 ohm        in series with id1
    a sound crimp is < 0.01 ohm          -> this is ~1000x that

That single number explains every id1 symptom at once:

| id1 draws | drop across 11 ohm | servo sees |
|---|---|---|
| 180 mA idle | 2.0 V | 10.5 V - runs, reports low |
| ~1 A moving | 11.4 V | nothing - browns out |
| 2.7 A stall | - | nothing |

So id1 survives at idle and **dies the instant it is commanded to move**. It then
stops answering, the ESP32 trips fault 2, wrist torque goes off and the hardware
component deactivates. That is the whole "it fails when I move it" chain, and it
is a connection hanging by a few strands - not a servo, not the bus, not the pack.
A meter across that leg under load will confirm it; re-crimping both ends fixes it.

**Why the voltage never looked bad enough to chase earlier.** At idle the deficit
is only 0.1-0.3 V, which reads as "slightly longer wire". The resistance is only
visible when multiplied by current, and the ratchet warning reports just new
minima - so it showed the extremes and hid how ordinary the idle reading was. The
load column is what made it conclusive: a drop at LOAD 0 cannot be the servo
drawing it.

**The software bug: a latched fault killed the stack before activation.** The
stack died 8 ms after startup with

    Successful 'configure' of hardware 'arm_system'
    Successfully set arm_system to state inactive
    ESP32 fault on servo 1: a servo stopped answering the bus - wrist torque off
    Deactivating following hardware components ... [ arm_system ]

before any controller was loaded and before any goal. The same configure read had
just reported **all three servos answering** (1978 2449 2458 counts), so id1 was
alive; the fault was LATCHED from the previous session. Both boards hold gFault
until a 'c' clears it, and only `on_activate` sends 'c' - but ros2_control reads
INACTIVE hardware too, so the first read saw the stale fault and deactivated the
component before activation could ever clear it. The launch's next attempt then
activated the arm perfectly, which is the proof.

`read()` now converts a board fault into ERROR **only while the component is
ACTIVE**, and otherwise logs that it was latched from before this stack started.
While inactive nothing is energised and nothing can move, so there is nothing to
stop; if the fault is still real, activation clears it, asks again, and refuses to
enable quoting the board's own words.

**Board diagnostics are no longer discarded.** `scan_frames` skipped every
non-frame byte silently, which threw away the only place the firmware says WHY:
the status frame carries fault code 4 ("refused to enable") while the board prints
`!! refusing: id 1 not answering`. Both boards' plain-text lines beginning `!!`
are now logged as `ESP32 says: ...` / `Mega says: ...`, held across reads so a
split line survives. It paid for itself on the first run:

    ESP32 says: !! FAULT id 1 servo stopped answering

which named the servo where the previous message only said the wrist refused.
Complete lines only, `!!` only, so the continuous `##` status chatter cannot bury
them.

### 4.42 The shoulder cannot track, and the Mega's rail was the one thing not measured

Same session, one move, three events in 14 seconds:

    t+949  joint 1 (0-based = joint2, SHOULDER)  error 0.445061 rad = 25.5 deg -> abort
    t+962  joint 2 (0-based = joint3, ELBOW)     error 0.350353 rad = 20.1 deg -> abort
    t+963  Mega says: !! FAULT joint 3 tracking error - jam, collision, or lost power
           -> every servo relaxed, hardware deactivated

**The shoulder is a new fault, distinct from the elbow's pot.** 25.5 deg of host-side
error, of which the firmware's approach filter can explain at most 8 deg, leaves
**17.5 deg of real tracking failure** - 2.2x the firmware's own FAULT_ERR_DEG. And
the shoulder's pot is clean: 0.03 deg over 90 s in the 4.40 measurement, same as the
base. So this is not a sensor.

Two loaded DS51150s losing tracking in the same move is the signature of torque or
volts, not of a sensor. Which leaves the gap: **the three DS51150s report no
voltage at all.** The wrist servos report their own, which is how 4.29, 4.38 and
id1's ~11 ohm (4.41) were all found - and "lost power" is literally one of the
three causes the Mega names when it trips, the one branch that could not be
checked.

**What the loosened tolerance bought.** Raising the Mega path tolerance to 0.350
rad (4.39's derivation) was not a patch over this - it is what made it legible. At
the old 0.175 rad everything tripped and the designed 8 deg lag was
indistinguishable from a real failure. Now 25.5 deg on the shoulder is
unambiguously real, while the elbow's 20.08 deg cleared the bar by 0.02 deg -
right at the boundary, consistent with its noisy pot rather than a second
independent failure. A tolerance sized from the mechanism turns an abort message
into evidence.

**The instrument added.** The 2 Hz trace now carries a MEGA line beside SUPPLY:
each joint's measured, setpoint, the error **against the firmware's own setpoint**
(the quantity its watchdog trips on, excluding the approach lag), and
`command_us` - the pulse width being driven, 1500 us neutral. That last column is
what separates the two candidate causes, which the abort message cannot:

**CORRECTION, same day.** The first version of this claimed `command_us` would
separate "the loop is demanding and the servo is not delivering" from "the loop
is not even asking". It does not, and the live trace disproved it within seconds:
`us = 1500 + deg * 7.4834` reproduces every sample to under 2 us (-63.82 deg ->
1022.4 predicted, 1022 actual). **`command_us` is the position command
re-expressed and carries no effort information at all.** These servos take a
position and close their own loop internally, and the Mega has no current
sensing, so there is no effort signal on that board to log.

What the trace does give is **ERR = set - meas**, the firmware's own tracking
error against its own approach setpoint - the exact quantity FAULT_ERR_DEG trips
on, and not the host-trajectory error that legitimately contains up to 8 deg of
designed lag (4.39). A large ERR proves the servo is not reaching where it is
told, which on a position-commanded servo means torque or volts. Distinguishing
*which* still needs open question 0m: a divider on the Mega's 12 V rail. The
instrument is useful, just less than was claimed for it.

**Two cheap open questions just became blockers**, because the torque arithmetic
in 4.40 rests on both: #2, the stall rating on the servo's own label (50 vs
150 kg.cm is 4.9 vs 14.7 N.m), and #3b, weighing one printed link (`real_density`
600 kg/m^3 is an estimate, and the 0.32 N.m and 1.55 N.m figures scale directly
with it). Until they are settled, "the shoulder lacks torque" cannot be
distinguished from "the shoulder lacks volts" by calculation - only by that trace.

### 4.43 id2's leg collapses to 5.3 V - and in doing so proves the star feed works

2026-11-11, later the same session. The arm enabled, held, and then:

    id1 12.0 V / load 0   id2 10.4 V / load 0   id3 12.5 V
    id1 12.2 V / load 0   id2 10.1 V / load 0   id3 12.5 V
    id1 12.2 V / load 0   id2  5.3 V / load 0   id3 12.5 V
    ESP32 says: !! FAULT id 2 supply below 9.0 V - pack, wiring or a stall
    ESP32 fault on servo 2: supply below 9.0 V - wrist torque off
    -> hardware deactivated

**id2 fell to 5.3 V while drawing nothing but idle current.** At the ST3215's
180 mA idle that is

    R = (12.35 - 5.3) / 0.18 = ~39 ohm        in id2's leg
    id1, measured the same way earlier       = ~12 ohm
    a sound crimp                            = < 0.01 ohm

Both are effectively open connections passing just enough to keep the servo's
logic alive. id1 was the worst leg earlier in the session and id2 is the worst
now, and both have got worse across the session - which favours **strain working
them loose** over a static bad crimp.

**The topology question is settled, and the answer is the good one.** id2 dropped
7.05 V while id1 held 12.2 V and id3 held 12.5 V. A daisy chain CANNOT do that -
series resistance drags the downstream servos down with it. So the power really is
star-fed as 4.38 specified, and **the star did exactly what a star is for: it
confined the fault to one leg instead of taking the whole wrist down.** The
monotonic 12.2 / 12.3 / 12.5 ladder noted earlier, which looked like series
resistance, was just three legs of differing quality at equal idle current - at
idle the two topologies are indistinguishable, and only a fault big enough to
separate them could tell. This one did.

**And the fault is not shared infrastructure.** id3 was never the low servo in a
single reading all day - same pack, same board, same bus, same harness run, same
session. So this is not the pack (#0), not the board, not the ST3215 bus, and not
a flaw in the rewire. Two of the three newly made legs are bad and the third is
fine, which points at those two terminations or at what is working them loose.

**How to localise it without energising anything:** `wrist_tune.py --watch 30`
reports per-servo missed reads live with torque off, while each segment is flexed
by hand. Whichever flex drops a servo names the segment to remake. That is the
difference between remaking the right leg and remaking all three - and a meter
across id2's leg under load shows the 7 V directly.

**Note for the repair:** both ends re-terminated, and the extra length fitted as a
HELICAL LOOP around the roll axis, not slack in a straight run (4.38). If strain
is what is degrading these legs, a perfect crimp on a wire that gets wound every
move will fail again. This is the fourth repair in this area.

### 4.44 joint5's limits were 39.5 deg past its stop - and that is what browned out the wrist

2026-11-11. `wrist_tune.py --range 30`, torque off, joint5 worked by hand:

    id 2: 1243..2623 counts = 121.3 deg of travel

Through actuators.yaml (direction -1, zero 2048) that is **-50.5 deg .. +70.8 deg**
of joint5, against a configured **+/-90 deg**:

| side | configured | real stop | over-travel the config allowed |
|---|---|---|---|
| lower | -90.0 deg | -50.5 deg | **39.5 deg** |
| upper | +90.0 deg | +70.8 deg | **19.2 deg** |

**The complete chain, every number measured:**

    limit beyond the stop -> MoveIt plans a goal there -> the servo drives into
    the stop -> stall, ~2.7 A -> 3.78 V across id2's measured 1.4 ohm leg ->
    servo sees 8.7 V -> under the 9.0 V undervoltage release -> wrist torque off
    -> hardware component deactivated.

Limits are now -45 deg / +65 deg: 5 deg of margin off each measured end, rounded
toward zero, the same rule `--limit-here` applies and the same reasoning as
joint3's 4 deg off its stops - a goal AT the limit must not settle into the stop,
because a servo held against a stop stalls.

**Two wrong turns on the way, both worth recording.**

*The resistance.* id2 read 5.3 V at the collapse while reporting `load 0`, and
~39 ohm was derived from that on the assumption of 180 mA idle current. A meter
then measured 0.7 ohm per wire - 1.4 ohm for the loop - so the assumption was
wrong, not the meter: at 1.4 ohm a 7 V drop means **5 A**, not idle. `load 0` was
the symptom of a servo browning out (a browned-out ST3215 reports zeros), and
earlier frames did show real loads (`id2 load 191`), so the register works and
simply cannot be trusted from a servo losing power. Lesson: a telemetry field
from a browning-out device is not evidence about the brownout.

*The "fine" circuit.* The wiring checked out with a meter and a continuity test,
and both were right: at zero current a resistive joint drops nothing. The same
trap is recorded in 4.24 ("every easily probed point is UPSTREAM of the fault").
The reading that mattered was the servo's own, under its own current:

| | id1 | id2 | id3 |
|---|---|---|---|
| torque OFF, no current | 12.2 | **12.4** | 12.4 |
| torque ON, holding | 12.2 | **5.3** | 12.5 |

And 1.4 ohm is still ~100x too high for the metal - 0.3 m of 18 AWG is 0.006 ohm -
so it is a termination, not the wire, and worth remaking regardless: on its own it
takes a stalling servo under the release. But it is the SECOND fault here, not the
first. Without a limit that commands the joint into a stop, nothing draws stall
current and 1.4 ohm never matters.

**This is 4.37's lesson one joint over, and it has now cost two sessions.** joint4
was limited by the harness; joint5 by a mechanical stop. In both cases the config
held a round, symmetric, plausible-looking number that the mechanism did not have,
and in both cases the arm found out the hard way. A limit the mechanism does not
have is not a conservative guess - **it is an instruction to drive into
something.** Every joint whose limits are still round and symmetric should be
treated as unmeasured until it has been through `--range` or `--limit-here`.

**Not measured by that run:** joint4 and joint6 showed 0.8 deg each because only
joint5 was moved. Those are not travel measurements, and the tool says so in its
own output.

### 4.45 One limit explained every failure of the session

2026-11-11, after joint5's limits were corrected to -45/+65 (4.44). Four
consecutive moves, all `SUCCEEDED`, with the new 2 Hz MEGA trace running:

| joint | peak \|ERR\| during a 17 s full-arm move | firmware watchdog |
|---|---|---|
| joint1 base | 0.70° | 8.0° |
| joint2 shoulder | **1.07°** | 8.0° |
| joint3 elbow | **1.11°** | 8.0° |

Supply held 12.1 / 12.3 / 12.4 V throughout, peak loads 83 / 58 / 0, and the
lowest any servo saw was 12.1 V against a 9.0 V release. No fault, no abort, no
sag.

**So the three "independent faults" were one cause.** joint5's limits allowed
39.5° of over-travel past its mechanical stop, and everything else followed:

    MoveIt plans joint5 past its stop
      -> the servo drives into the stop and stalls (~5 A)
      -> the rail sags; the DS51150s lose torque
         -> shoulder lags 25.5°, elbow 20.1° -> PATH_TOLERANCE_VIOLATED aborts
         -> the elbow passes 8° of its own error -> Mega tracking watchdog ->
            every servo relaxed
      -> and id2's own 1.4 ohm leg drops it under 9.0 V
         -> undervoltage release -> wrist torque off -> hardware deactivated

That is the same shape as 4.37, where one exceeded limit explained a fortnight of
"wandering" wrist dropouts. **A joint commanded into a stop is a system-wide
electrical event, not a local one**, because the stall current it draws starves
every other actuator on the same rail. Two joints on a different board, with
clean sensors, reported failures whose cause was a number in a YAML file.

**CORRECTION to 4.42: the shoulder is not faulty.** It was recorded as a third
independent fault, possibly needing re-gearing, on the strength of a 25.5° error
with a clean pot. It tracks to **1.07°** once nothing is stalling on its rail. The
25.5° was a consequence. The lesson is ordering: with several faults live at once,
the loudest is not the first, and a fault should not be called independent until
the known ones are cleared.

This also retires the torque hypothesis, so open questions #2 (the servo's stall
rating) and #3b (weighing a link) go back to ordinary curiosities - worth knowing,
no longer blocking. The shoulder demonstrably has the torque it needs.

**CORRECTION to 4.40: the elbow's pot noise was measured in the wrong state.**
The 4.11° came from `arm3-monitor`, which runs with **torque OFF**. With torque
ON, over a 130 s stationary stretch:

| joint | torque ON | torque OFF |
|---|---|---|
| base | 0.23° | 0.03° |
| shoulder | 0.14° | 0.03° |
| **elbow** | **0.53°** | **4.11°** |

The elbow is still the noisiest channel - 2-4x its neighbours driven, and ~8x
worse relaxed than driven, which is a real asymmetry nobody has explained (the
other two go the other way: driving them adds PWM noise). But **0.53° cannot trip
an 8° watchdog**, and its tracking error through a real move was 1.11°, the same
ballpark as the shoulder's. So it was not the cause of anything, and the
instruction to reflow that channel was premature: it would have meant soldering a
circuit that works.

What it may still cost, and the reason not to forget it: `on_activate` seeds the
commands from the measured position, and that read happens while the arm is still
relaxed. A 4° error at that instant becomes a 4° lurch on enable. Worth watching
for; not worth a repair on today's evidence.

**The general lesson, and it has now cost two sessions.** Measure in the state
that matters. A sensor read with the loop off, a voltage read with no current
flowing (4.44), and a load register read from a browning-out servo (4.44) were
each perfectly accurate measurements of the wrong condition - and each one sent a
session off after the wrong component.

### 4.46 Two wrist legs are non-ohmic, which is why the meter and the servo disagreed

2026-11-11, after 4.45's clean moves. With the stalls gone, what remained showed
itself plainly:

    t+622  id2 10.4 V / load 0   id1 12.2   id3 12.4
    t+635  id1 10.0 V / load 0   id2 12.3   id3 12.4     <- the low one swapped
    t+809  !! FAULT id 2 servo stopped answering -> wrist torque off -> deactivated

| | deficit at LOAD 0 | implies at 0.18 A idle |
|---|---|---|
| id1 | 2.35 V | ~13 ohm |
| id2 | 1.90 V | ~11 ohm |
| id3 | **none, in any reading all day** | - |

**The meter said 0.7 ohm per wire on id2 - 1.4 ohm for the loop - and both
readings are right, because a failing crimp is not ohmic.** A few strands in
contact measure ~1 ohm at a multimeter's milliamp test current and behave like
tens of ohms at amps, rising further as they heat. The meter and the servo probe
the same joint at currents three orders of magnitude apart and get different
answers. That also explains the ending: a stable resistance does not become a dead
open, but a few strands do.

This retires the apparent contradiction that cost most of the session. It joins
the other three measurement-condition traps from today: a pot read with the loop
off (4.45), a voltage read with no current (4.44), and a load register read from a
browning-out servo (4.44). **Every one was an accurate measurement of the wrong
condition.**

**id3 is the control, and it exonerates everything shared.** Same pack, same
board, same bus, same harness run, same session, never once low. So this is not
the pack (#0), not the board, not the ST3215 bus, and not the star topology
(4.43) - it is two terminations, alternating which is worse, both degrading across
the session. Repair: remake id1's and id2's power legs, both ends, with the
helical service loop of 4.38, because joint4 winds that bundle on every move and
has additionally been hand-rotated a full turn (4.44).

**Instrumentation added.** Nothing logged the wrist's joint ANGLES - the MEGA line
carries joints 1-3 and SUPPLY carries the wrist's volts and load - so a
START_STATE_INVALID could only be narrowed to "a wrist joint, or a collision".
There is now a WRIST line at 2 Hz giving each wrist servo's counts AND degrees:
counts because a joint sitting on the 0/4095 seam shows only there, degrees
because a joint outside its limit shows only there.

**OVERSTATED, and corrected here.** That line was then used to announce the
START_STATE_INVALID as "found": joint6 at 4039 counts, on its -175 deg limit. Two
reasons that is not proven. First the arithmetic: 4039 counts is **-174.9901 deg**
against a -175.0000 deg limit, so it is INSIDE by 0.0098 deg - about a tenth of
one encoder count, but inside, and MoveIt's bounds check should accept it. (The
first check called it "outside" only by comparing the log's rounded -175.0 with
the exact limit - the same rounding trap, in the tool written to avoid it.)
Second the timing: that reading came from the session AFTER the failures, so it
describes where the arm was found, not provably where it was when planning was
refused - a fair inference, since the arm does not move while the stack is down,
but an inference.

So a joint parked a tenth of a count from its limit remains the leading suspect
and the cause is still open. What is certain is that the planner may legitimately
drive joint6 to exactly -175 deg and park there, leaving no room - and this
project's own convention already says not to allow that: joint3 keeps 4 deg off
its stops and joint5 keeps 5 deg (4.44), for exactly this reason. joint6 has 57
counts of SEAM margin but no margin as a GOAL. If it recurs, the WRIST line now
names it within the same session.

### 4.47 An UPSTREAM collapse, told apart from a bad leg by its shape

2026-11-12. The wrist browned out again, and the per-servo telemetry said
immediately that it was **not** the fault chased all of 2026-11-11:

    t+0 ms    id1  9.9   id2 10.0   id3 10.0    spread 0.1 V   loads 27 / 0 / -19
    t+10      id1  9.6   id2  9.7   id3  9.8    spread 0.2 V
    t+20      id1  9.4   id2  9.5   id3  9.6    spread 0.2 V
    t+30      id1  9.2   id2  9.3   id3  9.3    spread 0.1 V
    t+50      id1  8.7   id2  8.8   id3  8.8    -> !! FAULT id 1 supply below 9.0 V

3.7 V in 50 ms, and **the spread never exceeded 0.2 V**. The two signatures are
now worth stating as a rule, because one session was spent on each:

| shape | meaning |
|---|---|
| **gradient** - one servo low, its neighbours fine | THAT LEG is resistive (4.46: id1 2.35 V, id2 1.90 V deficits, id3 never low) |
| **together** - all three fall in lockstep | **UPSTREAM**: the pack, its main lead, or a shared connector |

This one is the second shape, so id1's leg - still unrepaired and still showing
10.4 / 10.1 V at load 0 earlier in the same session - is **not** the cause here.
Nor is the wrist drawing it: loads were 27 / 0 / -19, essentially nothing. The
current went to the Mega's three DS51150s, on the rail that still has no
instrumentation. That makes open question **0m** the reason this cannot be taken
further from telemetry alone, for the third session running.

**It then did it again four minutes later, and that run settles the diagnosis.**

| event | id1 | id2 | id3 | spread | shape |
|---|---|---|---|---|---|
| t+206 | 10.1 | 12.3 | 12.4 | 2.3 V | LEG - id1 resistive |
| t+446 | **8.4** | **8.3** | **8.3** | **0.1 V** | **UPSTREAM - together** |

Both signatures in one log four minutes apart, which is as clean a demonstration
as this project is likely to get that they are different faults. Two further
things make the pack itself, rather than a resistive main lead, the answer:

* **The loads at the collapse were 20 / 33 / 23 of 1000** - about 3% of stall. A
  sound pack does not sag 4 V at 3% of stall, whatever the lead.
* **id1 came out HIGHER than its neighbours** (8.4 vs 8.3). Its own 2.3 V leg
  deficit had become irrelevant next to the collapse, which is what it looks like
  when the dominant impedance moves upstream of everything local.

**And the pack is being damaged by continuing.** 8.3 V on 3S is **2.77 V/cell**,
under the usual 3.0 V/cell floor - which is exactly where UNDERVOLT_TENTHS = 90
is set. Runs below that cost permanent capacity, so testing stops until it is
charged. The discriminator remains a repeat on a FULL pack: if it still collapses
at 3% load, the limit is the C-rating or the main lead and open question 0m
becomes the only way forward, because the current is going somewhere nobody can
measure.

**Third occurrence of open question #0, with the same tell.** It recovered on a
power cycle with nothing replaced, which #0 already records as pointing at the
pack's protection rather than a dead cell. Two things favour charge or pack
condition over a resistive main lead: the arm had been running for hours, and the
enable-inrush minimum had crept to **9.9 V** this session against a 9.0 V release -
almost no headroom before the arm moved at all. After the power cycle it was
10.5 V. The discriminator is a repeat on a FULL pack: if it still collapses, the
limit is the C-rating or the main lead, not the charge.

**Also caught, and this is the instrumentation paying off twice in one session.**
Before the collapse, the new WRIST line showed joint4 at 3074 counts =
**-90.1757 deg**, against a -90 deg limit: 2 counts outside, so MoveIt refused
every plan with START_STATE_INVALID. That is the same failure 4.46 suspected of
joint6 and could not prove - a joint driven to its limit and parked there, unable
to plan its way out. Now observed directly, and computed from integer counts
rather than the log's rounded degrees, which is the trap 4.46 fell into. Open
question **0p** is therefore no longer a suspicion: a joint CAN be parked at or
past its limit by ordinary operation, and the arm cannot recover by itself.

### 4.48 The margins were the wrong fix, and MoveIt has no tolerance to configure

Three sessions were lost to joints parked a fraction of a degree outside their
limits (4.46, 4.47, 0p). The plan was to give every joint a margin so it would
stop happening. **That reasoning was wrong, and the margins do not fix it.**

A margin moves the boundary; it does not stop the planner putting a goal ON the
new boundary. The mechanism is the interaction of two settings that are each
correct on their own:

* MoveIt's `CheckStartStateBounds` adapter refuses to plan if any start-state
  joint is outside its URDF limits, **by any amount**;
* the `JointTrajectoryController` declares a goal reached anywhere inside its
  goal tolerance - 1.5 to 2.6° here.

So a goal at a limit may settle a degree past it, report **SUCCEEDED**, and
leave the arm somewhere the planner will not plan from. A successful move bricks
the arm, and only a hand-move recovers it.

**`fix_start_state` is not the knob,** though its name says otherwise. Tested on
the arm on 2026-11-14: the parameter read back `True` and the adapter still
aborted. Upstream (moveit2 2.12.4) it only NORMALISES continuous, planar and
floating joints; a bounded revolute joint that is out of range aborts the
pipeline either way. There is no tolerance to configure.

**Reproduced deliberately before fixing anything.** joint1's upper limit was set
to 0.63° below where the arm was actually sitting - the same violation joint6 had
in 4.47 - and the failure appeared verbatim:

```
Joint 'joint1' from the starting state is outside bounds by: [0.00558505]
  should be in the range [-2.0944], [-0.005].
PlanningRequestAdapter 'CheckStartStateBounds' failed, because
  'Start state out of bounds.'. Aborting planning pipeline.
```

**The fix is in the driver:** `ArmSystemHardware::reported_position()` pulls a
measured position back inside its limits when it is less than `kReportClamp`
(0.05 rad, 2.9°) outside them, and reports the TRUE value beyond that with a
throttled warning. 2.9° is >= every joint's goal tolerance AND >= every joint's
measured backlash (0.52-1.08°), so inside that band the clamp is smaller than
the arm's own measurement noise - the clamped value is no less true than the raw
one. Beyond it, a real excursion still stops the planner, which is what should
happen.

Both branches were tested on joint6 with a temporary limit:

| raw violation | reported | warning | plan |
|---|---|---|---|
| 0.22° (< clamp) | exactly the limit | none | **SUCCESS** |
| 5.73° (> clamp) | the true value | logged, 5 s throttle | correctly **REFUSED** |

**What the margins are actually for.** They are not wasted - they are what makes
the clamp safe. Every limit now sits at least `goal tolerance + 3°` inside the
nearest hard boundary (firmware soft limit, measured stop, or encoder seam), so a
reading clamped by the full 2.9° is still >= 2.1° clear of anything that could be
damaged. joint1 and joint2 had NO margin at all before this - their limits were
exactly the firmware's own soft limits, leaving the backstop nothing to back up.

| joint | was | now | hard boundary | margin |
|---|---|---|---|---|
| joint1 | ±120 | **±115** | firmware ±120 | 5.0° |
| joint2 | ±90 | **±84** | structure/firmware ±90 | 6.0° |
| joint3 | −115.5/+122.5 | **−113.5/+120.5** | firmware −119.5/+126.5 | 6.0° |
| joint4 | ±90 | ±90 (unchanged) | harness, UNKNOWN | n/a |
| joint5 | −45/+65 | −45/+65 (unchanged) | stops −50.5/+70.8 | 5.5/5.8° |
| joint6 | ±175 | **±170** | encoder seam ±180 | 10.0° |

joint4 is deliberately unchanged: its boundary is the harness, which cannot be
measured by rotating the joint (4.37, 4.38), and the SRDF's `ready` pose needs
+78.9° of it - so tightening would park a routine pose against the new boundary,
the exact condition being eliminated. The rule is enforced by three new tests in
`test_limits_match.py`, one of which ties `kReportClamp` to the margins so the
C++ constant and the YAML cannot drift apart. The first attempt at these margins
was out by 0.1° on joint3 and the test caught it.

### 4.49 The pack does not "cut out" - the BASE oscillates, and that is the load

Open question 0 has blamed the pack since September: it "cuts out and recovers
untouched". On 2026-11-14 the pack was charged to full and it happened anyway, so
**charge is not the cause** - and watching the right trace showed what is.

The base limit-cycles while HOLDING, and it diverges. With the host's setpoint
pinned at +8.00° and no commanded motion at all:

```
   t(s)     meas      set      ERR
   48.5    +7.98    +7.98    +0.00
   49.5    +8.31    +7.99    -0.32
   50.6    +8.43    +8.00    -0.43
   51.1    +7.45    +8.00    +0.55
   51.6    +8.93    +8.00    -0.93
   52.6    +5.38    +8.00    +2.62
   53.1    +4.52    +8.00    +3.48      <- 8.0 deg trips the watchdog
```

`set` never moves, so this is entirely the Mega's inner loop - not the host, not
the planner, and not the limits changed in 4.48. The sign alternates, so it is an
oscillation and not drift, and it grew 35x in five seconds. An earlier run the
same day reached **|ERR| 7.52°** against the firmware's 8.0° `FAULT_ERR_DEG`.

**This is a known mode and the firmware says so.** arm3.cpp, on why `kd = 1.5`
was added to the base alone in 2026-09-28: *"the base - the one joint with no
gravity load for the integral to work against - limit-cycled at 0.34 Hz. Its
amplitude also scales with supply: 0.39 deg p-p measured on a sagging 11 V pack,
1.24 deg on a healthy 12 V one, because more volts mean more authority inside the
backlash where the output shaft cannot respond."*

**That scaling is the whole story.** `kd = 1.5` was tuned against an 11-12 V
pack. The pack is now genuinely healthy for the first time (12.6 V = 4.2 V/cell,
a full 3S), the base therefore has more authority than the damping was ever
sized for, and the limit cycle has gone from 1.24° p-p to divergent. **The arm
got worse because the power got better** - which is why months of blaming the
pack found nothing.

**And the oscillation is the load that collapses the pack.** The wrist telemetry
timestamps the causal order. Volts hold while the base error grows, then sag as
it diverges:

```
SUPPLY 12.3 12.5 12.6 V    <- base error already growing
SUPPLY 12.1 12.2 12.2 V
SUPPLY 11.8 12.0 12.1 V    <- 0.5 V, all three together
...then, faster than the 2 Hz trace: 10.0/10.0/9.9 -> 9.0/8.9/8.8
ESP32 says: !! FAULT id 2 supply below 9.0 V
```

All three fall TOGETHER with a small spread, which by 4.47's rule is upstream of
the wrist chain - the pack, its main lead or a shared connector. So open question
**0 is answered**: the limit is the C-rating or the main lead, NOT the charge,
and the trigger that exposes it is the base oscillation drawing a large
alternating current. It also explains the old puzzle of why the cut-outs started
when all six joints were first driven: that is when the base spends real time
energised and holding.

The hazard is worth stating plainly. At 8.0° of error the Mega relaxes **every**
servo, so an extended arm drops. The base reached 7.52° of 8.0°.

**CORRECTION, same day, before any gain was touched.** The paragraphs above
blame the base's damping and propose raising `kd`. **Measurement says that is
wrong, and the gain was deliberately NOT changed.**

With the Mega holding ON ITS OWN - no ros2_control, wrist unpowered - the base is
stable for as long as it is asked to be:

| hold | base spread | base \|err\| max |
|---|---|---|
| 90 s | 0.68° | 0.50° |
| 280 s | 0.69° | 0.44° |

Over the 280 s run joints 2 and 3 drifted (-0.28° and -1.00°) while the base did
not, which is the right way round: they hold against gravity and the base does
not. So the loop is NOT marginally damped at 12.6 V, and the supply-scaling
argument above, while quoted correctly from arm3.cpp, does not explain what is
happening now.

**It needs ros2_control, and then it is deterministic.** Stack up, arm holding,
nothing commanded: the base error crossed 1.5° at 145 s and the joint ran to
+10.65° at 165 s. The signature is not an under-damped oscillation at all:

```
   t(s)     meas      set      ERR    us(base)
  142-161   +0.4ish  +0.50    <0.19    1503     <- us PINNED, 40+ s, loop frozen
  161.8     +0.86    +0.50    -0.36    1494     <- it leaves the freeze
  162.8     +2.80    +0.50    -2.30    1334
  163.3     +9.66    +0.50    -9.16    1171
  165.3    +10.65    +0.50   -10.15    1187     <- past the 8.0 deg watchdog
```

Read `us` against the firmware's own `us = 1500 + deg x 7.4834`: 1171 us is a
command of about **-44°**. So the loop demanded a large NEGATIVE move while the
joint travelled POSITIVE, to +10.65°. That is not a joint oscillating about its
setpoint - it is the loop driving hard one way and the joint going the other,
after 40+ seconds of sitting motionless with `us` pinned at exactly 1503 (the
hold-band freeze of 4.18). Integral windup accumulated during the freeze is the
obvious suspect, but it is a suspect, not a finding.

Pot noise is not the explanation either. Torque off, 45 s, zero glitches on all
three channels: base 0.13°, shoulder 0.03°, elbow 0.02° peak-to-peak. The base is
4-6x noisier than its neighbours and is the only joint with a derivative term, so
it is worth remembering - but 0.13° is a quarter of the base's own 0.52° backlash
and cannot produce a 10° excursion. (This also retires the last of 0k: the elbow,
once measured at 4.11°, is now the CLEANEST channel at 0.02°.)

**The discriminator was run, and it eliminated BOTH of its candidates.**
`arm3-stream --amplitude 0` streams a constant target with the wrist NOT
energised. The rate was hard-coded at 50 Hz, which made it a weaker test than the
thing it stands in for, so `--rate` was added (ros2_control's controller_manager
runs at `update_rate: 100`):

| run | rate | duration | base \|tracking err\| max |
|---|---|---|---|
| Mega alone, no stream | - | 280 s | 0.44° |
| constant target streamed | 50 Hz | 220 s | 0.82° |
| constant target streamed | 100 Hz | 220 s | 0.35° |
| **ros2_control** | 100 Hz | **ran away at ~162 s** | **10.15°** |

So it is not the base loop, not the streamed setpoint, and not the command rate -
even though at 100 Hz roughly **1.9% of received lines failed checksum**
(rx_bad 397-408 of ~20600) and an XOR8 lets about 1 in 256 corrupted lines
through. That was the leading hypothesis and the data does not support it.

**CORRECTION, 2026-11-15: the fault is INTERMITTENT, which weakens every
elimination below.** After the eliminations were written, ros2_control itself -
the exact configuration that ran away at 145 s and at 162 s - held for **265 s
with the base error never exceeding 0.43°**, host target pinned at a single value
(+4.80) for all 579 traced samples. So the trigger is a RARE EVENT, not a
condition that is either present or absent.

That matters because each "stable" run below is 220-280 s long. A single clean
run of that length does not eliminate a trigger that fires once every few
minutes at random - it is evidence, not proof, and the eliminations should be
read as **"did not reproduce in one 220 s run"** rather than "ruled out". The
honest position is that nothing is yet ruled out, and that the two original
failures may simply have been where the dice landed.

**What IS settled by arithmetic**, independent of how often the fault fires, is
the transmit-load hypothesis - that a corrupted `G` line passing its XOR by
chance is being acted on. The firmware documents both frame sizes: ASCII is
~110 bytes, binary 34. So:

| | frame | rate | bytes/s |
|---|---|---|---|
| the stand-in test (`F 50`) | ~110 B | 50 Hz | **5500** |
| ros2_control (`B 100`) | 34 B | 100 Hz | **3400** |

The stand-in transmits **62% more** than the configuration that fails, and the
firmware notes ASCII at 50 Hz pushes the worst-case control block to 3196 of
5000 us, so it loads the Mega's CPU harder too. The failing case has MORE link
and CPU margin, not less. Whatever the trigger is, it is not link stress.

**The instrumentation that will settle it next time.** The binary frame has
always carried the host's `target` alongside the firmware's filtered `set`, and
only `set` was being printed - which is exactly why the runaway could not be
attributed. With `set` pinned at +0.50 there was no way to tell a HOST that sent
something wrong from a LOOP that misbehaved on a good target, and those need
different fixes. The MEGA trace now prints `tgt` as well. Next occurrence:

* `tgt` jumps -> the host sent it, or a corrupted line was accepted;
* `tgt` steady while `set`/`meas` diverge -> the firmware's loop or the servo.

**The wrist is not obviously the trigger either**, which is the uncomfortable
part. Its telemetry is flat right up to the event and only reacts afterwards -
i.e. it is shaken BY the base, not shaking it:

```
   t(s)   id1   id2   id3 | load1 load2 load3 | base ERR
  161.8  12.3  12.4  12.5 |     0     0   -19 |   -0.36   <- onset, wrist flat
  162.3  12.3  12.5  12.5 |     0     0   -21 |   +0.69
  162.8  12.1  12.2  12.3 |   -30    43   -37 |   -2.30   <- wrist reacts AFTER
```

But "flat in its own telemetry" is not the same as "drawing no current", so the
wrist being ENERGISED remains the last untested difference. **That is the next
test:** energise the three wrist servos on their own (the ESP32's `hold <id>`,
no stack) and run `arm3-stream --amplitude 0 --rate 100 --seconds 220` against
them. A first attempt at a standalone wrist-hold helper failed on the ESP32's
`scan` timing out, so the helper needs finishing first.

Do not touch a gain before that answers. `tools/motor_test.py arm3-hold --gains
N:KP,KI,KD[,KV]` was added for the tuning that this correction postponed; the
firmware's `k` writes to RAM, so anything tried there is undone by a power cycle
and has to be written into arm3.cpp to persist.

Also seen in the same session, and unexplained: `move_group` exited with
**SIGSEGV** as the hardware deactivated.

## 5. Open questions — things that need a measurement, not a decision

Ordered by how much they currently cost.

| # | Question | Why it matters | How to settle it |
|---|---|---|---|
| 0q | **The base runs away while HOLDING under ros2_control** (4.49), reaching +10.65° and past the 8.0° watchdog. **INTERMITTENT** - seen at 145 s and 162 s, but the same configuration then held 265 s clean with a worst error of 0.43°, so it is a rare event rather than a standing condition. Candidates that did NOT reproduce in one 220-280 s run each: the base loop standalone, the streamed setpoint at 50 Hz, the command rate at 100 Hz, pot noise (0 glitches), and the energised wrist. Treat those as evidence, NOT as ruled out | At 8.0° the Mega relaxes EVERY servo, so an extended arm drops. The arm cannot be trusted to hold a pose unattended until this is understood | **The trace can now attribute it, so WAIT for the next occurrence rather than guessing.** The MEGA line prints the host's `tgt` beside the firmware's `set`: `tgt` jumps -> the host sent it, or a corrupted line was accepted; `tgt` steady while `set` and `meas` diverge -> the firmware loop or the servo. Run with `ARM_LOG_SUPPLY=1` so the trace is on. Do NOT tune a gain first - the loop was stable for 280 s standalone. Transmit load is already excluded by arithmetic (4.49) |
| 0 | ~~**ANSWERED 2026-11-14 (4.49): it is not the pack cutting out, it is the BASE oscillating, and that oscillation is the load.**~~ On a FULLY CHARGED pack (12.6 V = 4.2 V/cell) the wrist still fell 12.3/12.5/12.6 -> 10.0/10.0/9.9 -> 9.0/8.9/8.8 V, all three together, and tripped the 9.0 V release. So the limit is NOT the charge. **Nor is it the pack's C-rating:** it is a 30C 8000 mAh 3S, i.e. 240 A continuous, against an arm that draws single-digit amps - three orders of magnitude of headroom (confirmed by the owner 2026-11-14). That leaves the WIRING. Dropping ~3.8 V at a few amps implies on the order of 0.5-1 ohm in series, which is nowhere near a healthy pack's milliohms - it is a main lead, a pack connector, or a shared distribution point. The trigger is the divergent base limit cycle drawing a large alternating current - which also explains why the cut-outs began only once all six joints were driven, i.e. once the base spent real time holding. **What remains of this question:** with the C-rating ruled out (30C 8000 mAh = 240 A), the series resistance is in the main lead, the pack connector or a shared distribution point, and has to be found and remade. Measure voltage AT THE PACK TERMINALS while the arm browns out: if the pack holds 12 V there while the servos see 9 V, the fault is entirely in the wiring between them. Superseded description: | 2026-10-05: all three wrist servos fell together (9.2/9.3/9.4 then 8.6/8.7/8.8 V - uniform, so upstream of the chain), the undervoltage release let go at 9.0 V, and then BOTH rails died. Power returned next day with no part replaced, exactly as on 2026-09-28 (4.22). A pack that recovers untouched points at its protection tripping, not a dead cell - and the trigger is new: this is the first time all six joints have been driven | Charge it, then drive a full sweep watching `grep "servo supply" /tmp/arm_stack.log`. If it trips on a FULL pack, the limit is the pack's C-rating or the main lead, not its charge. The parallel-power rewire below helps either way, by removing the series resistance that makes the arm demand more headroom than it should |
| 0j | **Wrist power is daisy-chained and marginal: the far end dips to 9.3 V under load** | 0.3 V above the undervoltage release, and it moves to whichever servo is last in the chain - so it is resistance spread over series connectors, not one bad part (4.29). The acute fault is gone and the arm runs, but this is why it keeps coming back | Feed the three wrist servos' POWER in parallel from the board (star or short bus) and daisy-chain only the data line. Verify with `grep "servo supply" /tmp/arm_stack.log` - all three should stay within a few tenths under load |
| 0i | ~~CONFIRMED: the fault is the cable to id3. Replace it~~ | **Superseded 2026-10-05**: the swap test showed the fault followed the POSITION, not the servo or any one cable | Found by inspection 2026-09-30, after the telemetry localised it. Until it is replaced the arm cannot be trusted for more than a few moves: the supply half was repaired and held, the open-under-flex half still drops id3 mid-motion | Replace the segment and route it with a service loop so wrist rotation does not work it loose. Verify with `tools/wrist_tune.py --watch 30` while flexing by hand (expect zero missed reads), then a full pose sweep at 30% and 50% |
| 0h | ~~2026-09-30 after the repair: id3's SUPPLY is fixed; an intermittent OPEN remains~~ | The two faults are different and only one is fixed. Supply: under load id3 now reads 12.1 V against neighbours at 11.9/12.0 - it was 3.6 V below them, so that gradient is genuinely gone, and five moves ran clean. Then id3 stopped replying mid-sweep with its supply at **12.0 V just 0.26 s earlier** - no sag at all, which is an open circuit rather than a resistance, and it only happens while the arm moves | `tools/wrist_tune.py --watch 30` prints which servo misses reads, live and with nothing energised. Flex each segment by hand - id3's connector, id2's, then along the lengths - and find the movement that does it. Then REPLACE that segment: re-seating does not repair a broken strand or a pulled crimp. Route it with a service loop so wrist rotation stops straining it |
| 0g | ~~FAILED 2026-09-30: wrist servo id3 no longer answers the bus at all~~ | The arm cannot be energised - and will not be, by design. Localised the day before from per-servo telemetry (`id1 11.2 / id2 11.3 / id3 7.6 V` under load, neighbours tracking within 0.1 V), then failed outright | Re-make the id2->id3 cable and both its connectors, or replace that segment. If id3 still does not answer, move its cable to id2's position to separate "cable" from "servo" - that is the only test that distinguishes them without probing. `tools/wrist_status.py` shows the answer immediately: id3 either reports a position and ~12 V, or nothing |
| 0f | ~~LOCALISED 2026-09-29: the bad connection is in the leg feeding wrist servo id3~~ | Reporting each servo's OWN supply instead of the minimum settled it in one move: under load `id1 11.2 V  id2 11.3 V  id3 7.6 V`. Neighbours track within 0.1 V while id3 drops 3.6 V, so the resistance is between id2 and id3 - that cable segment, its crimps, or id3's own connector. It also explains why a meter reads 12.2 V at the servos: every easily probed point is UPSTREAM of the fault, and why every "servo stopped answering" trip named id3 | Re-make that connection: unplug and inspect id3's power pins and the segment feeding it, re-crimp or replace it. id3 is the joint whose gripper was remounted, so the cable may have been strained then. Verify by repeating a loaded move and watching the three numbers converge |
| 0e | ~~A resistive power path: 12.2 V at rest, 7.7 V under load (§4.24)~~ | It is now THE blocker - eight consecutive moves failed on it in one run, and it is worsening across sessions (9.8 V -> 7.7 V for similar moves). Nothing else can be trusted or tuned until it is fixed | With the arm HOLDING a loaded pose, measure voltage at three points in turn: the pack terminals, the distribution/buck input, and a servo's own connector. The pair with the big difference between them contains the fault. Feel for a warm connector. Then: lead gauge, crimp quality, and the pack's C-rating/age |
| 0d | ~~The power path caps the arm at ~50% speed~~ | **Superseded by #0** - the same fault, measured more precisely | Measured 2026-09-28 with a charged pack: 12.1 V at rest, **10.2 V at 30%**, **9.7-9.9 V at 50%** - against a 9.5 V undervoltage release. 100% collapsed the rail to 4.9 V the day before. A 1.9 V drop at 30% is source impedance, not a flat cell | Measure the voltage AT servo 3's connector under load and compare with the pack terminals: the difference is the wiring. Check gauge, length and connectors (a warm connector is a resistive one). The base-pot-glitch theory in §4.22 is now **ruled out** - `tools/trace.py` found no single-sample step over 1° on joint1 across 30% and 50% moves, and a stalled servo would have shown one |
| 0c | ~~Servo power died entirely~~ | **Resolved 2026-09-28**: power came back with no part replaced, so it was the pack's protection or a connection, not a failed buck. The undervoltage release (§4.22) now stops the arm before it gets there |
| 0a | ~~The LiPo sags to 9.5 V under a static hold~~ | **Superseded by #0**: with a charged pack it held 12.1 V, so the sag was a symptom of the load event, not a tired cell | 3.17 V/cell on 3S, at the safe cutoff. It is the direct cause of servo 3 dropping off the bus during fast moves (§4.21), it caps how hard the arm can be driven, and continuing to use the pack in this state damages it | Charge it and repeat `tools/jitter.py` and a 100% move watching the `servo supply` line. If a full pack still sags, the pack is tired or the wrist's power run is too thin/long - measure the voltage AT servo 3 under load, not at the battery |
| 0k | **DOWNGRADED (4.45): the elbow's pot channel is noisiest but not faulty.** 0.53° driven vs 0.23/0.14 on its neighbours; the 4.11° figure was measured with TORQUE OFF and does not appear in operation. Do not reflow it on that evidence. Originally recorded as: 137x noisier than its neighbours (4.11° vs 0.03° with the arm stationary, 4.40) | It is THE blocker: it trips the firmware's tracking watchdog, which relaxes every servo and deactivates the hardware mid-session. Both of 2026-11-11's aborts were this | Inspect and reflow the elbow pot's wiper connection and its MCP6004 buffer channel (ADS1115 0x49) - the same circuit whose BASE channel failed in 4.35. Then re-run `motor_test.py arm3-monitor 90` with the arm stationary: the elbow must come down to the 0.03° its neighbours show. Do not tune or filter around it first - a loop chasing a bad sensor is what made this look like three different faults |
| 0m | **The Mega's servo supply is uninstrumented** | The wrist servos report their own volts, which is how 4.29 and 4.38 were solved. The three DS51150s report nothing, so "lost power" - one of the three causes the firmware names when it trips - cannot be distinguished from a jam or a bad sensor without a meter | Add a divider on a spare Mega analog pin reading the 12 V servo rail, and put it in the status frame next to the pot readings. Until then any Mega-side fault has one branch that can only be checked by hand |
| 0n | **id1's and id2's wrist power legs are non-ohmic partial contacts** (4.46) | THE remaining blocker. 2.35 V and 1.90 V deficits at LOAD 0, alternating which is worse, and id2 ended the session as a dead open. id3 has never been low, so nothing shared is implicated | Remake both legs, both ends, with the helical service loop of 4.38 - joint4 winds that bundle every move. Meter id1's wires too; only id2's have been measured (0.7 ohm each, which a meter cannot reconcile with 13 ohm behaviour because the contact is not ohmic). Verify with `ARM_LOG_SUPPLY=1`: all three within a couple of tenths at LOAD 0 |
| 0p | ~~**FIXED 2026-11-14 (4.48): a joint parked just outside its limit no longer stops the planner.**~~ `ArmSystemHardware::reported_position()` clamps a violation smaller than the arm's own backlash and reports the truth beyond it; both branches tested. The fix is NOT the margins added the same day - those only make the clamp safe. | The planner may park it exactly on +/-175 deg, leaving a tenth of a count of room, and a START_STATE_INVALID followed - suspected but not proven | Decide whether the limit should sit inside the seam margin the way joint3 and joint5 sit inside their stops. Needs one recurrence with the new WRIST line to confirm the cause first; do not narrow a limit on a suspicion |
| 0b | **ST3215 static error under load: 1.36°** | It sets the wrist's goal tolerance, and so the tool's achievable orientation accuracy (§4.21) | Tune each servo's own PID registers (21/22/23) and re-measure before tightening `ros2_controllers.yaml` |
| 1 | **How much cable slack does the wrist have?** (was "real travel of joint4-6") | ANSWERED differently than asked (4.37): the mechanism allows >= 150° on joint4 and the HARNESS allows less - the probe proved it by snapping the cable. So the question is not what the joint can reach but how much slack to fit. +/-90° is the proven-safe figure and is recorded as harness-imposed | **Being rebuilt, 2026-11-02 (4.38).** With the new loop fitted: roll the joint BY HAND with torque off until the loop just goes taut, then `tools/wrist_tune.py --limit-here <id>`, which converts that position through `actuators.yaml` and takes its margin. Do not probe for it. Fit the loop first, measure second, raise the limit third - in that order |
| 3b | **Weigh one printed link** (no longer blocking - 4.45) | `real_density` is an estimate (600 kg/m^3); everything mass-derived scales off it | Put the largest printed part on a scale and compare with the prediction in 4.25 - `base_link` 691 g, `Link1_Shoulder` 517 g. Say whether the part includes its servo, because then one density is an approximation |
| 2 | **DS51150 stall torque: 150 or 50 kg·cm?** (no longer blocking - 4.45) | `START_HERE.md` says "50 kg version"; `RESULTS.md` says 150 kg-cm throughout and the part number agrees. 14.7 N·m vs 4.9 N·m | Check the label on the servo |
| 3 | ~~Link masses are Fusion's "Steel" default~~ | **Fixed 2026-10-01** (4.25): scaled to printed plastic, 24.94 kg -> 1.953 kg. The density itself is still an estimate - see 3b | Total 25.5 kg for a hobby-servo arm is not plausible — `base_link` alone is 9.03 kg. Harmless for kinematic planning, fatal for any physics sim or gravity compensation | Weigh the links, or set real materials in Fusion and re-export |
| 4 | **True TCP** | `tool0` is derived from CAD geometry, not measured on the built arm | Only once the gripper is in use |
| 5 | **Gripper finger zero** | The CAD range is entirely negative; where is the finger actually closed? | Measure when the gripper is wired |
| 7 | ~~Pot cables: fix the wiring or keep the software map?~~ | **Decided 2026-09-18: wiring stays, software follows it.** Firmware, `FeedbackConfig.h` and `actuators.yaml` all use base 0x4A, shoulder 0x48, elbow 0x49 | — |
| 5b | **joint6's re-zeroed centre vs the model** | The servo's zero was re-indexed onto wherever the wrist stood after the remount (§4.19), which restored its travel but was not checked against the URDF. A residual there is a constant roll error on every tool orientation | Arm at home, compare RViz with the machine, fold the residual into `motor_counts_at_zero` — a count offset, no EEPROM write |
| 6 | **Does mid-travel really equal the upright pose?** | The whole joint-limit story rests on it. See 4.10–4.11 | Quick check: drive the shoulder to its structural ±90° stops; the upper arm should be horizontal both ways. Then the per-joint re-zeroing already on the firmware to-do list |

---

## 6. What is not done yet

### Blockers on touching real hardware

1. ~~Not in the `dialout` group~~ — **done.** Added, logged out and back in;
   both ports are read-write from a new login.
2. **The three-joint Mega firmware does not exist.** `START_HERE.md` lists it as
   "not started", and the last PlatformIO build in the tree is `s1_probe`. The
   single-joint `stage5_closed_loop.cpp` is the template and every per-joint
   constant is already recorded in `docs/SERVOS.md`. Needs no hardware to write.
3. ~~Sensing perfboard not built~~ — **done.** All three ADS1115s answer, all
   three pot excitations are good, and every motor has been moved and read
   (§4.14). The pot cables are rotated relative to the design.

Until 2 is done, `use_mock_hardware:=false` has nothing to talk to.
The ROS side is built so that switching over is one launch argument.

### Next steps, in order

1. **Three-joint Mega firmware** — extend `stage5_closed_loop.cpp` to three
   channels. No hardware needed. Define the serial protocol at the same time
   (see below) so the ROS side can be written against it.
2. **`arm_hardware` package** — a `ros2_control` `SystemInterface` that talks to
   both boards. Already referenced by the URDF as
   `arm_hardware/ArmSystemHardware`; the parameters it expects (`mega_port`,
   `mega_baud`, `esp32_port`, `esp32_baud`, `handshake_timeout_ms`,
   `fake_gripper`) are already declared in `arm.ros2_control.xacro`.
3. **Bring up joints one at a time** — base first, on mock for the other five.
4. **Re-tune loaded.** `RESULTS.md` is explicit that the unloaded gains will not
   survive contact with the arm; gravity changes droop, settling and `Ki`.
5. Then: Gazebo (needs real masses first), gripper, perception.

### Serial protocol — design note

The stage-5 firmware already has most of what a hardware interface needs:
`g <deg>` to command, `e`/`s` to enable/relax, `?` for status, CSV telemetry, and
crucially an explicit `## target` acknowledgement for every command. Extending it
to three joints wants:

- A joint index on every command: `g <n> <deg>`.
- One telemetry line carrying all three joints, at a fixed rate, so the
  interface's `read()` is one line parse rather than three round trips.
- The existing ack discipline kept. A silently dropped command that produces no
  error is the worst failure mode there is, and `RESULTS.md` records it
  happening.
- Fault state in the telemetry line, so `read()` can surface a relaxed servo as
  a ros2_control error instead of the arm quietly going limp.

Note the units boundary: the firmware works in **degrees**, ros2_control in
**radians**. Convert in exactly one place, in the hardware interface, and say so
in a comment.

---

## 7. Tools written for this project

| Tool | What it does |
|---|---|
| `arm_description/scripts/make_collision_meshes.py` | Audits collision geometry against the visual meshes; regenerates failures as convex hulls |
| `arm_moveit_config/scripts/analyse_self_collision.py` | Derives the SRDF collision matrix by exact convex separation over random configurations |
| `arm_moveit_config/scripts/workspace_map.py` | Prints a reachability map, marking where the tool can point down |
| `arm_moveit_config/scripts/ik_benchmark.py` | Benchmarks the installed IK solver against move_group — the evidence behind §4.7 |
| `arm_description/scripts/capture_zero.py` | Captures the pose set in RViz and bakes it in as the new joint zero |
| `arm_demos move_to` | Plan and execute from the command line, with three orientation modes |
| `tools/start_stack.sh` / `tools/stop_stack.sh` | Bring the live stack up and down correctly: stop interrupts so the boards are told to relax, start waits for `arm_controller` to be ACTIVE rather than for the arm to be merely enabled |
| `tools/wrist_tune.py` | Bench tuning for the ST3215s: read their own Kp/Kd/Ki and insensitive areas, change them, and measure the hunt before and after. This is what found the 1-count deadband |
| `tools/moving_ripple.py` | Per-joint smoothness DURING a move, for all joints at once — velocity ripple about its own moving average. Distinguishes a joint vibrating in place from one crossing its path in micro-jumps |
| `arm_demos cartesian_to` | Straight-line tool motion in Cartesian space, for approach and retract. Reports what fraction of the line it solved and refuses to execute a partial path, because stopping short of an approach target is the wrong place rather than a partial success |
| `tools/set_zeros.py` | Makes the arm's current pose the zero by recording where each MOTOR sits — for a mounting offset after reassembly. Averages over seconds and prints the spread, warns when a change is small enough to be droop rather than mounting, and takes `--joints` so only deliberately posed joints are captured. Not `capture_zero.py`, which moves the URDF's zero instead and would undo "upright at zero" |
| `tools/motor_test.py center-all` | Parks every motor at its mechanical centre and HOLDS it, for bolting links on during reassembly — mount each link upright from there and motor-zero is URDF-zero, with no offset to recover by eye afterwards |
| `tools/joint_check.py` | Moves each joint on its own and reports how well each followed — the "which joint is wrong" test after a reassembly or a servo swap. Refuses to run unless the arm is actually active |
| `tools/wrist_status.py` | Per-servo report from the ESP32 including SUPPLY VOLTAGE — the reading that separates a dead 5 V buck from a disconnected pack. Bench only: never with the stack up |
| `tools/trace.py` | One joint's commanded vs measured position at full rate, and whether a deviation was physical motion or the sensing dropping out (a multi-degree step inside 10 ms is not motion) |
| `tools/jitter.py` | Per-joint jitter while holding, from `/joint_states`: peak-to-peak, sd, drift and the dominant frequency that separates a control limit cycle from servo dither. Safe with the stack live — it only subscribes |
| `tools/track.py` | The controller's own following error during a move, from `/arm_controller/controller_state` — whether the arm actually keeps up with the plan |
| `tools/axis_check.py` | Prints every joint's axis in world frame at the home pose, so "is +q clockwise?" is answered from the model instead of by eye. This is what caught joint6's reversed direction |
| `tools/motor_test.py` | One-motor-at-a-time hardware test, `arm3-probe` to measure a joint's real limit under power (the only safe way - these joints are not back-drivable), `arm3-monitor` which also reports travel, `arm3-hold` for jitter measurement, `esp-centre` to re-index a wrist servo's zero, plus `estop` for both boards |
| `DS51150_test/src/motor_test.cpp` | Mega test firmware: safe at boot, dead-man, tracking and wrong-pot trips |
| `firmware/esp32_st3215/` | ESP32 firmware, two builds: `test` (per-servo bring-up, plus the one deliberate EEPROM write — `centre`) and `arm` (production, three joints, binary frames, dead-man, load trip, no EEPROM writes at all) |
| `firmware/esp32_stock_backup/` | Full 4 MB image of the ESP32's previous firmware, with restore steps |

Re-run the first two after **any** change to a mesh, a link or a joint limit.
Both are deterministic given a seed, and both print what they changed.

---

## 8. Conventions

- **Frames.** `world` is the root, z up, the base sits on z=0, the arm reaches
  along +X. `tool0` is the planning tip: +Z is the approach direction, +Y the
  finger-closing direction.
- **Units.** SI everywhere in ROS: radians, metres, N·m. The firmware uses
  degrees and microseconds; converting is the hardware interface's job.
- **Provenance.** Every number in `joint_parameters.yaml` is tagged
  `[MEASURED]`, `[PROVISIONAL]` or `[CAD]`. Keep it that way — the difference
  between a bench-verified limit and a guess is the whole point.
- **Measurement beats theory.** Carried over from the servo bring-up, and it has
  already earned its place twice here: the IK solver choice (4.7) and the
  velocity feedforward in `RESULTS.md`.
