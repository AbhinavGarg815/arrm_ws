# CAD provenance — what came from Fusion, and what we changed

This package started as a verbatim copy of the output of the Fusion URDF/XACRO
Exporter v3.1.1. That export is kept **untouched** outside this repository as the
record; diff against it to see everything below.

Each entry says what the exporter produced, what it is now, and why. Anything
not listed here is exactly as exported.

---

## Unchanged

- All **mesh files** for visuals (`meshes/*/*.dae`).
- All **joint origin translations** (`xyz`).
- All **inertial** blocks — masses, centres of mass, inertia tensors. Note these
  are Fusion's default "Steel" material and total 25.5 kg, which is not
  plausible for a hobby-servo arm. Harmless for kinematic planning, wrong for
  any physics sim. Tracked in `docs/PROJECT.md` open questions.
- `robot_data.yaml`, `docs/transforms.md`, `config/frame_overrides.csv`,
  `config/FRAME_OVERRIDES.md` — kept as the exporter's own record. **They
  describe the export, not the current model:** they still show continuous
  joints and the CAD joint-origin rotations, before the zero offsets in §5.
  Read the current values from the xacro and `config/joint_parameters.yaml`.
- `config/joint_state.yaml` — unused; `joint_state_publisher` now reads limits
  from the URDF.

---

## 1. Package renamed

`arm_final_description` → `arm_description`, including every `package://` URI.
`urdf/arm_final.urdf.xacro` → `urdf/arm.urdf.xacro`;
`urdf/assemblies/arm_final.urdf.xacro` → `urdf/assemblies/arm_links.urdf.xacro`
(and the macro inside it, `arm_final` → `arm_links`).

`urdf/arm_final.urdf` — the flat URDF — is kept unchanged. It is the reference
the collision audit measures against.

## 2. Collision geometry replaced with convex hulls

**Exported:** one axis-aligned box per link, referenced as `*_collision.stl` at
`scale="0.01 0.01 0.01"`.

**Now:** convex hulls of the visual meshes, `*_convex.stl` at `scale="1 1 1"`,
for six of the eight links. `base_link` and `Finger_Moving` keep their original
boxes — they were already accurate.

**Why:** the exported boxes did not contain the parts. Up to **62.9%** of a
link's visual geometry lay outside its own collision box, protruding by as much
as **107 mm**. `Link4_Wrist1` and `Link5_Wrist2` had been given identical boxes.
Full measurements in `docs/PROJECT.md` §4.1.

Regenerate with `scripts/make_collision_meshes.py`. It audits first and only
rewrites links that fail, so it is safe after any CAD change.

Collision `<origin>` entries are unchanged: each hull is built in its visual
mesh's own frame, so it reuses the matching `<visual>` origin.

## 3. `world` link and base mount added

**Exported:** `base_link` is the root, in the Fusion design frame — where the
waist axis points along **+Y**. Loaded as-is the robot lies on its side.

**Now:** a `world` link with a fixed `world_joint` to `base_link`, values in
`config/joint_parameters.yaml` under `base_mount`.

Derived, not guessed:
- rotation maps `joint1`'s axis onto world **+Z** and `joint2`'s onto world
  **−Y** (those two axes are perpendicular to 0.0° in the CAD), plus 180° of yaw
  so the arm reaches along **+X**;
- `z` = lowest point of `base_link`'s collision mesh, so the base sits on z=0;
- `x, y` put the **waist axis on the world Z axis**. The CAD's `base_link`
  origin sits 217 mm off the middle of the pedestal.

## 4. Joints given real limits

**Exported:** all six revolute joints as `type="continuous"` — no limits. Fusion
has no joint-limit concept.

**Now:** `type="revolute"` with `lower`/`upper`/`effort`/`velocity` read from
`config/joint_parameters.yaml`, which is the single source of truth. Every value
there is tagged `[MEASURED]`, `[PROVISIONAL]` or `[CAD]`.

`joint1` and `joint2` limits come from the firmware's own soft limits
(`DS51150_test/lib/ServoFeedback/FeedbackConfig.h`). The rest are provisional.

## 5. Joint zeros shifted so home is upright

**Exported:** q=0 is whatever pose the CAD assembly was in — for this arm, bent:
the forearm 190 mm off the shoulder axis, the tool 20.1° off vertical.

**Now:** the `rpy` of all six joint origins is `cad_rpy * Rz(offset)`, with the
offsets recorded in `config/joint_parameters.yaml` under `zero_offsets_rad`. At
q=0 the arm stands as a vertical column.

Nothing else changes — no mesh, no translation, no limit value — but what the
limits mean *physically* does, which is why the self-collision matrix must be
regenerated after any change here.

The vertical stacking was solved numerically; the wrist orientation was set by
hand at the machine, because `joint4` and `joint6` are roll joints that the
solve cannot constrain. `scripts/capture_zero.py` does the capture.

## 6. `tool0` frame added

**Exported:** nothing — the last link is `Link6_Gripper`, whose origin is a CAD
frame inside the casting.

**Now:** a fixed `tool0` at the fingertip, +Z along the approach direction and
+Y along the finger travel (ROS-Industrial convention). Derived from the gripper
geometry; provisional as an absolute TCP until measured on the built arm.

## 7. `ros2_control` rewritten

**Exported:** two `<ros2_control>` blocks — one in the top-level file, one in
the assembly macro — both named `arm_final_system`, which `controller_manager`
rejects as a duplicate. Between them they covered five of the seven joints.

**Now:** one block in `urdf/arm.ros2_control.xacro` covering all seven, with a
`use_mock_hardware` argument selecting `mock_components/GenericSystem` or the
real `arm_hardware/ArmSystemHardware`.

`Slider_39`'s mock `initial_value` changed from `0.0` to `-0.065`: its range is
entirely negative, so `0.0` was outside its own limits and made every start
state invalid.

## 8. Controllers, launch and RViz

- `config/ros2_controllers.yaml`: two `ForwardCommandController`s replaced by a
  `JointTrajectoryController` per group, which is what MoveIt drives. Tolerances
  sized from the measured servo accuracy.
- `launch/display.launch.py`: rewritten. `robot_description` now wrapped in
  `ParameterValue(..., value_type=str)`, without which launch tries to parse the
  URDF as YAML.
- `rviz/display.rviz`: fixed frame `base_link` → `world`.

## 9. Added

- `config/joint_parameters.yaml` — limits, base mount, zero offsets.
- `scripts/make_collision_meshes.py`, `scripts/capture_zero.py`.
- `test/test_description.py`, `test/test_configs_parse.py` — regression tests,
  one per defect above that could silently return.
