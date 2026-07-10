# CEER in-browser demo — port spec (authoritative reference)

Source of truth: `ceer_deploy` (MuJoCo sim2sim deploy of the G1 CEER policy).
This file captures everything the JS control loop must reproduce. When in doubt,
trust **empirically captured golden vectors** from the running Python sim over
any reasoning below (see "Ground-truth capture").

## Pipeline (one control tick @ 50 Hz)

```
env state ──obs_adapter(env→policy order)──▶ build obs(257) ──(o-loc)/scale──▶ ONNX ──▶ action(29)
                                                                                          │
pd_target(policy order) = action*action_scales + default_pos   ◀── clamp/scale ──────────┘
        │ actions_adapter(policy→env order)
        ▼
env.step: for 20 substeps @ dt=0.001:  torque = (pd_target - dof_pos)*Kp - dof_vel*Kd,
          clip to torque_limits, data.ctrl = torque, mj_step
```

- Control rate: `freq = 50` Hz → control_dt = 0.02 s.
- Physics: `sim_dt = 0.001`, `sim_decimation = 20` (20 substeps per control tick).
  Set `model.opt.timestep = 0.001` after compile.
- `mj_forward` once for rendering each tick; PD torque recomputed every substep
  (pd_target held constant across the 20 substeps).

## MuJoCo scene (verified via smoke_load.mjs)

- XML: `public/scene/g1_29dof_table_box.xml`, meshes in `public/scene/meshes/*.STL`.
- Load path (official DeepMind `mujoco` npm binding, NOT `mj_loadXML`):
  `vfs = new mj.MjVFS(); vfs.addBuffer('meshes/'+name, uint8)` for every STL,
  `spec = mj.parseXMLString(xml)`, `model = mj.mj_compile(spec, vfs)`.
- Dims: nq=71, nv=65, **nu=29**, nbody=50, ngeom=126, nmesh=36, njnt=35.
  Robot: root free joint (7 qpos / 6 qvel) + 29 hinge joints. Remaining qpos/qvel
  are 5 free objects (box, ball, ellipsoid, cylinder, prism) @ 7/6 each.
- Furniture/objects present: table, table_back, box, ball, ellipsoid, cylinder,
  prism, bed, nightstand, chair, floor.

## Observation layout (257) — build order

| slice | dim | contents |
|---|---|---|
| boot | 1 | `boot_indicator / 25`; boot_indicator starts 25, −1 per tick to 0 |
| command | 6 | see below |
| root_and_wrist_6d | 12 | teleop EE cmd: `[lpos(3), rpos(3), l_axisangle(3), r_axisangle(3)]` |
| base_ang_vel | 3 | world angvel rotated into body frame |
| projected_gravity | 3 | `[0,0,-1]` rotated into body frame |
| joint_pos_history | 145 | 5 steps `[0,1,2,3,4]` × 29 dof, **policy order**, newest first, absolute joint pos |
| prev_actions | 87 | 3 steps × 29, raw clamped network outputs, newest first |

**command (6):**
1. `root_height` = constant **0.79**
2–3. `target_linvel_b` = teleop target root world x,y = `root_cmd7[:,0:2]`
   (NB: this is the teleop **target root position** x,y, not a velocity)
4–5. `target_heading_b_xy`: heading `[1,0,0]` rotated by target-yaw quat → world,
   then rotated into current robot yaw-frame; take xy.
6. `force_safe_limit` = constant **15.0**

Quaternion convention inside obs math is **wxyz**; MuJoCo qpos root quat is wxyz,
env base_quat is stored xyzw then converted. Gravity/angvel use
`quat_rotate(conj(base_quat_wxyz), v)`.

**Normalization:** `obs_norm = (obs - loc) / scale`, loc/scale in `assets/vecnorm.json` (dim 257).

## Action post-processing (per policy.get_action)

1. `act = onnx(obs_norm)` → 29.
2. `act_clamped = clamp(act, -10, 10)` → stored as newest `prev_actions[0]`.
3. EMA in raw space is **disabled** in code (`act_smooth = act_clamped`).
4. `act_scaled = act_clamped * action_scales` (per-joint, policy order — see meta).
5. Final applied action = EMA: `applied = lerp(applied, act_scaled, alpha)`,
   `alpha = action_beta = 0.9` (+ optional jitter ±0.025, clamp [0.825,0.975] —
   can disable for determinism in browser).
   Communication-delay emulation (max_delay=4, decim=4) is a training-robustness
   feature; **v1 browser may disable it** (set delay=0 → pure EMA).
6. `pd_target = applied + default_pos` (policy order), then reorder to env order.

## Joint orders & gains — RESOLVE EMPIRICALLY

Two orders exist and the pipeline does `env.update_dof_cfg(override=policy.action_dof)`
plus obs_adapter/actions_adapter reordering. The exact runtime-resolved arrays are
error-prone to derive by reading configs, so **capture them from the live sim**:

- `env_joint_order` (G1_29DoF): left leg6, right leg6, waist3, left arm7, right arm7.
- `policy_joint_order` (G1MyCustomFullBodyDoF): interleaved (see control_meta.json).
- `default_pos`, `stiffness`, `damping`, `torque_limits`, `action_scales`:
  all in control_meta.json, but VERIFY which order/values the running env.step
  actually uses (override interaction).

### Ground-truth capture (Phase 2 keystone)

Instrument `ceer_deploy` to dump, at runtime:
- resolved `env.dof_cfg.joint_names`, `env.stiffness/damping/torque_limits`,
  `policy.default_pos`, and the MuJoCo qpos/qvel index per joint.
- a sequence of golden tuples `(full_obs_257, action_29, pd_target_env_29)` for a
  few ticks under known teleop commands.
Use these as JS unit-test fixtures: JS obs-builder + onnx + post-processing must
reproduce `pd_target_env` to <1e-4.

## Teleop (Phase 3) — keyboard, mirrors teleop_dummy_pub.py

Root velocity-integrated; hands are body-frame targets. Keys:
- W/S ±0.2 m/s fwd/back, A/D ±0.2 m/s left/right (persist), SPACE = stop.
- F/H root up/down (z), Q/E yaw ±10°/press.
- I/K both hands ±x, J/L both hands ±y (apart/together), U/O both hands ±z.
The teleop produces the 28-float packet (root/head/left/right pos+quat, world/body
frame per policy._get_udp_control) that feeds `command` + `root_and_wrist_6d`.

## Assets in this app
- `assets/policy.onnx` (257→29, opset 17; torch-parity 3.5e-6)
- `assets/vecnorm.json` (loc/scale, 257)
- `assets/control_meta.json` (orders, gains, scales, constants)
- `scene/g1_29dof_table_box.xml` + `scene/meshes/*.STL`
