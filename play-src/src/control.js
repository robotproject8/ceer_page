// control.js — CEER policy control math, ported 1:1 from the Python deploy
// pipeline (robojudo MyCustomPolicy.get_observation / get_action / get_pd_target
// + PolicyWrapper). Framework-agnostic pure logic: no MuJoCo, no ONNX runtime —
// the caller supplies MuJoCo state and an ONNX "run(normObs)->rawAction" fn.
//
// Verified tick-by-tick against scripts/capture_golden.py fixtures
// (test/golden.json) to <1e-4 by control_test.mjs.
//
// Pipeline per 50 Hz control tick:
//   stepUpdateBuffers(dofPosEnv)             // history + prev-action + boot decay
//   obs(257) = assemble(command, wrist6d, angVel, gravity, jointHist, prevAct)
//   norm = (obs - loc) / scale
//   raw(29) = onnx(norm)                      // policy joint order
//   clamped = clamp(raw, -10, 10)             // becomes next prev_action[0]
//   applied = lerp(applied, clamped*scales, action_beta)   // EMA, stateful
//   pdPolicy = applied + default_pos          // policy order
//   pdEnv = reorder(pdPolicy, policy->env)    // env / MuJoCo qpos order

// ---- quaternion helpers (wxyz) --------------------------------------------
function qNorm(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) + 1e-8;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
function qConj(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}
function qMul(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}
// Rotate vec v by quaternion q (wxyz), matching policy._quat_rotate_wxyz.
function qRotate(q, v) {
  const qn = qNorm(q);
  const vq = [0, v[0], v[1], v[2]];
  const r = qMul(qMul(qn, vq), qConj(qn));
  return [r[1], r[2], r[3]];
}

function clamp(x, lo, hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

/**
 * @param {object} meta  the `meta` block from golden.json (runtime-resolved
 *   joint orders / gains / scales / default_pos), plus vecnorm loc/scale.
 * @param {number[]} loc  vecnorm loc (257)
 * @param {number[]} scale vecnorm scale (257)
 */
export class Controller {
  constructor(meta, loc, scale) {
    this.meta = meta;
    this.loc = Float64Array.from(loc);
    this.scale = Float64Array.from(scale);

    this.numDof = 29;
    this.histSteps = meta.joint_hist_steps || [0, 1, 2, 3, 4];
    this.maxHist = Math.max(...this.histSteps);
    this.prevActionSteps = meta.prev_action_steps ?? 3;
    this.bootMax = meta.boot_max ?? 25;
    this.actionBeta = meta.action_beta ?? 0.9;
    this.actionScales = Float64Array.from(meta.action_scales);
    this.defaultPosPolicy = Float64Array.from(meta.policy_default_pos);

    // env(MuJoCo qpos) -> policy-obs order map for joint_pos_history.
    // obs_adapter maps src=env -> tar=policy; dof_pos_policy[tar] = dof_pos_env[src].
    this.envToPolicy = new Int32Array(this.numDof); // policyIdx -> envIdx
    {
      const a = meta.obs_adapter;
      for (let k = 0; k < a.src_indices.length; k++) {
        this.envToPolicy[a.tar_indices[k]] = a.src_indices[k];
      }
    }
    // policy-action -> env order map for pd_target.
    // actions_adapter maps src=policy-action -> tar=env; pd_env[tar]=pd_policy[src].
    this.policyToEnv = new Int32Array(this.numDof); // envIdx -> policyIdx
    {
      const a = meta.actions_adapter;
      for (let k = 0; k < a.src_indices.length; k++) {
        this.policyToEnv[a.tar_indices[k]] = a.src_indices[k];
      }
    }

    this.reset();
  }

  reset() {
    const H = this.maxHist + 1;
    this.jointHist = Array.from({ length: H }, () => new Float64Array(this.numDof));
    this.prevActions = Array.from({ length: this.prevActionSteps }, () => new Float64Array(this.numDof));
    this.applied = new Float64Array(this.numDof);
    this.lastRawAction = new Float64Array(this.numDof);
    this.bootIndicator = this.bootMax;
    this.histInitialized = false;
  }

  // Reorder env-order dof_pos into policy-obs order.
  _dofEnvToPolicy(dofPosEnv) {
    const out = new Float64Array(this.numDof);
    for (let p = 0; p < this.numDof; p++) out[p] = dofPosEnv[this.envToPolicy[p]];
    return out;
  }

  // Mirror MyCustomPolicy._step_update_buffers (called at obs-build time).
  _stepUpdateBuffers(dofPosEnv) {
    const dofPolicy = this._dofEnvToPolicy(dofPosEnv);

    if (!this.histInitialized) {
      for (let k = 0; k <= this.maxHist; k++) this.jointHist[k].set(dofPolicy);
      this.histInitialized = true;
    }
    // roll +1 (newest at index 0)
    for (let k = this.jointHist.length - 1; k > 0; k--) this.jointHist[k].set(this.jointHist[k - 1]);
    this.jointHist[0].set(dofPolicy);

    for (let k = this.prevActions.length - 1; k > 0; k--) this.prevActions[k].set(this.prevActions[k - 1]);
    this.prevActions[0].set(this.lastRawAction);

    this.bootIndicator = Math.max(0, this.bootIndicator - 1);
  }

  /**
   * Build the unnormalized 257-dim observation.
   * @param {object} s state:
   *   command        [6]  (root_height, linvel_b xy, heading_b xy, force_limit)
   *   wrist6d        [12] root_and_wrist_6d (teleop EE cmd)
   *   baseQuatXYZW   [4]  MuJoCo base orientation (xyzw)
   *   baseAngVelWorld[3]  world-frame base angular velocity
   *   dofPosEnv      [29] joint positions in env / MuJoCo qpos order
   * @returns {Float64Array} obs (257)
   */
  buildObs(s) {
    this._stepUpdateBuffers(s.dofPosEnv);

    const qxyzw = s.baseQuatXYZW;
    const q = qNorm([qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]]); // -> wxyz
    const qc = qConj(q);
    const angVelBody = qRotate(qc, s.baseAngVelWorld);
    const gravity = qRotate(qc, [0, 0, -1]);

    const obs = new Float64Array(257);
    let o = 0;
    obs[o++] = this.bootIndicator / this.bootMax; // boot (1)
    for (let i = 0; i < 6; i++) obs[o++] = s.command[i]; // command (6)
    for (let i = 0; i < 12; i++) obs[o++] = s.wrist6d[i]; // root_and_wrist_6d (12)
    for (let i = 0; i < 3; i++) obs[o++] = angVelBody[i]; // base_ang_vel (3)
    for (let i = 0; i < 3; i++) obs[o++] = gravity[i]; // projected_gravity (3)
    for (const k of this.histSteps) { // joint_pos_history (len*29)
      const h = this.jointHist[k];
      for (let i = 0; i < this.numDof; i++) obs[o++] = h[i];
    }
    for (let a = 0; a < this.prevActionSteps; a++) { // prev_actions (3*29)
      const p = this.prevActions[a];
      for (let i = 0; i < this.numDof; i++) obs[o++] = p[i];
    }
    return obs;
  }

  normalize(obs) {
    const out = new Float32Array(obs.length);
    for (let i = 0; i < obs.length; i++) out[i] = (obs[i] - this.loc[i]) / this.scale[i];
    return out;
  }

  /**
   * Turn a raw network output into an env-order PD target, updating EMA + the
   * prev-action buffer source. Mirrors get_action + get_pd_target.
   * @param {ArrayLike<number>} raw  network output (29, policy order)
   * @returns {Float64Array} pd_target in env / MuJoCo qpos order (29)
   */
  postprocess(raw) {
    const clamped = new Float64Array(this.numDof);
    for (let i = 0; i < this.numDof; i++) clamped[i] = clamp(raw[i], -10, 10);
    this.lastRawAction.set(clamped); // becomes next tick's prev_actions[0]

    const beta = this.actionBeta;
    const pdPolicy = new Float64Array(this.numDof);
    for (let i = 0; i < this.numDof; i++) {
      const scaled = clamped[i] * this.actionScales[i];
      this.applied[i] = this.applied[i] * (1 - beta) + scaled * beta; // EMA
      pdPolicy[i] = this.applied[i] + this.defaultPosPolicy[i];
    }
    const pdEnv = new Float64Array(this.numDof);
    for (let e = 0; e < this.numDof; e++) pdEnv[e] = pdPolicy[this.policyToEnv[e]];
    return pdEnv;
  }
}
