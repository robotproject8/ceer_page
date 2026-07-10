// teleop.js — keyboard teleoperation, ported from scripts/teleop_dummy_pub.py +
// the command/wrist6d construction in MyCustomPolicy (_get_udp_control +
// get_observation). Produces the policy's `command` (6) and `root_and_wrist_6d`
// (12) inputs from a keyboard-driven target pose.
//
// Conventions (match the deploy code exactly):
//   * root target quat is carried XYZW (converted to wxyz for math).
//   * hand target quats are treated WXYZ by axis_angle_from_quat (identity for
//     keyboard teleop, which only moves hand positions).
//   * command = [root_height=0.79, root_pos_w.x, root_pos_w.y,
//                heading_b.x, heading_b.y, force_limit=15]
//   * wrist6d = [lpos(3), rpos(3), axisangle(lquat)(3), axisangle(rquat)(3)]

const ROOT_HEIGHT = 0.79;
const FORCE_LIMIT = 15.0;

// ---- quaternion helpers (wxyz) --------------------------------------------
function yawFromQuatWXYZ(q) {
  const [w, x, y, z] = q;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}
function yawQuatWXYZ(q) {
  const h = yawFromQuatWXYZ(q) / 2;
  return [Math.cos(h), 0, 0, Math.sin(h)];
}
function quatApply(q, v) {
  const w = q[0], xyz = [q[1], q[2], q[3]];
  const t = cross(xyz, v).map((c) => c * 2);
  return [
    v[0] + w * t[0] + (xyz[1] * t[2] - xyz[2] * t[1]),
    v[1] + w * t[1] + (xyz[2] * t[0] - xyz[0] * t[2]),
    v[2] + w * t[2] + (xyz[0] * t[1] - xyz[1] * t[0]),
  ];
}
function quatApplyInverse(q, v) {
  const w = q[0], xyz = [q[1], q[2], q[3]];
  const t = cross(xyz, v).map((c) => c * 2);
  return [
    v[0] - w * t[0] + (xyz[1] * t[2] - xyz[2] * t[1]),
    v[1] - w * t[1] + (xyz[2] * t[0] - xyz[0] * t[2]),
    v[2] - w * t[2] + (xyz[0] * t[1] - xyz[1] * t[0]),
  ];
}
function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
function xyzwToWxyz(q) {
  return [q[3], q[0], q[1], q[2]];
}
// axis_angle_from_quat (wxyz), mirroring policy.axis_angle_from_quat.
function axisAngleFromQuatWXYZ(q) {
  const sign = q[0] < 0 ? -1 : 1;
  const qq = [q[0] * sign, q[1] * sign, q[2] * sign, q[3] * sign];
  const mag = Math.hypot(qq[1], qq[2], qq[3]);
  const halfAngle = Math.atan2(mag, qq[0]);
  const angle = 2 * halfAngle;
  const s = Math.abs(angle) > 1e-6 ? Math.sin(halfAngle) / angle : 0.5 - (angle * angle) / 48;
  return [qq[1] / s, qq[2] / s, qq[3] / s];
}

/**
 * Build the 6-dim command from the teleop root target + current robot pose.
 * @param {number[]} rootPosW      teleop root target position, world [x,y,z]
 * @param {number[]} rootQuatXYZW  teleop root target orientation (xyzw)
 * @param {number[]} baseQuatXYZW  current robot base orientation (xyzw)
 */
export function computeCommand(rootPosW, rootQuatXYZW, baseQuatXYZW) {
  const targetYawQuat = yawQuatWXYZ(xyzwToWxyz(rootQuatXYZW));
  const currentYawQuat = yawQuatWXYZ(xyzwToWxyz(baseQuatXYZW));
  const headingW = quatApply(targetYawQuat, [1, 0, 0]);
  const headingB = quatApplyInverse(currentYawQuat, headingW);
  return [ROOT_HEIGHT, rootPosW[0], rootPosW[1], headingB[0], headingB[1], FORCE_LIMIT];
}

/**
 * Build root_and_wrist_6d (12) from body-frame hand targets.
 * @param {number[]} lPosB, rPosB   hand positions, body frame [x,y,z]
 * @param {number[]} lQuatWXYZ, rQuatWXYZ  hand orientations (wxyz; identity for kbd)
 */
export function computeWrist6d(lPosB, rPosB, lQuatWXYZ = [1, 0, 0, 0], rQuatWXYZ = [1, 0, 0, 0]) {
  const la = axisAngleFromQuatWXYZ(normalizeQuat(lQuatWXYZ));
  const ra = axisAngleFromQuatWXYZ(normalizeQuat(rQuatWXYZ));
  return [lPosB[0], lPosB[1], lPosB[2], rPosB[0], rPosB[1], rPosB[2], la[0], la[1], la[2], ra[0], ra[1], ra[2]];
}
function normalizeQuat(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) + 1e-8;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

// ---------------------------------------------------------------------------
// Keyboard teleop state machine (mirrors scripts/teleop_dummy_pub.py).
// Root motion is velocity-based (WASD add to a persistent root speed); the root
// world target is integrated from that speed each control tick. QE rotate yaw.
// IJKL/UO move both hands in the body frame. SPACE stops; hands rest offsets.
// ---------------------------------------------------------------------------
const SPEED_STEP = 0.2; // m/s per press (W/S/A/D)
const YAW_STEP = (10 * Math.PI) / 180; // per press (Q/E)
const HAND_STEP = 0.02; // m per press (I/K/J/L/U/O)
const LH_REST = [0.25, 0.18, 0.15];
const RH_REST = [0.25, -0.18, 0.15];

export class KeyboardTeleop {
  constructor() {
    this.reset();
  }
  reset() {
    this.vx = 0; this.vy = 0; // body-ish root velocity command
    this.yaw = 0;
    this.rootX = 0; this.rootY = 0;
    this.lh = [...LH_REST];
    this.rh = [...RH_REST];
  }
  // Discrete key press (keydown), matching teleop_dummy_pub key bindings.
  onKey(key) {
    switch (key.toLowerCase()) {
      case 'w': this.vx += SPEED_STEP; break;
      case 's': this.vx -= SPEED_STEP; break;
      case 'a': this.vy += SPEED_STEP; break;
      case 'd': this.vy -= SPEED_STEP; break;
      case ' ': this.vx = 0; this.vy = 0; break;
      case 'q': this.yaw += YAW_STEP; break;
      case 'e': this.yaw -= YAW_STEP; break;
      case 'i': this.lh[0] += HAND_STEP; this.rh[0] += HAND_STEP; break;
      case 'k': this.lh[0] -= HAND_STEP; this.rh[0] -= HAND_STEP; break;
      case 'j': this.lh[1] += HAND_STEP; this.rh[1] -= HAND_STEP; break; // apart
      case 'l': this.lh[1] -= HAND_STEP; this.rh[1] += HAND_STEP; break; // together
      case 'u': this.lh[2] += HAND_STEP; this.rh[2] += HAND_STEP; break;
      case 'o': this.lh[2] -= HAND_STEP; this.rh[2] -= HAND_STEP; break;
      default: break;
    }
  }
  // Advance the integrated root world target by dt (velocity is in the yaw frame).
  integrate(dt) {
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    this.rootX += (c * this.vx - s * this.vy) * dt;
    this.rootY += (s * this.vx + c * this.vy) * dt;
  }
  rootQuatXYZW() {
    return [0, 0, Math.sin(this.yaw / 2), Math.cos(this.yaw / 2)];
  }
  // Produce { command, wrist6d } given the current robot base orientation.
  toPolicyInputs(baseQuatXYZW) {
    const command = computeCommand([this.rootX, this.rootY, ROOT_HEIGHT], this.rootQuatXYZW(), baseQuatXYZW);
    const wrist6d = computeWrist6d(this.lh, this.rh);
    return { command, wrist6d };
  }
}
