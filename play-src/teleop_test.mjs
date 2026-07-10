// Validate src/teleop.js command/wrist6d construction against golden fixtures:
// feed each tick's recorded teleop sample + robot base_quat and check the JS
// produces the same info_command / info_wrist6d as the Python policy.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { computeCommand, computeWrist6d } from './src/teleop.js';

const dir = path.dirname(fileURLToPath(import.meta.url));
const golden = JSON.parse(fs.readFileSync(path.join(dir, 'test/golden.json'), 'utf8'));
const TOL = 1e-4;
const maxAbs = (a, b) => { let m = 0; for (let i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i] - b[i])); return m; };

let worstCmd = 0, wc = -1, worstW = 0, ww = -1;
for (const t of golden.ticks) {
  if (t.teleop_seq < 0) continue; // no packet yet
  const cmd = computeCommand(t.teleop_root_pos_w, t.teleop_root_quat_xyzw, t.base_quat_xyzw);
  // hand quats are carried WXYZ (consumed by axis_angle_from_quat); pass as-is.
  const w6 = computeWrist6d(t.teleop_l_pos_b, t.teleop_r_pos_b, t.teleop_l_quat_wxyz, t.teleop_r_quat_wxyz);
  const ec = maxAbs(cmd, t.info_command);
  const ew = maxAbs(w6, t.info_wrist6d);
  if (ec > worstCmd) { worstCmd = ec; wc = t.tick; }
  if (ew > worstW) { worstW = ew; ww = t.tick; }
}
console.log(`command  worst |JS-py| = ${worstCmd.toExponential(2)} @tick ${wc}`);
console.log(`wrist6d  worst |JS-py| = ${worstW.toExponential(2)} @tick ${ww}`);
const pass = worstCmd < TOL && worstW < TOL;
console.log(pass ? `PASS (< ${TOL})` : `FAIL (>= ${TOL})`);
process.exit(pass ? 0 : 1);
