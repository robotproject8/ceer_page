// Golden replay test (node): validate src/control.js against the Python deploy
// pipeline. Replays scripts/capture_golden.py fixtures (test/golden.json) tick by
// tick — feeding each tick's recorded MuJoCo state + teleop command into the JS
// obs-builder, running the real ONNX policy, and post-processing — then asserts
// the resulting observation and pd_target match Python to < 1e-4.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Controller } from './src/control.js';
import { PolicyNet } from './src/policyNet.js';

const dir = path.dirname(fileURLToPath(import.meta.url));
const golden = JSON.parse(fs.readFileSync(path.join(dir, 'test/golden.json'), 'utf8'));
const vecnorm = JSON.parse(fs.readFileSync(path.join(dir, 'test/vecnorm.json'), 'utf8'));
const { meta, ticks } = golden;

const TOL = 1e-4;
const maxAbs = (a, b) => {
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i] - b[i]));
  return m;
};

// Pure-JS policy net (validates policyNet.js against the golden pd_target too).
const weightsBuf = fs.readFileSync(path.join(dir, 'test/policy_weights.bin'));
const ab = weightsBuf.buffer.slice(weightsBuf.byteOffset, weightsBuf.byteOffset + weightsBuf.byteLength);
const arch = JSON.parse(fs.readFileSync(path.join(dir, 'test/policy_arch.json'), 'utf8'));
const net = new PolicyNet(ab, arch);

const ctrl = new Controller(meta, vecnorm.loc, vecnorm.scale);
let worstObs = 0, worstObsTick = -1;
let worstPd = 0, worstPdTick = -1;

for (const t of ticks) {
  const obs = ctrl.buildObs({
    command: t.info_command,
    wrist6d: t.info_wrist6d,
    baseQuatXYZW: t.base_quat_xyzw,
    baseAngVelWorld: t.base_ang_vel_world,
    dofPosEnv: t.dof_pos_env,
  });
  const eObs = maxAbs(obs, t.obs_raw);
  if (eObs > worstObs) { worstObs = eObs; worstObsTick = t.tick; }

  const norm = ctrl.normalize(obs);
  const raw = net.forward(norm);
  const pdEnv = ctrl.postprocess(raw);

  const ePd = maxAbs(pdEnv, t.pd_target_env);
  if (ePd > worstPd) { worstPd = ePd; worstPdTick = t.tick; }
}

console.log(`obs      worst |JS-py| = ${worstObs.toExponential(2)} @tick ${worstObsTick}`);
console.log(`pd_target worst |JS-py| = ${worstPd.toExponential(2)} @tick ${worstPdTick}`);
const pass = worstObs < TOL && worstPd < TOL;
console.log(pass ? `PASS (< ${TOL}) over ${ticks.length} ticks` : `FAIL (>= ${TOL})`);
process.exit(pass ? 0 : 1);
