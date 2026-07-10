// simWorker.js — the MuJoCo simulation, isolated in a dedicated Web Worker.
//
// WHY A WORKER + THE `mt` BUILD (the deadlock this fixes):
//   The official DeepMind `mujoco` WASM is a pthread build. mj_compile
//   parallelizes mesh processing across ~hardwareConcurrency/2 pthreads, each a
//   Web Worker. A module Worker's script only loads once the CREATING thread
//   yields to the event loop — but mj_compile is a single blocking WASM call, so
//   the creating thread never yields and the pthread Workers never finish
//   loading: "thread pool is exhausted" → hang. This happens on the main browser
//   thread AND inside a plain worker if the pool is created lazily.
//   Two independent things make it work here:
//     1. All MuJoCo runs in THIS worker (never the main thread), so the browser
//        main thread stays free and the app UI never blocks.
//     2. We load the `mt` (multi-threaded) emscripten build, whose
//        initMainThread() PRE-CREATES the whole pthread pool during startup
//        (while the event loop is free) and whose factory promise only resolves
//        once every pool worker has loaded the wasm. So by the time we call
//        mj_compile, ready workers are already waiting — no Worker is created
//        mid-block. (See loadMuJoCo below.)
//   The main thread only ever sees plain data / TypedArrays over postMessage.
//
// MESSAGE PROTOCOL (see also main.js):
//   main → worker  { type: 'init' }
//   worker → main  { type: 'status', message }           (load progress)
//   worker → main  { type: 'ready', model, geoms, meshes, xpos, xmat }
//   worker → main  { type: 'error', message }
//   main → worker  { type: 'setStepping', on: bool }
//   worker → main  { type: 'frame', xpos: Float32Array, xmat: Float32Array }
//
// PHASE-2 SEAM: control (setCtrl) + onnx go HERE, not on main. See stepOnce()
//   and the message switch below for the exact insertion points.

// Trivial array reader — tolerates a plain TypedArray (this build) or a future
// emscripten `.get(i)` accessor. Duplicated from mujocoScene.readEl so this
// worker stays free of any three.js / DOM import.
import { Controller } from './control.js';
import { KeyboardTeleop } from './teleop.js';
import { PolicyNet } from './policyNet.js';

const readEl = (arr, i) => (typeof arr.get === 'function' ? arr.get(i) : arr[i]);

const MESH_GEOM_TYPE = 7; // mjtGeom.MESH
const STL_KEY_PREFIX = 'meshes/';
const SUBSTEPS_PER_TICK = 20; // SPEC sim_decimation
const TICK_MS = 20; // ~50 Hz

let mj = null;
let model = null;
let data = null;
let ngeom = 0;
let stepTimer = null;

// --- control (policy) state ------------------------------------------------
let ctrl = null; // Controller (obs build + postprocess)
let teleop = null; // KeyboardTeleop
let net = null; // PolicyNet (pure-JS forward)
let meta = null; // control_meta.json (gains, joint orders, qpos indices)
let mode = 'idle'; // 'idle' | 'policy'
// PD gains + MuJoCo index maps (env joint order), filled on init.
let Kp = null, Kd = null, tlim = null, qposIdx = null, qvelIdx = null, nCtrl = 0;
const CONTROL_DT = TICK_MS / 1000;

async function loadControlAssets() {
  const base = new URL('assets/', BASE).href;
  const [metaJson, vecnorm, arch, wbuf] = await Promise.all([
    fetch(base + 'control_meta.json').then((r) => r.json()),
    fetch(base + 'vecnorm.json').then((r) => r.json()),
    fetch(base + 'policy_arch.json').then((r) => r.json()),
    fetch(base + 'policy_weights.bin').then((r) => r.arrayBuffer()),
  ]);
  meta = metaJson;
  ctrl = new Controller(meta, vecnorm.loc, vecnorm.scale);
  teleop = new KeyboardTeleop();
  net = new PolicyNet(wbuf, arch);

  Kp = Float64Array.from(meta.env_stiffness);
  Kd = Float64Array.from(meta.env_damping);
  tlim = Float64Array.from(meta.env_torque_limits);
  qposIdx = Int32Array.from(meta.mj_qpos_indices);
  qvelIdx = Int32Array.from(meta.mj_qvel_indices);
  nCtrl = qposIdx.length; // 29
}

function post(msg, transfer) {
  self.postMessage(msg, transfer || []);
}
const status = (message) => post({ type: 'status', message });

// --- MuJoCo boot + compile -------------------------------------------------

// Cap the pthread pool. The `mt` build pre-allocates navigator.hardwareConcurrency
// workers at init; on a 32-core box that is many Workers each mapping the 8.6 MB
// wasm. mj_compile only parallelizes to ~hardwareConcurrency/2, so a modest cap
// keeps resource use sane while leaving ample headroom (pool >= threads needed).
const MAX_POOL = 8;

// App base URL (absolute), passed from main.js init. Assets (mt/, assets/,
// scene/) resolve against it so the app works under a GitHub Pages subpath too.
let BASE = self.location.origin + '/';

async function loadMuJoCo() {
  // WHY THE `mt` BUILD: the official `mujoco` package ships two emscripten builds.
  // The top-level one has a LAZY (size-0) pthread pool: it creates a pthread
  // Worker on demand, and a module Worker's script fetch needs the CREATING
  // thread to yield — which never happens inside a blocking mj_compile, so it
  // deadlocks even off the main thread. The `mt` build's initMainThread()
  // PRE-CREATES the pool during startup (event loop free) and the factory promise
  // only resolves once every pool worker has loaded the wasm. So by the time we
  // call mj_compile, ready workers wait in the pool and NO Worker is created
  // mid-block → no deadlock.
  //
  // The glue + wasm are served as SIBLINGS from public/mt/ (so the glue's
  // `new URL("mujoco.js", import.meta.url)` pthread-worker resolution stays
  // correct). We load the glue from a RUNTIME-COMPUTED absolute URL so Vite never
  // statically parses it (its `new Worker(url, {...})` breaks Vite's parser) and
  // so its wasm + pthread siblings resolve against a real http origin.
  const glueUrl = new URL('mt/mujoco.js', BASE).href;
  const wasmUrl = new URL('mt/mujoco.wasm', BASE).href;

  // initMainThread() reads navigator.hardwareConcurrency to size the pre-created
  // pool, so cap it BEFORE loading the glue. (Thread count affects only speed,
  // not simulation results.)
  try {
    const real = self.navigator.hardwareConcurrency || MAX_POOL;
    Object.defineProperty(self.navigator, 'hardwareConcurrency', {
      value: Math.min(real, MAX_POOL),
      configurable: true,
    });
  } catch (e) {
    console.warn('[simWorker] could not cap hardwareConcurrency', e);
  }

  const factory = (await import(/* @vite-ignore */ glueUrl)).default;
  return factory({ locateFile: (p) => (p.endsWith('.wasm') ? wasmUrl : p) });
}

/** Extract the STL filenames referenced by <mesh file="..."/> in the XML. */
function parseMeshFileList(xml) {
  const files = [];
  const re = /<mesh\b[^>]*\bfile\s*=\s*"([^"]+)"/g;
  let m;
  while ((m = re.exec(xml)) !== null) {
    const f = m[1].split('/').pop();
    if (f) files.push(f);
  }
  return [...new Set(files)];
}

async function init() {
  if (typeof crossOriginIsolated !== 'undefined' && !crossOriginIsolated) {
    console.warn(
      '[simWorker] crossOriginIsolated is false — SharedArrayBuffer/pthreads ' +
        'will fail. Dev needs COOP/COEP headers (vite.config.js).',
    );
  }

  status('Loading MuJoCo runtime…');
  mj = await loadMuJoCo();

  status('Fetching scene XML…');
  const sceneBase = new URL('scene/', BASE).href;
  const xml = await (await fetch(sceneBase + 'g1_29dof_table_box.xml')).text();

  const meshFiles = parseMeshFileList(xml);
  status(`Fetching ${meshFiles.length} meshes…`);
  const vfs = new mj.MjVFS();
  await Promise.all(
    meshFiles.map(async (file) => {
      const buf = await (await fetch(sceneBase + 'meshes/' + file)).arrayBuffer();
      vfs.addBuffer(STL_KEY_PREFIX + file, new Uint8Array(buf));
    }),
  );

  status('Compiling model…');
  const spec = mj.parseXMLString(xml);
  model = mj.mj_compile(spec, vfs);
  data = new mj.MjData(model);
  model.opt.timestep = 0.001; // per SPEC
  mj.mj_forward(model, data); // populate geom_xpos / geom_xmat at initial pose

  ngeom = model.ngeom;

  status('Loading policy…');
  await loadControlAssets();

  const payload = extractRenderModel();
  post({ type: 'ready', ...payload.msg }, payload.transfer);
}

// --- policy control tick ---------------------------------------------------
// Reproduces the deploy loop (verified in control_test.mjs): read MuJoCo state →
// teleop command → obs → normalize → policy net → pd_target (env order) → PD
// torque held across the substeps. All joint math is in env / MuJoCo qpos order.
const _dofPosEnv = () => {
  const q = data.qpos;
  const out = new Float64Array(nCtrl);
  for (let i = 0; i < nCtrl; i++) out[i] = readEl(q, qposIdx[i]);
  return out;
};

let _pdTarget = null; // Float64Array(29), env order — held across substeps

function computePolicy() {
  const q = data.qpos;
  const qd = data.qvel;
  const baseQuatXYZW = [readEl(q, 4), readEl(q, 5), readEl(q, 6), readEl(q, 3)]; // wxyz→xyzw
  const baseAngVelWorld = [readEl(qd, 3), readEl(qd, 4), readEl(qd, 5)];

  teleop.integrate(CONTROL_DT);
  const { command, wrist6d } = teleop.toPolicyInputs(baseQuatXYZW);

  const obs = ctrl.buildObs({
    command,
    wrist6d,
    baseQuatXYZW,
    baseAngVelWorld,
    dofPosEnv: _dofPosEnv(),
  });
  const raw = net.forward(ctrl.normalize(obs));
  _pdTarget = ctrl.postprocess(raw); // env order (29)
}

function applyPdTorque() {
  const q = data.qpos;
  const qd = data.qvel;
  const ctrlArr = data.ctrl;
  for (let i = 0; i < nCtrl; i++) {
    const dp = readEl(q, qposIdx[i]);
    const dv = readEl(qd, qvelIdx[i]);
    let tau = (_pdTarget[i] - dp) * Kp[i] - dv * Kd[i];
    const lim = tlim[i];
    if (tau > lim) tau = lim;
    else if (tau < -lim) tau = -lim;
    // actuator i drives env-order joint i (matches the deploy env.step, which
    // assigns the whole ctrl array from env-order torques).
    if (typeof ctrlArr.set === 'function' && ctrlArr.length === undefined) ctrlArr.set(i, tau);
    else ctrlArr[i] = tau;
  }
}

// --- one-time render-model extraction --------------------------------------
// Turns the live MuJoCo model into plain data the main thread can build three.js
// geometry from, so the main thread never touches the WASM binding.

function extractMesh(dataid) {
  const mesh = model.mesh(dataid);
  const vertadr = mesh.vertadr;
  const vertnum = mesh.vertnum;
  const faceadr = mesh.faceadr;
  const facenum = mesh.facenum;

  const src = model.mesh_vert; // flat float, 3 per vertex, global
  const positions = new Float32Array(vertnum * 3);
  const vbase = vertadr * 3;
  for (let k = 0; k < vertnum * 3; k++) positions[k] = readEl(src, vbase + k);

  const faces = model.mesh_face; // flat int, 3 per triangle, LOCAL indices
  const indices = new Uint32Array(facenum * 3);
  const fbase = faceadr * 3;
  for (let k = 0; k < facenum * 3; k++) indices[k] = readEl(faces, fbase + k);

  return { positions, indices };
}

function extractRenderModel() {
  const geoms = new Array(ngeom);
  const meshes = {}; // dataid -> { positions, indices }
  const transfer = [];

  // body_mocapid[bodyid] >= 0 marks a mocap body — in this scene those hold the
  // deploy-only debug viz markers (coordinate-frame axes, target sphere/arrow).
  const bodyMocapId = model.body_mocapid;
  for (let i = 0; i < ngeom; i++) {
    const g = model.geom(i);
    const dataid = g.dataid;
    const mocap = bodyMocapId ? readEl(bodyMocapId, g.bodyid) >= 0 : false;
    geoms[i] = {
      type: g.type,
      size: [readEl(g.size, 0), readEl(g.size, 1), readEl(g.size, 2)],
      rgba: [readEl(g.rgba, 0), readEl(g.rgba, 1), readEl(g.rgba, 2), readEl(g.rgba, 3)],
      dataid,
      bodyid: g.bodyid,
      name: g.name || '',
      mocap,
    };
    if (g.type === MESH_GEOM_TYPE && dataid >= 0 && !(dataid in meshes)) {
      const m = extractMesh(dataid);
      meshes[dataid] = m;
      transfer.push(m.positions.buffer, m.indices.buffer);
    }
  }

  const { xpos, xmat } = snapshotPose();
  transfer.push(xpos.buffer, xmat.buffer);

  return {
    msg: {
      model: {
        ngeom,
        nbody: model.nbody,
        nmesh: model.nmesh,
        nq: model.nq,
      },
      geoms,
      meshes,
      xpos,
      xmat,
    },
    transfer,
  };
}

// --- per-frame pose snapshot (Float32 copies for transfer) -----------------

function snapshotPose() {
  const srcPos = data.geom_xpos; // Float64Array, ngeom*3
  const srcMat = data.geom_xmat; // Float64Array, ngeom*9, row-major
  const xpos = new Float32Array(ngeom * 3);
  const xmat = new Float32Array(ngeom * 9);
  for (let k = 0; k < ngeom * 3; k++) xpos[k] = readEl(srcPos, k);
  for (let k = 0; k < ngeom * 9; k++) xmat[k] = readEl(srcMat, k);
  return { xpos, xmat };
}

// --- stepping loop ---------------------------------------------------------

function stepOnce() {
  if (mode === 'policy' && ctrl) {
    computePolicy(); // once per control tick; pd_target held across substeps
    for (let s = 0; s < SUBSTEPS_PER_TICK; s++) {
      applyPdTorque(); // torque recomputed each substep from current qpos/qvel
      mj.mj_step(model, data);
    }
  } else {
    // idle: passive physics preview (robot sags without a policy)
    for (let s = 0; s < SUBSTEPS_PER_TICK; s++) mj.mj_step(model, data);
  }
}

function tick() {
  stepOnce();
  const { xpos, xmat } = snapshotPose();
  const q = data.qpos;
  post(
    { type: 'frame', xpos, xmat, rootX: readEl(q, 0), rootY: readEl(q, 1), rootZ: readEl(q, 2) },
    [xpos.buffer, xmat.buffer],
  );
}

function setStepping(on) {
  if (on) {
    if (stepTimer == null) stepTimer = self.setInterval(tick, TICK_MS);
  } else if (stepTimer != null) {
    self.clearInterval(stepTimer);
    stepTimer = null;
  }
}

function resetSim() {
  mj.mj_resetData(model, data);
  mj.mj_forward(model, data);
  if (ctrl) ctrl.reset();
  if (teleop) teleop.reset();
  _pdTarget = null;
}

// --- message pump ----------------------------------------------------------

self.onmessage = async (ev) => {
  const msg = ev.data || {};
  switch (msg.type) {
    case 'init':
      try {
        if (msg.baseUrl) BASE = msg.baseUrl;
        await init();
      } catch (err) {
        console.error('[simWorker] init failed', err);
        post({ type: 'error', message: (err && err.message) || String(err) });
      }
      break;
    case 'setStepping':
      setStepping(!!msg.on);
      break;
    case 'setMode':
      // 'policy' → run the onnx control loop; 'idle' → passive physics.
      mode = msg.mode === 'policy' ? 'policy' : 'idle';
      if (mode === 'policy') setStepping(true);
      break;
    case 'key':
      if (teleop && typeof msg.key === 'string') teleop.onKey(msg.key);
      break;
    case 'reset':
      resetSim();
      break;
    default:
      console.warn('[simWorker] unknown message', msg.type);
  }
};
