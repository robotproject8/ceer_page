// mujocoScene.js — main-thread three.js scene construction + pose sync.
//
// ARCHITECTURE (post-worker refactor):
//   The MuJoCo simulation runs entirely inside src/simWorker.js (a Web Worker),
//   because the official pthread WASM build DEADLOCKS if stepped from the main
//   browser thread. This module is now PURE three.js: it consumes the plain
//   data / TypedArrays the worker posts (geom descriptors, mesh vertex/index
//   arrays, geom_xpos/geom_xmat) and never touches the MuJoCo binding.
//
//   Main-thread render pipeline:
//     worker 'ready' → buildThreeSceneFromData({ geoms, meshes }) → add group
//     worker 'frame' → syncFromArrays(xpos, xmat, handle)
//
//   The `loadScene` / `buildMeshGeometry` (live-model) helpers below are kept
//   ONLY for the headless node/browser test harnesses (smoke_render.mjs runs in
//   node where MuJoCo has unlimited threads; mjtest.html deliberately loads on
//   the main thread to DEMONSTRATE the deadlock this refactor fixes). Do NOT use
//   them from the app — the app path is the worker.
//
// Verified facts (see RENDERER_NOTES.md) for this mujoco@3.1.16 build:
//   * geom_xmat is a row-major 3x3 rotation, 9 floats per geom.
//   * mesh_face indices are LOCAL to each mesh; the worker already slices each
//     mesh's own vertex block, so indices arrive 0..vertnum-1.
//   * MuJoCo is +Z up, radians. The three scene is configured +Z up to match.

import * as THREE from 'three';

// MuJoCo geom type enum (mjtGeom) values we handle.
export const mjGEOM = {
  PLANE: 0,
  HFIELD: 1,
  SPHERE: 2,
  CAPSULE: 3,
  ELLIPSOID: 4,
  CYLINDER: 5,
  BOX: 6,
  MESH: 7,
};

const STL_KEY_PREFIX = 'meshes/';

/**
 * Read element `i` from a MuJoCo array field, tolerating either a plain
 * TypedArray (this build) or an emscripten `.get(i)` accessor.
 */
export function readEl(arr, i) {
  return typeof arr.get === 'function' ? arr.get(i) : arr[i];
}

/** Read `n` contiguous elements starting at `off` into a plain Array. */
export function readArray(arr, off, n) {
  if (typeof arr.subarray === 'function') return arr.subarray(off, off + n);
  const out = new Array(n);
  for (let i = 0; i < n; i++) out[i] = readEl(arr, off + i);
  return out;
}

// ---------------------------------------------------------------------------
// three.js scene construction from WORKER-POSTED plain data
// ---------------------------------------------------------------------------

/**
 * Build a THREE.BufferGeometry from a worker-posted mesh block
 * ({ positions:Float32Array, indices:Uint32Array }). Vertices are already
 * sliced to this mesh and indices are already local, so use them as-is.
 */
export function meshGeometryFromData(mesh) {
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(mesh.positions, 3));
  geom.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
  geom.computeVertexNormals();
  return geom;
}

/**
 * Map one geom descriptor to a THREE.BufferGeometry.
 * @param {{type:number,size:number[],dataid:number}} g  geom descriptor.
 * @param {(dataid:number)=>THREE.BufferGeometry|null} resolveMesh  mesh lookup.
 * @returns {{geometry:THREE.BufferGeometry, preRotZ:boolean}|null}
 */
function geometryForGeom(g, resolveMesh) {
  const size = g.size; // [3]
  let geometry = null;
  // three's built-in primitives are +Y-aligned; MuJoCo cylinder/capsule are
  // +Z-aligned, so those get pre-rotated about X below.
  let preRotZ = false;

  switch (g.type) {
    case mjGEOM.PLANE:
      // size may be 0 (infinite). Use a large finite plane in the geom's frame.
      geometry = new THREE.PlaneGeometry(40, 40, 1, 1);
      break;
    case mjGEOM.SPHERE:
      geometry = new THREE.SphereGeometry(size[0], 24, 16);
      break;
    case mjGEOM.ELLIPSOID:
      geometry = new THREE.SphereGeometry(1, 24, 16);
      geometry.scale(size[0], size[1], size[2]);
      break;
    case mjGEOM.CAPSULE:
      // MuJoCo: size[0]=radius, size[1]=half-length of the cylinder part.
      geometry = new THREE.CapsuleGeometry(size[0], 2 * size[1], 8, 16);
      preRotZ = true;
      break;
    case mjGEOM.CYLINDER:
      geometry = new THREE.CylinderGeometry(size[0], size[0], 2 * size[1], 24);
      preRotZ = true;
      break;
    case mjGEOM.BOX:
      geometry = new THREE.BoxGeometry(2 * size[0], 2 * size[1], 2 * size[2]);
      break;
    case mjGEOM.MESH:
      if (g.dataid >= 0) geometry = resolveMesh(g.dataid);
      break;
    default:
      // hfield / sdf / decor types not present in this scene — skip.
      break;
  }
  return geometry ? { geometry, preRotZ } : null;
}

/**
 * Build a THREE.Group holding one mesh per geom from the worker's 'ready'
 * payload. Transforms are applied from the payload's initial xpos/xmat.
 *
 * @param {object} payload  { geoms:[...], meshes:{dataid:{positions,indices}},
 *                            xpos:Float32Array, xmat:Float32Array }
 * @returns {{ group: THREE.Group, geomObjects: THREE.Object3D[] }}
 */
export function buildThreeSceneFromData(payload) {
  const { geoms, meshes, xpos, xmat } = payload;
  const group = new THREE.Group();
  group.name = 'mujoco-geoms';

  const meshCache = new Map(); // dataid -> THREE.BufferGeometry
  const resolveMesh = (dataid) => {
    if (!meshCache.has(dataid)) {
      const m = meshes[dataid];
      meshCache.set(dataid, m ? meshGeometryFromData(m) : null);
    }
    return meshCache.get(dataid);
  };

  const geomObjects = new Array(geoms.length).fill(null);

  for (let i = 0; i < geoms.length; i++) {
    const g = geoms[i];
    const built = geometryForGeom(g, resolveMesh);
    if (!built) continue;

    let geometry = built.geometry;
    if (built.preRotZ) {
      geometry = geometry.clone();
      geometry.rotateX(Math.PI / 2);
    }

    const rgba = g.rgba;
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      roughness: 0.7,
      metalness: 0.05,
      transparent: rgba[3] < 1,
      opacity: rgba[3],
      side: g.type === mjGEOM.PLANE ? THREE.DoubleSide : THREE.FrontSide,
    });

    const obj = new THREE.Mesh(geometry, material);
    obj.name = `geom_${i}_${g.name || ''}`;
    obj.matrixAutoUpdate = false; // we drive obj.matrix directly from MuJoCo
    obj.userData.geomId = i;
    geomObjects[i] = obj;
    group.add(obj);
  }

  const handle = { group, geomObjects };
  if (xpos && xmat) syncFromArrays(xpos, xmat, handle);
  return handle;
}

const _mat4 = new THREE.Matrix4();

/**
 * Update every geom object's transform from worker-posted geom_xpos (ngeom×3)
 * and geom_xmat (ngeom×9, row-major) TypedArrays.
 *
 * @param {ArrayLike<number>} xpos
 * @param {ArrayLike<number>} xmat
 * @param {{geomObjects:THREE.Object3D[]}} handle
 */
export function syncFromArrays(xpos, xmat, handle) {
  if (!handle) return;
  const objs = handle.geomObjects;
  for (let i = 0; i < objs.length; i++) {
    const obj = objs[i];
    if (!obj) continue;
    const p = i * 3;
    const r = i * 9;
    // MuJoCo xmat is row-major 3x3. THREE.Matrix4.set takes row-major args.
    _mat4.set(
      xmat[r + 0], xmat[r + 1], xmat[r + 2], xpos[p + 0],
      xmat[r + 3], xmat[r + 4], xmat[r + 5], xpos[p + 1],
      xmat[r + 6], xmat[r + 7], xmat[r + 8], xpos[p + 2],
      0, 0, 0, 1,
    );
    obj.matrix.copy(_mat4);
    obj.matrixWorldNeedsUpdate = true;
  }
}

// ---------------------------------------------------------------------------
// Live-model helpers — TEST HARNESS ONLY (node smoke_render.mjs / mjtest.html).
// The app does NOT call these; MuJoCo runs in simWorker.js. See the file header.
// ---------------------------------------------------------------------------

/**
 * Boot MuJoCo and compile the CEER scene from public/scene/ ON THE CALLING
 * THREAD. In node (smoke_render.mjs) this is fine. In a BROWSER MAIN THREAD it
 * DEADLOCKS (pthread pool exhaustion) — mjtest.html uses it to prove exactly
 * that. The app uses simWorker.js instead.
 *
 * @returns {Promise<{ mj, model, data, spec }>}
 */
export async function loadScene(opts = {}) {
  const baseUrl =
    opts.baseUrl ??
    (typeof import.meta !== 'undefined' && import.meta.env
      ? import.meta.env.BASE_URL
      : './');
  const report = opts.onProgress || (() => {});

  report('Loading MuJoCo runtime…');
  if (typeof crossOriginIsolated !== 'undefined' && !crossOriginIsolated) {
    console.warn(
      '[mujocoScene] crossOriginIsolated is false — SharedArrayBuffer/pthreads ' +
        'will fail. Dev needs COOP/COEP headers; prod needs coi-serviceworker.',
    );
  }
  const glueUrl = new URL(joinUrl(baseUrl, 'mujoco.js'), window.location.href).href;
  const wasmUrl = new URL(joinUrl(baseUrl, 'mujoco.wasm'), window.location.href).href;
  const loadMuJoCo = (await import(/* @vite-ignore */ glueUrl)).default;
  const mj = await loadMuJoCo({
    locateFile: (p) => (p.endsWith('.wasm') ? wasmUrl : p),
  });

  report('Fetching scene XML…');
  const sceneBase = joinUrl(baseUrl, 'scene/');
  const xml = await (await fetch(joinUrl(sceneBase, 'g1_29dof_table_box.xml'))).text();

  const meshFiles = parseMeshFileList(xml);
  report(`Fetching ${meshFiles.length} meshes…`);
  const vfs = new mj.MjVFS();
  await Promise.all(
    meshFiles.map(async (file) => {
      const buf = await (await fetch(joinUrl(sceneBase, 'meshes/' + file))).arrayBuffer();
      vfs.addBuffer(STL_KEY_PREFIX + file, new Uint8Array(buf));
    }),
  );

  report('Compiling model…');
  const spec = mj.parseXMLString(xml);
  const model = mj.mj_compile(spec, vfs);
  const data = new mj.MjData(model);

  model.opt.timestep = 0.001; // per SPEC
  mj.mj_forward(model, data); // populate geom_xpos / geom_xmat at initial pose

  return { mj, model, data, spec };
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

function joinUrl(base, path) {
  if (!base) return path;
  return base.endsWith('/') ? base + path : base + '/' + path;
}

/**
 * Build a THREE.BufferGeometry for MuJoCo mesh `dataid` from a LIVE model.
 * TEST HARNESS ONLY (smoke_render.mjs). The app path extracts meshes in the
 * worker and rebuilds them via meshGeometryFromData().
 */
export function buildMeshGeometry(model, dataid) {
  const mesh = model.mesh(dataid);
  const vertadr = mesh.vertadr;
  const vertnum = mesh.vertnum;
  const faceadr = mesh.faceadr;
  const facenum = mesh.facenum;

  const src = model.mesh_vert; // flat float32, 3 per vertex, global
  const positions = new Float32Array(vertnum * 3);
  const base = vertadr * 3;
  for (let k = 0; k < vertnum * 3; k++) positions[k] = readEl(src, base + k);

  const faces = model.mesh_face; // flat int32, 3 per triangle, LOCAL indices
  const indices = new Uint32Array(facenum * 3);
  const fbase = faceadr * 3;
  for (let k = 0; k < facenum * 3; k++) indices[k] = readEl(faces, fbase + k);

  return meshGeometryFromData({ positions, indices });
}
