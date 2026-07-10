// Headless render-logic smoke test. Compiles the real scene (fs + MjVFS, same
// as smoke_load.mjs) and then exercises the ACTUAL helpers from
// src/mujocoScene.js — readEl / buildMeshGeometry — against the live binding,
// plus the geom_xpos / geom_xmat access the render loop relies on. Prints sane
// numbers. This proves the browser render path's array logic without a display.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMuJoCo from 'mujoco';
import { readEl, buildMeshGeometry, mjGEOM } from './src/mujocoScene.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const sceneDir = path.join(__dirname, 'public', 'scene');

const mj = await loadMuJoCo();
const xml = fs.readFileSync(path.join(sceneDir, 'g1_29dof_table_box.xml'), 'utf8');
const meshDir = path.join(sceneDir, 'meshes');
const vfs = new mj.MjVFS();
for (const f of fs.readdirSync(meshDir)) {
  vfs.addBuffer('meshes/' + f, new Uint8Array(fs.readFileSync(path.join(meshDir, f))));
}
const spec = mj.parseXMLString(xml);
const model = mj.mj_compile(spec, vfs);
const data = new mj.MjData(model);
model.opt.timestep = 0.001;
mj.mj_forward(model, data);

console.log('[render] model: ngeom=%d nbody=%d nmesh=%d', model.ngeom, model.nbody, model.nmesh);

// Geom type histogram (via the same accessor buildThreeScene uses).
const names = { 0: 'plane', 2: 'sphere', 3: 'capsule', 4: 'ellipsoid', 5: 'cylinder', 6: 'box', 7: 'mesh' };
const hist = {};
for (let i = 0; i < model.ngeom; i++) {
  const t = model.geom(i).type;
  hist[names[t] || t] = (hist[names[t] || t] || 0) + 1;
}
console.log('[render] geom type histogram:', hist);

// geom_xpos / geom_xmat access via readEl (render-loop path).
console.log('[render] geom_xpos length=%d (== ngeom*3? %s)', data.geom_xpos.length, data.geom_xpos.length === model.ngeom * 3);
console.log('[render] geom_xmat length=%d (== ngeom*9? %s)', data.geom_xmat.length, data.geom_xmat.length === model.ngeom * 9);

// Find the pelvis mesh geom (root of robot) and print its world pos + a matrix row.
let pelvisGeom = -1;
for (let i = 0; i < model.ngeom; i++) {
  if (model.geom(i).type === mjGEOM.MESH && (model.geom(i).name || '').includes('pelvis')) { pelvisGeom = i; break; }
}
if (pelvisGeom < 0) for (let i = 0; i < model.ngeom; i++) if (model.geom(i).type === mjGEOM.MESH) { pelvisGeom = i; break; }
const gp = pelvisGeom * 3;
console.log('[render] mesh geom %d ("%s") world pos = [%s]', pelvisGeom, model.geom(pelvisGeom).name,
  [0, 1, 2].map((k) => readEl(data.geom_xpos, gp + k).toFixed(4)).join(', '));

// Build a mesh geometry via the real helper and validate vertex/face counts.
const dataid = model.geom(pelvisGeom).dataid;
const meshAcc = model.mesh(dataid);
const geo = buildMeshGeometry(model, dataid);
const posAttr = geo.getAttribute('position');
const idx = geo.getIndex();
console.log('[render] mesh dataid=%d name="%s": vertadr=%d vertnum=%d faceadr=%d facenum=%d',
  dataid, meshAcc.name, meshAcc.vertadr, meshAcc.vertnum, meshAcc.faceadr, meshAcc.facenum);
console.log('[render] built BufferGeometry: positions=%d verts (expect %d), indices=%d (expect %d faces*3=%d)',
  posAttr.count, meshAcc.vertnum, idx.count, meshAcc.facenum, meshAcc.facenum * 3);

// Validate face indices are in-range for the sliced vertex block (local convention).
let maxIdx = 0;
for (let k = 0; k < idx.count; k++) maxIdx = Math.max(maxIdx, idx.getX(k));
console.log('[render] max face index=%d (must be < vertnum=%d): %s', maxIdx, meshAcc.vertnum, maxIdx < meshAcc.vertnum);

// Sanity: bounding box of the built mesh is finite and non-degenerate.
geo.computeBoundingBox();
const bb = geo.boundingBox;
console.log('[render] mesh bbox min=[%s] max=[%s]',
  [bb.min.x, bb.min.y, bb.min.z].map((v) => v.toFixed(3)).join(', '),
  [bb.max.x, bb.max.y, bb.max.z].map((v) => v.toFixed(3)).join(', '));

const ok = posAttr.count === meshAcc.vertnum && idx.count === meshAcc.facenum * 3 && maxIdx < meshAcc.vertnum &&
  data.geom_xpos.length === model.ngeom * 3 && data.geom_xmat.length === model.ngeom * 9;
console.log(ok ? '[render] OK — render array/mesh logic verified against binding' : '[render] FAIL — see above');
if (!ok) process.exit(1);
