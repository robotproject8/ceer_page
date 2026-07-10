# Renderer notes (Phase 1)

In-browser MuJoCo scene viewer: loads `public/scene/g1_29dof_table_box.xml`
(+ 38 STL meshes) with the official `mujoco` WASM binding and renders it with
three.js. Camera orbit only — **no policy/control logic** (that is Phase 2).

## How to run

```bash
npm run dev       # dev server at http://localhost:5173/
npm run build     # production build → dist/  (verified: succeeds)
npm run preview   # serve the production build
node smoke_render.mjs   # headless verification of the render array/mesh logic
```

Open the dev URL in a browser: you should see the G1 robot standing at a table
with several objects on/around it, on a ground grid. Drag to orbit, scroll to
zoom, right-drag to pan. A checkbox toggles policy-less physics stepping (the
robot will sag/collapse — expected without a controller).

## WASM serving (the Vite gotcha)

The emscripten glue `mujoco.js` contains a pthread code path:
`new Worker(new URL("mujoco.js", import.meta.url), { ..., "name": "em-pthread-" + id })`.
The worker options object is non-static, and Vite's **build-time** worker parser
rejects it (`Vite is unable to parse the worker options`) even though this
single-threaded build never executes that path.

**Fix (robust, no node_modules edits):** load the glue and its wasm from
`public/` at runtime so Vite never bundles or parses `mujoco.js`:

1. Copied `node_modules/mujoco/mujoco.js` → `public/mujoco.js` and
   `node_modules/mujoco/mujoco.wasm` → `public/mujoco.wasm` (served at
   `<base>mujoco.js` / `<base>mujoco.wasm`).
2. In `src/mujocoScene.js`, load the glue via a `/* @vite-ignore */` dynamic
   import of that public URL, and pass `locateFile` to point the wasm fetch at
   the public copy:
   ```js
   const glueUrl = import.meta.env.BASE_URL + 'mujoco.js';
   const loadMuJoCo = (await import(/* @vite-ignore */ glueUrl)).default;
   const mj = await loadMuJoCo({
     locateFile: (p) => p.endsWith('.wasm') ? import.meta.env.BASE_URL + 'mujoco.wasm' : p,
   });
   ```
3. `vite.config.js` sets `base: './'` (deployed under a subpath) and
   `optimizeDeps.exclude: ['mujoco']`.

Verified: `npm run build` succeeds; dev server returns 200 for `/mujoco.js`
(294661 B) and `/mujoco.wasm` (8607662 B); and the public glue copy boots the
wasm through this exact `locateFile` path.

> Note: `public/mujoco.js` / `public/mujoco.wasm` are copies of the installed
> package files. If `mujoco` is upgraded, re-copy them
> (`cp node_modules/mujoco/mujoco.{js,wasm} public/`).

## MuJoCo array access (empirically confirmed, mujoco@3.1.16)

For this build the MjModel/MjData array fields are **plain JS TypedArrays**, so
direct indexing works:

- `data.geom_xpos` — `Float64Array`, length `ngeom*3` (378).
- `data.geom_xmat` — `Float64Array`, length `ngeom*9` (1134), **row-major 3x3**.
- `model.mesh_vert` — `Float32Array`, 3 per vertex (global, offset by `vertadr`).
- `model.mesh_face` — `Int32Array`, 3 per triangle; indices are **LOCAL** to
  each mesh (range `0..vertnum-1`), confirmed empirically.
- Accessor scalar fields (`geom.type`, `geom.dataid`, `mesh.vertadr`, …) are
  plain `number`s; vector fields (`geom.size`, `geom.rgba`) are TypedArrays.

`src/mujocoScene.js` still routes all reads through a defensive `readEl(arr, i)`
helper (`arr.get(i)` if that's a function, else `arr[i]`) so the code also works
if a future build switches to the emscripten `.get()` accessor style.

Mesh extraction (`buildMeshGeometry`): slice this mesh's own vertex block out of
`mesh_vert` (`[vertadr*3, (vertadr+vertnum)*3)`) and use the local face indices
as-is; then `computeVertexNormals()`.

## Up-axis handling

MuJoCo world is **+Z up**, radians. three.js defaults to +Y up. Handled by:
- `camera.up.set(0, 0, 1)` so OrbitControls orbits about the Z axis and the
  robot stands upright.
- The `GridHelper` (native XZ plane) is rotated `+π/2` about X to lie in XY.
- Each geom's world transform comes straight from `geom_xpos` + `geom_xmat`
  (already +Z-up), written into `obj.matrix` with `matrixAutoUpdate = false`.

## Geom types built / approximations

Types present in this scene (histogram from `smoke_render.mjs`, total 126):
`plane 1, mesh 60, sphere 13, cylinder 14, box 37, ellipsoid 1`. Mapping:

| MuJoCo geom | three geometry | notes |
|---|---|---|
| plane (0)     | `PlaneGeometry(40,40)` | size is 0/infinite → large finite plane, `DoubleSide` |
| sphere (2)    | `SphereGeometry(size[0])` | |
| capsule (3)   | `CapsuleGeometry(size[0], 2*size[1])` | none in this scene; rotated +Y→+Z |
| ellipsoid (4) | `SphereGeometry(1)` scaled by `size` | |
| cylinder (5)  | `CylinderGeometry(size[0], size[0], 2*size[1])` | rotated +Y→+Z |
| box (6)       | `BoxGeometry(2*size)` | |
| mesh (7)      | `BufferGeometry` from `mesh_vert`/`mesh_face` | per-mesh offsets |

three's cylinder/capsule are +Y-aligned; MuJoCo's are +Z-aligned, so those two
geometries are pre-rotated `+π/2` about X. Colors come from `geom.rgba`
(`transparent` when alpha < 1); materials (`matid`) are not read in Phase 1 —
rgba fallback is sufficient. Unhandled decorative/hfield/sdf types are skipped
(none present). The scene contains both visual (group 1) and collision geoms;
all are rendered — collision geoms mostly coincide with the visual meshes.

## API for Phase 2 (`src/mujocoScene.js` exports)

```js
loadScene({ baseUrl?, onProgress? })  // → { mj, model, data, spec }; sets
                                      //   opt.timestep=0.001, runs mj_forward.
buildThreeScene(model, data)          // → { group, geomObjects };
                                      //   add `group` to your THREE.Scene.
syncTransforms(model, data, handle)   // write geom_xpos/xmat → object matrices.
buildMeshGeometry(model, dataid)      // → THREE.BufferGeometry for a mesh.
readEl(arr, i) / readArray(arr, off, n)   // array-access helpers.
mjGEOM                                // geom-type enum constants.
```

Phase-2 control loop shape: set `data.ctrl` → `mj.mj_step(model, data)` ×20 →
`syncTransforms(model, data, handle)` once per rendered frame. `main.js` already
demonstrates the stepping structure behind the "Step physics" checkbox.
