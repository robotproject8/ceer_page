# CEER G1 — in-browser interactive demo

Real-time, in-browser playground for the CEER G1 loco-manipulation policy: the
MuJoCo physics + the trained policy run entirely client-side (no server), and you
drive the humanoid with the keyboard. Modeled on Gentle Humanoid's "play in your
browser" demo.

- **Main-page entry:** the button *"Play in your browser"* on the CEER project
  page links to `./play/`.
- **Stack:** MuJoCo WASM (official DeepMind `mujoco` npm, `mt` build) +
  three.js + a pure-JS policy forward pass, orchestrated in a Web Worker.

## Repo layout

- `play-src/` — **source** (this directory): the Vite project.
- `play/` — **build output** (committed): the static app GitHub Pages serves at
  `/<repo>/play/`. Produced by `npm run build` (Vite `outDir: ../play`).

## Develop / build

```bash
cd play-src
npm install
npm run dev      # http://localhost:5173/  (dev server sends COOP/COEP headers)
npm run build    # writes the static app to ../play/
```

The dev server sets the `Cross-Origin-Opener-Policy` / `Cross-Origin-Embedder-Policy`
headers the pthread MuJoCo WASM needs. In production (GitHub Pages, which can't set
headers) `public/coi-serviceworker.js` installs those headers via a service worker
and reloads once — no server config required.

## Controls

`W A S D` walk · `Q E` turn · `Space` stop · `I K` hands fwd/back ·
`J L` hands apart/together · `U O` hands up/down · `R` reset.

## Where the assets come from (regenerating)

The runtime assets in `public/` are exported from the `ceer_deploy` repo (the
policy + MuJoCo source of truth):

- `public/assets/policy_weights.bin`, `policy_arch.json` ← `scripts/export_policy_json.py`
- `public/assets/vecnorm.json`, `control_meta.json` ← `scripts/export_onnx.py` /
  `scripts/capture_golden.py` (control_meta is the runtime-resolved `meta` block
  from the golden capture — authoritative joint orders / gains / qpos indices)
- `public/scene/g1_29dof_table_box.xml` + `public/scene/meshes/*.STL` ← the MuJoCo
  scene (only the meshes referenced by the XML are shipped)
- `public/mt/mujoco.{js,wasm}` ← `node_modules/mujoco/mt/` (re-copy on upgrade)

## Correctness

The ported control math is validated **tick-by-tick against the Python deploy
pipeline** (golden fixtures from `ceer_deploy/scripts/capture_golden.py`):

```bash
node control_test.mjs   # obs + policy net + pd_target vs PyTorch, <1e-4
node teleop_test.mjs     # command + wrist6d construction, <1e-4
```

Headless-Chrome end-to-end checks (need the dev/preview server running):

```bash
node worker_test.mjs   # MuJoCo loads in the worker, no pthread deadlock
node policy_test.mjs    # robot balances/stands under policy control
node drive_test.mjs     # W drives it forward while balancing
```

(Test fixtures under `test/` are gitignored — regenerate them with the
`ceer_deploy` scripts.)

## Why these design choices

- **`mt` build + Web Worker:** the top-level `mujoco.js` has a lazy size-0 pthread
  pool and deadlocks in `mj_compile` (even off the main thread). The `mt` build
  pre-creates the pool at startup; running it in a worker keeps the UI thread free.
- **Pure-JS policy (no onnxruntime-web):** the net is a small fixed feed-forward
  (Linear + LayerNorm + Mish, three branches); a hand-written forward pass
  (`src/policyNet.js`) drops a 26 MB wasm runtime and matches ONNX to <1e-4.
