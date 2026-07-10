// main.js — Phase 1 viewer entry point.
//
// The MuJoCo simulation runs entirely in a Web Worker (src/simWorker.js): the
// official pthread WASM build deadlocks if stepped on the main browser thread.
// This module owns ONLY the three.js render side. It spawns the worker, builds
// the scene from the worker's 'ready' payload, and applies per-'frame' poses.
// NO policy/control logic here — that is Phase 2 and lives in the worker.

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Reflector } from 'three/examples/jsm/objects/Reflector.js';
import { buildThreeSceneFromData, syncFromArrays } from './mujocoScene.js';

const appEl = document.getElementById('app');
const statusEl = document.getElementById('status');
const overlayEl = document.getElementById('overlay');
const stepToggle = document.getElementById('step-toggle');
const resetBtn = document.getElementById('reset-btn');
const hudZ = document.getElementById('hud-z');
const hudFps = document.getElementById('hud-fps');
let _fpsCount = 0, _fpsLast = 0, _fpsTimer = 0;

// --- three.js core ---------------------------------------------------------
const scene = new THREE.Scene();
// Deep-blue vertical gradient backdrop (Gentle-Humanoid style).
scene.background = makeGradientBackground();

// Cinematic 3/4 view of the robot + the table/box it works with, +Z up. Kept a
// bit elevated so the front table doesn't occlude the robot (our scene, unlike
// Gentle Humanoid's empty floor, has furniture).
const camera = new THREE.PerspectiveCamera(45, aspect(), 0.02, 1000);
camera.up.set(0, 0, 1); // MuJoCo / robot world is +Z up
camera.position.set(2.8, -3.2, 1.8);

// WebGL may be unavailable (e.g. headless test browsers with no GPU context).
// Wrap creation so a failure only disables rendering — the worker/sim path and
// the window.__ceer probe still run, keeping the app testable and robust.
let renderer = null;
let controls = null;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = false;
  appEl.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(0, 0, 0.72); // orbit around the robot's mid-body
  controls.minDistance = 1.6;
  controls.maxDistance = 12;
  // Keep the camera above the ground (never orbit under the floor).
  controls.maxPolarAngle = Math.PI * 0.5 - 0.04;
  controls.update();
} catch (err) {
  console.warn('[viewer] WebGL unavailable — rendering disabled, sim still runs:', err);
  renderer = null;
  controls = null;
}

// --- lighting --------------------------------------------------------------
scene.add(new THREE.HemisphereLight(0xffffff, 0x3a3f4a, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 1.6);
key.position.set(3, -2, 5);
scene.add(key);
const fill = new THREE.DirectionalLight(0xffffff, 0.5);
fill.position.set(-3, 3, 2);
scene.add(fill);

// Reflective blue floor (Gentle-Humanoid style): a mirror that reflects the
// robot, tinted/dimmed to a dark steel-blue. Replaces both the grid and the
// MuJoCo floor plane (mujocoScene.js skips the plane geom). PlaneGeometry lies in
// XY with +Z normal — exactly our ground plane, so no rotation needed.
const reflector = new Reflector(new THREE.PlaneGeometry(120, 120), {
  color: 0x30414f, // dims + tints the reflection toward a subtle steel blue
  textureWidth: Math.min(2048, window.innerWidth * window.devicePixelRatio),
  textureHeight: Math.min(2048, window.innerHeight * window.devicePixelRatio),
  clipBias: 0.003,
});
reflector.position.z = 0.0;
scene.add(reflector);

// --- sim worker ------------------------------------------------------------
let sceneHandle = null; // { group, geomObjects }

const setStatus = (msg, isError = false) => {
  statusEl.textContent = msg;
  statusEl.classList.toggle('error', isError);
};

// Vite bundles the module worker referenced by this new URL(...) form.
const worker = new Worker(new URL('./simWorker.js', import.meta.url), {
  type: 'module',
});

worker.onmessage = (ev) => {
  const msg = ev.data || {};
  switch (msg.type) {
    case 'status':
      setStatus(msg.message);
      break;
    case 'ready':
      onReady(msg);
      break;
    case 'frame':
      if (sceneHandle) syncFromArrays(msg.xpos, msg.xmat, sceneHandle);
      if (window.__ceer) {
        window.__ceer.frames = (window.__ceer.frames || 0) + 1;
        if (typeof msg.rootZ === 'number') window.__ceer.rootZ = msg.rootZ;
        if (typeof msg.rootX === 'number') window.__ceer.rootX = msg.rootX;
        if (typeof msg.rootY === 'number') window.__ceer.rootY = msg.rootY;
      }
      updateHud(msg.rootZ);
      break;
    case 'error':
      onError(msg.message);
      break;
    default:
      console.warn('[viewer] unknown worker message', msg.type);
  }
};
worker.onerror = (e) => onError((e && e.message) || 'worker error');

// Pass the app's absolute base URL so the worker resolves /mt, /assets, /scene
// relative to it — works both in dev (/) and under a GitHub Pages subpath.
const APP_BASE = new URL(import.meta.env.BASE_URL, location.href).href;
worker.postMessage({ type: 'init', baseUrl: APP_BASE });

function onReady(msg) {
  try {
    sceneHandle = buildThreeSceneFromData(msg);
    scene.add(sceneHandle.group);
    // Camera stays framed on the robot (target set at init); do NOT retarget to
    // the whole-scene bounding box — the 40 m floor plane skews its center and
    // sends the orbit pivot off toward/under the ground.

    statusEl.classList.add('hidden');
    overlayEl.hidden = false;

    const rendered = sceneHandle.geomObjects.filter(Boolean).length;
    const m = msg.model;
    console.log(
      `[viewer] scene ready: ngeom=${m.ngeom} nbody=${m.nbody} ` +
        `nmesh=${m.nmesh} rendered=${rendered}`,
    );
    // Test/debug probe (harmless in prod): lets headless checks confirm the
    // worker + scene-build pipeline without depending on a visible render.
    window.__ceer = {
      ready: true,
      ngeom: m.ngeom,
      nbody: m.nbody,
      nmesh: m.nmesh,
      geomCount: rendered,
      rootZ: null,
      frames: 0,
    };

    // Start the policy control loop: the robot balances/stands and responds to
    // the keyboard. (The sim + policy run entirely in the worker.)
    worker.postMessage({ type: 'setMode', mode: 'policy' });
    if (stepToggle) stepToggle.checked = true;
  } catch (err) {
    onError((err && err.message) || String(err));
  }
}

function onError(message) {
  console.error('[viewer] error:', message);
  window.__ceer = { ready: false, error: message };
  setStatus('Failed to load scene:\n' + message, true);
}

// --- run/pause toggle: forward to the worker -------------------------------
if (stepToggle) {
  stepToggle.addEventListener('change', () => {
    worker.postMessage({ type: 'setStepping', on: stepToggle.checked });
  });
}

if (resetBtn) {
  resetBtn.addEventListener('click', () => worker.postMessage({ type: 'reset' }));
}

// HUD: root height + control-loop fps (frame messages arrive at the sim rate).
function updateHud(rootZ) {
  if (hudZ && typeof rootZ === 'number') hudZ.textContent = rootZ.toFixed(2);
  _fpsCount++;
  const now = performance.now();
  if (_fpsLast === 0) _fpsLast = now;
  if (now - _fpsLast >= 500) {
    const fps = Math.round((_fpsCount * 1000) / (now - _fpsLast));
    if (hudFps) hudFps.textContent = String(fps);
    _fpsCount = 0;
    _fpsLast = now;
  }
}

// --- keyboard teleop -------------------------------------------------------
// Forward the deploy key bindings to the worker (WASD walk, QE yaw, IJKL/UO
// hands, SPACE stop, R reset). See teleop.js for the exact semantics.
const TELEOP_KEYS = new Set([
  'w', 'a', 's', 'd', 'q', 'e', 'i', 'k', 'j', 'l', 'u', 'o', ' ',
]);
window.addEventListener('keydown', (e) => {
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  if (k === 'r') {
    worker.postMessage({ type: 'reset' });
    e.preventDefault();
    return;
  }
  if (TELEOP_KEYS.has(k)) {
    worker.postMessage({ type: 'key', key: k });
    if (k === ' ') e.preventDefault(); // don't scroll the page on SPACE
  }
});

// --- render loop -----------------------------------------------------------
function animate() {
  requestAnimationFrame(animate);
  if (controls) controls.update();
  if (renderer) renderer.render(scene, camera);
}
if (renderer) animate();

// --- helpers ---------------------------------------------------------------
function aspect() {
  return window.innerWidth / window.innerHeight;
}

// Vertical deep-blue gradient used as the scene backdrop (and, via the Reflector
// reading scene.background, the empty-floor reflection colour).
function makeGradientBackground() {
  const cv = document.createElement('canvas');
  cv.width = 4;
  cv.height = 256;
  const ctx = cv.getContext('2d');
  const g = ctx.createLinearGradient(0, 0, 0, 256);
  g.addColorStop(0.0, '#0a1524'); // dark navy top
  g.addColorStop(0.55, '#1c3a55'); // mid blue
  g.addColorStop(1.0, '#2c4a66'); // lighter steel-blue toward the horizon
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 4, 256);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

window.addEventListener('resize', () => {
  camera.aspect = aspect();
  camera.updateProjectionMatrix();
  if (renderer) renderer.setSize(window.innerWidth, window.innerHeight);
});
