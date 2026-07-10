import { defineConfig } from 'vite';

// Deployed under a subpath, so use relative asset URLs.
export default defineConfig({
  // Relative asset URLs → the built app is subpath-agnostic (works at
  // /ceer_page/play/ on GitHub Pages or anywhere else).
  base: './',
  // Source lives in ceer_page/play-src/; the built static app is emitted to
  // ceer_page/play/ (what the main page's "Play in your browser" button links to
  // and what GitHub Pages serves at /ceer_page/play/).
  build: {
    outDir: '../play',
    emptyOutDir: true,
  },
  optimizeDeps: {
    exclude: ['mujoco'],
  },
  server: {
    fs: { allow: ['..'] },
    // The official MuJoCo WASM is a pthread build → needs SharedArrayBuffer,
    // which browsers gate behind cross-origin isolation. Send the required
    // headers in dev. (Production/GitHub Pages gets them via coi-serviceworker.)
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
  preview: {
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
});
