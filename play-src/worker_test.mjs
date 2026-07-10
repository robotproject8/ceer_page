// Headless test of the WORKER-based sim path. Loads the full app, waits for
// window.__ceer.ready and asserts:
//   - NO "thread pool is exhausted" appears (proves the main-thread deadlock is
//     gone: MuJoCo now runs in the worker),
//   - __ceer.ngeom === 126.
// Headless Chrome has NO working WebGL, so a WebGLRenderer failure is EXPECTED;
// main.js is structured to keep the worker/sim path + __ceer working anyway.
import puppeteer from 'puppeteer-core';

const URL = process.env.URL || 'http://localhost:5173/';
const browser = await puppeteer.launch({
  executablePath: process.env.CHROME || '/usr/bin/google-chrome',
  headless: 'new',
  protocolTimeout: 120000,
  args: ['--no-sandbox'],
});
const page = await browser.newPage();
const logs = [];
page.on('console', (m) => logs.push(`[${m.type()}] ${m.text()}`));
page.on('pageerror', (e) => logs.push(`[pageerror] ${e.message}`));
page.on('response', (r) => {
  if (r.status() >= 400 && !r.url().endsWith('favicon.ico')) logs.push(`[http ${r.status()}] ${r.url()}`);
});

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 30000 });

let res;
try {
  await page.waitForFunction(() => window.__ceer !== undefined, { timeout: 90000, polling: 400 });
  res = await page.evaluate(() => window.__ceer);
} catch (e) {
  res = { ready: 'TIMEOUT', note: String(e).slice(0, 160) };
}

const poolExhausted = logs.filter((l) => l.includes('thread pool is exhausted'));

console.log('=== WORKER TEST RESULT ===');
console.log(JSON.stringify(res, null, 2));
console.log('poolExhaustedCount =', poolExhausted.length);

console.log('=== console (last 25) ===');
console.log(logs.slice(-25).join('\n'));

await browser.close();

const ok =
  res && res.ready === true && res.ngeom === 126 && poolExhausted.length === 0;
console.log('=== VERDICT ===');
console.log(ok ? 'PASS — worker sim ready, ngeom=126, no deadlock' : 'FAIL — see above');
process.exit(ok ? 0 : 1);
