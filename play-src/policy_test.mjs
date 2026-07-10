// Headless test of the FULL control loop: load the app, let the policy run in
// the worker for a few seconds, and check the robot stays standing (root height
// doesn't collapse) and frames advance — i.e. the ported obs+policy+PD actually
// balances the G1, matching the deploy sim. No WebGL needed.
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

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 30000 });

// wait for ready
try {
  await page.waitForFunction(() => window.__ceer && window.__ceer.ready === true, {
    timeout: 90000, polling: 300,
  });
} catch {
  console.log('NEVER READY'); console.log(logs.slice(-15).join('\n')); await browser.close(); process.exit(1);
}

// sample root height while the policy runs
const samples = [];
for (let i = 0; i < 12; i++) {
  await new Promise((r) => setTimeout(r, 400));
  const s = await page.evaluate(() => ({ z: window.__ceer.rootZ, f: window.__ceer.frames }));
  samples.push(s);
}

const zs = samples.map((s) => s.z).filter((z) => typeof z === 'number');
const minZ = Math.min(...zs);
const finalZ = zs[zs.length - 1];
const frames = samples[samples.length - 1].f;
const poolExhausted = logs.filter((l) => l.includes('thread pool is exhausted')).length;

console.log('=== POLICY TEST ===');
console.log('rootZ samples:', zs.map((z) => z.toFixed(3)).join(' '));
console.log('minZ =', minZ.toFixed(3), ' finalZ =', finalZ.toFixed(3), ' frames =', frames);
console.log('poolExhausted =', poolExhausted);
const errs = logs.filter((l) => l.startsWith('[pageerror]') || l.includes('[error]'));
if (errs.length) console.log('errors:\n' + errs.slice(-8).join('\n'));

// Standing ~0.79; allow some transient. Fail if it collapses (<0.55) or no frames.
const standing = minZ > 0.55 && finalZ > 0.6 && frames > 100 && poolExhausted === 0;
console.log(standing ? 'PASS — robot stands under policy control' : 'FAIL — collapsed / stalled');
await browser.close();
process.exit(standing ? 0 : 1);
