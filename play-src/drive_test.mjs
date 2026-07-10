// Headless test that keyboard teleop actually drives the robot: press W a few
// times and verify the base translates forward (rootX increases) while staying
// upright (rootZ stays ~standing). Exercises the full keydown→worker→teleop→
// policy→walk path.
import puppeteer from 'puppeteer-core';

const URL = process.env.URL || 'http://localhost:5173/';
const browser = await puppeteer.launch({
  executablePath: process.env.CHROME || '/usr/bin/google-chrome',
  headless: 'new', protocolTimeout: 120000, args: ['--no-sandbox'],
});
const page = await browser.newPage();
const logs = [];
page.on('console', (m) => logs.push(`[${m.type()}] ${m.text()}`));
page.on('pageerror', (e) => logs.push(`[pageerror] ${e.message}`));

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 30000 });
await page.waitForFunction(() => window.__ceer && window.__ceer.ready === true, { timeout: 90000, polling: 300 });

// let it settle standing
await new Promise((r) => setTimeout(r, 1500));
const start = await page.evaluate(() => ({ x: window.__ceer.rootX, z: window.__ceer.rootZ }));

// press W a few times (forward walk); dispatch real keydown events
for (let i = 0; i < 4; i++) {
  await page.keyboard.down('w');
  await page.keyboard.up('w');
  await new Promise((r) => setTimeout(r, 60));
}
// let it walk
await new Promise((r) => setTimeout(r, 4000));
const end = await page.evaluate(() => ({ x: window.__ceer.rootX, z: window.__ceer.rootZ }));

const dx = end.x - start.x;
console.log('=== DRIVE TEST ===');
console.log(`start x=${start.x.toFixed(3)} z=${start.z.toFixed(3)}`);
console.log(`end   x=${end.x.toFixed(3)} z=${end.z.toFixed(3)}`);
console.log(`Δx = ${dx.toFixed(3)} (forward walk), z stayed ${end.z.toFixed(3)}`);
const poolExhausted = logs.filter((l) => l.includes('thread pool is exhausted')).length;
// Forward command drives +x; expect a clear positive translation, still upright.
const pass = dx > 0.1 && end.z > 0.6 && poolExhausted === 0;
console.log(pass ? 'PASS — W drives the robot forward while balancing' : 'FAIL — did not walk / fell');
await browser.close();
process.exit(pass ? 0 : 1);
