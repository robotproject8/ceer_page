// Grab a software-rendered screenshot of the demo to eyeball the camera + ground.
import puppeteer from 'puppeteer-core';
const URL = process.env.URL || 'http://127.0.0.1:8200/play/';
const OUT = process.env.OUT || '/tmp/ceer_shot.png';
const browser = await puppeteer.launch({
  executablePath: '/usr/bin/google-chrome',
  headless: 'new', protocolTimeout: 120000,
  args: ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader',
         '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist',
         '--window-size=1000,700'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1000, height: 700 });
const logs = [];
page.on('console', (m) => logs.push(m.text()));
await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 30000 });
try {
  await page.waitForFunction(() => window.__ceer && window.__ceer.ready === true, { timeout: 90000, polling: 300 });
} catch { console.log('not ready'); }
await new Promise((r) => setTimeout(r, 3500)); // let it stand + render a few frames
await page.screenshot({ path: OUT });
const glErr = logs.filter((l) => /WebGL|context/i.test(l)).slice(-3);
console.log('saved', OUT, '| ready=', await page.evaluate(() => window.__ceer && window.__ceer.ready).catch(() => '?'));
if (glErr.length) console.log('gl:', glErr.join(' | '));
await browser.close();
process.exit(0);
