// Headless functional test: load the dev app in real Chrome (swiftshader WebGL),
// wait for the MuJoCo+scene pipeline to finish, report result + console.
import puppeteer from 'puppeteer-core';

const URL = process.env.URL || 'http://localhost:5173/';
const CHROME = process.env.CHROME || '/usr/bin/google-chrome';

const browser = await puppeteer.launch({
  executablePath: CHROME,
  headless: 'new',
  protocolTimeout: 180000,
  args: ['--no-sandbox',
         '--use-gl=angle', '--use-angle=swiftshader',
         '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const page = await browser.newPage();
const logs = [];
page.on('console', (m) => logs.push(`[${m.type()}] ${m.text()}`));
page.on('pageerror', (e) => logs.push(`[pageerror] ${e.message}`));
page.on('requestfailed', (r) =>
  logs.push(`[reqfail] ${r.url()} :: ${r.failure()?.errorText}`));
page.on('response', (r) => {
  if (r.status() >= 400 && !r.url().endsWith('favicon.ico'))
    logs.push(`[http ${r.status()}] ${r.url()}`);
});

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 30000 });

let result = null;
try {
  await page.waitForFunction(() => window.__ceer !== undefined, {
    timeout: 90000, polling: 500,
  });
  result = await page.evaluate(() => window.__ceer);
} catch (e) {
  result = { ready: 'TIMEOUT', note: String(e).slice(0, 120) };
}

const env = await page.evaluate(() => ({
  coi: self.crossOriginIsolated,
  sab: typeof SharedArrayBuffer !== 'undefined',
  status: document.querySelector('#status')?.textContent ?? null,
})).catch(() => ({ coi: '?', sab: '?', status: 'evaluate-blocked' }));

console.log('=== RESULT ===');
console.log('crossOriginIsolated:', env.coi, '| SharedArrayBuffer:', env.sab);
console.log('status overlay:', JSON.stringify(env.status));
console.log('__ceer:', JSON.stringify(result));
console.log('=== CONSOLE (last 30) ===');
console.log(logs.slice(-30).join('\n'));

await browser.close();
process.exit(0);
