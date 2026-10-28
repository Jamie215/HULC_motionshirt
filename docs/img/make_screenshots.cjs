// Regenerate the app / review-page screenshots in docs/img/ from one capture folder.
//
//   node docs/img/make_screenshots.cjs <capture-dir> [subject-id]
//
// Needs Python 3 + the repository requirements, and Node with Playwright
// (`npm install -g playwright`, then `npx playwright install chromium` once).
// It analyzes the block into a temporary folder, starts the HULC-MS app on a
// temporary workspace, and writes app-*.png and review-*.png here. The
// recording itself is never copied into the repository; check every image
// for anything identifying before committing.
const { execFileSync, spawn } = require('child_process');
const fs = require('fs'), os = require('os'), path = require('path');

function loadPlaywright() {
  try { return require('playwright'); } catch (_) {
    const root = execFileSync('npm', ['root', '-g']).toString().trim();
    return require(path.join(root, 'playwright'));
  }
}
const { chromium } = loadPlaywright();

const REPO = path.resolve(__dirname, '..', '..');
const IMG = __dirname;
const capture = path.resolve(process.argv[2] || '');
const subject = process.argv[3] || 'S01';
if (!process.argv[2] || !fs.existsSync(capture)) {
  console.error('usage: node docs/img/make_screenshots.cjs <capture-dir> [subject-id]');
  process.exit(2);
}
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'hulc_shots_'));
const out = path.join(tmp, 'out');
const PORT = 8799;
const base = `http://127.0.0.1:${PORT}`;
const sleep = ms => new Promise(r => setTimeout(r, ms));
const VIEW = { width: 1280, height: 800 };

async function api(method, p, body, raw) {
  const r = await fetch(base + p, {
    method, body: raw ?? (body ? JSON.stringify(body) : undefined),
    headers: raw ? {} : { 'Content-Type': 'application/json' },
  });
  return [r.status, await r.text()];
}

async function reviewShots(browser) {
  const p = await browser.newPage({ viewport: VIEW, colorScheme: 'light' });
  await p.goto('file://' + path.join(out, 'session.html'));
  await p.waitForTimeout(1500);
  const btn = name => p.getByRole('button', { name, exact: true });

  // neutral pose, raw vs calibrated: the 3-D view only, side by side later
  await btn('Neutral pose').click();
  await btn('Side').click();
  // clip to the 3-D scene, with the metrics panel and the legend card out of the way
  // the floating card around the legend: its widest ancestor still narrower than the scene
  const hideCard = vis => p.evaluate(vis => {
    let card = document.getElementById('legend');
    while (card.parentElement && card.parentElement.getBoundingClientRect().width < 420) card = card.parentElement;
    card.style.visibility = vis;
  }, vis);
  await hideCard('hidden');
  const sceneClip = await p.evaluate(() => {
    const v = document.getElementById('view').getBoundingClientRect();
    const m = document.getElementById('metrics').getBoundingClientRect();
    const x = Math.max(v.left, m.right) + 1;
    return { x, y: v.top, width: v.right - x, height: v.height };
  });
  await btn('Raw').click();
  await p.waitForTimeout(400);
  await p.screenshot({ path: path.join(tmp, 'neutral-raw.png'), clip: sceneClip });
  await btn('Calibrated').click();
  await p.waitForTimeout(400);
  await p.screenshot({ path: path.join(tmp, 'neutral-cal.png'), clip: sceneClip });
  await hideCard('visible');
  await btn('Front').click();

  // a movement graphed over time: the first DOF row of the first joint
  await p.getByText('Flexion / extension', { exact: true }).first().click();
  await p.waitForTimeout(600);
  await p.screenshot({ path: path.join(IMG, 'review-graph.png') });
  await p.close();

  execFileSync('python3', ['-c', `
from PIL import Image
a, b = Image.open(${JSON.stringify(path.join(tmp, 'neutral-raw.png'))}), Image.open(${JSON.stringify(path.join(tmp, 'neutral-cal.png'))})
w, h = a.size; gap = 16
c = Image.new('RGB', (w * 2 + gap, h), 'white'); c.paste(a, (0, 0)); c.paste(b, (w + gap, 0))
c.save(${JSON.stringify(path.join(IMG, 'review-neutral-raw-vs-calibrated.png'))}, optimize=True)
`]);
}

async function appShots(browser) {
  const ws = path.join(tmp, 'ws');
  const app = spawn('python3', [path.join(REPO, 'tools', 'app.py'), '--workspace', ws,
                                '--port', String(PORT), '--no-browser'], { stdio: 'ignore' });
  try {
    for (let i = 0; i < 50; i++) { try { await fetch(base + '/'); break; } catch (_) { await sleep(200); } }
    const p = await browser.newPage({ viewport: VIEW, colorScheme: 'light' });
    await p.goto(base + '/');
    await p.waitForTimeout(500);

    // the same calls the app's own selftest makes (tools/app.py selftest)
    const [, body] = await api('POST', '/api/sessions',
      { subject, protocol: 'sync-first', label: 'block1', no_opensense: true });
    const sid = JSON.parse(body).id;
    for (const f of fs.readdirSync(capture)) {
      if (!/\.(bin|seg\.json)$|^montage\.json$/.test(f)) continue;
      await api('PUT', `/api/sessions/${sid}/files/${f}`, null, fs.readFileSync(path.join(capture, f)));
    }
    await api('POST', `/api/sessions/${sid}/run`, {});
    for (let i = 0; i < 600; i++) {
      const st = JSON.parse((await api('GET', `/api/sessions/${sid}`))[1]);
      if (st.status === 'done' || st.status === 'failed') break;
      await sleep(500);
    }
    await p.reload();
    await p.waitForTimeout(800);
    await p.getByText(subject).first().click().catch(() => {});
    await p.waitForTimeout(1500);
    await p.screenshot({ path: path.join(IMG, 'app-block-check.png') });
    await p.close();
  } finally {
    await api('POST', '/api/quit', {}).catch(() => {});
    app.kill();
  }
}

(async () => {
  execFileSync('python3', [path.join(REPO, 'hulc-ms.py'), 'analyze', '--capture-dir', capture,
                           '--outdir', out, '--no-opensense'], { stdio: 'ignore' });
  const browser = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader'] });
  try { await reviewShots(browser); await appShots(browser); }
  finally { await browser.close(); }
  console.log('screenshots written to', IMG, '(temporary analysis in', tmp + ')');
})();
