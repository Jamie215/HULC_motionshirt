// Redraw the diagrams in README.md and docs/TECHNICAL_REFERENCE.md as PNG
// images in docs/img/diagrams/.
//
//   node docs/img/make_diagrams.cjs
//
// Each diagram's Mermaid source is kept in the Markdown, in a collapsed
// section just below its image:
//
//   ![...](img/diagrams/name.png)
//
//   <!-- diagram: name -->
//   <details><summary>Mermaid source</summary>
//
//   ```text
//   flowchart LR
//       ...
//   ```
//
//   </details>
//
// (The source is marked `text`, not `mermaid`, so that GitHub shows it as
// code instead of drawing it.)
// Edit the source there, then run this script and commit the new
// image. The diagrams are drawn as images because GitHub's own Mermaid
// rendering cuts off longer labels.
//
// Needs Node with Playwright (`npm install -g playwright`, then
// `npx playwright install chromium` once). Mermaid is loaded from a CDN, or
// from a local copy given in $MERMAID_JS.
const { execFileSync } = require('child_process');
const fs = require('fs'), path = require('path');

function loadPlaywright() {
  try { return require('playwright'); } catch (_) {
    const root = execFileSync('npm', ['root', '-g']).toString().trim();
    return require(path.join(root, 'playwright'));
  }
}
const { chromium } = loadPlaywright();

const REPO = path.resolve(__dirname, '..', '..');
const OUT = path.join(__dirname, 'diagrams');
const DOCS = ['README.md', path.join('docs', 'TECHNICAL_REFERENCE.md')];
const MERMAID_CDN = 'https://cdn.jsdelivr.net/npm/mermaid@11.4.1/dist/mermaid.min.js';

const jobs = [];
for (const doc of DOCS) {
  const text = fs.readFileSync(path.join(REPO, doc), 'utf8');
  const re = /<!-- diagram: ([\w-]+) -->\n<details><summary>Mermaid source<\/summary>\n\n```text\n([\s\S]*?)```/g;
  let m;
  while ((m = re.exec(text))) jobs.push({ name: m[1], src: m[2], doc });
}

(async () => {
  fs.mkdirSync(OUT, { recursive: true });
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1400, height: 1000 }, deviceScaleFactor: 2 });
  await page.setContent('<html><body style="margin:0;background:#fff"><div id="o" style="display:inline-block;padding:12px;background:#fff"></div></body></html>');
  if (process.env.MERMAID_JS) await page.addScriptTag({ path: process.env.MERMAID_JS });
  else await page.addScriptTag({ url: MERMAID_CDN });
  await page.evaluate(() => mermaid.initialize({
    startOnLoad: false, theme: 'default', fontFamily: 'Arial, Helvetica, sans-serif',
    flowchart: { useMaxWidth: false, padding: 12 }, sequence: { useMaxWidth: false },
    state: { useMaxWidth: false }, gantt: { useMaxWidth: false },
  }));
  let failed = 0, i = 0;
  for (const job of jobs) {
    const result = await page.evaluate(async ({ src, id }) => {
      try {
        const { svg } = await mermaid.render(id, src);
        document.getElementById('o').innerHTML = svg;
        return 'ok';
      } catch (e) { return 'error: ' + e.message; }
    }, { src: job.src, id: 'd' + (i++) });
    if (result !== 'ok') { failed++; console.log(`${job.doc} ${job.name}: ${result}`); continue; }
    await page.locator('#o').screenshot({ path: path.join(OUT, job.name + '.png') });
    console.log(`${job.doc} ${job.name}: ok`);
  }
  await browser.close();
  console.log(`${jobs.length} diagrams, ${failed} failed`);
  process.exit(failed ? 1 : 0);
})();
