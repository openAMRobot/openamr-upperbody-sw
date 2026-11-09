// SPDX-License-Identifier: MIT
// Write test/fixtures/roadmap_parity.json: the upper body of the openamrobot-docs roadmap model as its pages build it, for
// test/test_roadmap_parity.py (roadmap convergence phase 3: parity within 0.5 mm and 0.2 deg).
//
//   node scripts/roadmap_parity_dump.js <openamrobot-docs checkout> [out json] [--branch NAME] [--merged true|false]
//
// Needs Node with Playwright and its Chromium (npm i playwright && npx playwright install chromium). The script serves the
// checkout's docs/assets/roadmap itself (no other server), opens exploded.html?render=1 and, for each configuration of the parity
// report (bp000 at L1000 / L1175 / L1350, bp050 at L1000; head pitch 30 deg; arms at all joints 0, grip 0, the zero pose of the
// combined URDF), records in base_footprint:
//   parts: every part group of the upper body (under the lift assembly or the lift stage, plus lift_base_plate): per group its world
//          quaternion and every mesh with its name, material colour and world box (metres, 6 decimals). A group with more than
//          MAX_MESHES meshes (harness runs, fasteners) is recorded as one box (meshes null): those are roadmap-only parts.
//   arms:  per arm the world matrix of every link frame (base, links 1 to 7, fingers 1 and 2), as parity_dump.js of the docs records it.
// The source files of the scene (model.js, placement.json, finish.json) are pinned by SHA-256, the checkout by commit.
const { chromium } = require('playwright');
const crypto = require('crypto');
const fs = require('fs');
const http = require('http');
const path = require('path');
const { execFileSync } = require('child_process');

const argv = process.argv.slice(2);
const opt = (name, def) => { const i = argv.indexOf(name); if (i < 0) return def; const v = argv[i + 1]; argv.splice(i, 2); return v; };
const BRANCH = opt('--branch', null), MERGED = opt('--merged', 'false') === 'true';
const DOCS = path.resolve(argv[0] || '.'), OUT = argv[1] || 'roadmap_parity.json';
const ROOT = path.join(DOCS, 'docs', 'assets', 'roadmap');
const CONFIGS = [['bp000', 'L1000', 0], ['bp000', 'L1175', 0.175], ['bp000', 'L1350', 0.35], ['bp050', 'L1000', 0]];
const MAX_MESHES = 40;
const SOURCES = ['model/model.js', 'model/placement.json', 'model/finish.json'];
const TYPES = {'.html': 'text/html', '.js': 'text/javascript', '.json': 'application/json', '.css': 'text/css', '.png': 'image/png',
               '.glb': 'model/gltf-binary', '.svg': 'image/svg+xml', '.woff2': 'font/woff2'};

function serve() {
  const server = http.createServer((req, res) => {
    const rel = decodeURIComponent(req.url.split('?')[0]).replace(/^\/+/, '');
    const file = path.resolve(ROOT, rel || 'index.html');
    if (!file.startsWith(ROOT + path.sep) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) { res.writeHead(404); res.end(); return; }
    res.writeHead(200, {'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream'});
    fs.createReadStream(file).pipe(res);
  });
  return new Promise(resolve => server.listen(0, '127.0.0.1', () => resolve(server)));
}

// one mesh per line, so a change of the roadmap model is a readable diff of the fixture
function format(doc) {
  const j = v => JSON.stringify(v);
  const out = ['{'];
  for (const k of Object.keys(doc).filter(k => k !== 'configs')) out.push(` ${j(k)}: ${j(doc[k])},`);
  out.push(' "configs": {');
  const cfgs = Object.keys(doc.configs);
  cfgs.forEach((c, ci) => {
    const cfg = doc.configs[c];
    out.push(`  ${j(c)}: {`, '   "parts": {');
    const ids = Object.keys(cfg.parts).sort();
    ids.forEach((id, ii) => {
      const groups = cfg.parts[id].map(g => {
        if (g.meshes === null) return `    {"q": ${j(g.q)}, "box": ${j(g.box)}, "meshes": null}`;
        return `    {"q": ${j(g.q)}, "meshes": [\n` + g.meshes.map(m => `     ${j(m)}`).join(',\n') + '\n    ]}';
      });
      out.push(`   ${j(id)}: [\n${groups.join(',\n')}\n   ]${ii < ids.length - 1 ? ',' : ''}`);
    });
    out.push('   },', `   "arms": ${j(cfg.arms)}`, `  }${ci < cfgs.length - 1 ? ',' : ''}`);
  });
  out.push(' }', '}');
  return out.join('\n') + '\n';
}

(async () => {
  const server = await serve();
  const base = `http://127.0.0.1:${server.address().port}/`;
  const browser = await chromium.launch({args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
  const page = await browser.newPage();
  const errors = [];
  page.on('pageerror', e => errors.push(String(e)));
  await page.goto(base + 'exploded.html?render=1', {waitUntil: 'load'});
  await page.waitForFunction(() => window.OAMR_MODEL && window.THREE, null, {timeout: 60000});
  const configs = await page.evaluate(([CONFIGS, MAX_MESHES]) => {
    const T = window.THREE, M = window.OAMR_MODEL, D = M.DIMS, res = {};
    const r6 = v => Math.round(v * 1e6) / 1e6;
    const box = o => { const b = new T.Box3().setFromObject(o); return b.isEmpty() ? null : [b.min.toArray().map(r6), b.max.toArray().map(r6)]; };
    CONFIGS.forEach(([bp, stop, tr]) => {
      const R = M.build(T, {headPitch: 30, bp: bp});
      R.mov.position.z += tr;
      ['left', 'right'].forEach(s => M.poseArm(T, R.arms[s], s, [0, 0, 0, 0, 0, 0, 0], 0, D.STOPS.L1000 + tr, D.BP[bp]));
      R.root.updateMatrixWorld(true);
      const upper = g => { for (let o = g; o; o = o.parent) if (o === R.col || o === R.mov) return true; return false; };
      const parts = {};
      Object.keys(R.parts).forEach(id => {
        if (id !== 'lift_base_plate' && !R.parts[id].some(upper)) return;
        parts[id] = R.parts[id].map(g => {
          const q = new T.Quaternion(), v = new T.Vector3(), s = new T.Vector3();
          g.matrixWorld.decompose(v, q, s);
          const meshes = [];
          g.traverse(o => { if (o.isMesh) meshes.push(o); });
          const qa = q.toArray().map(r6);
          if (meshes.length > MAX_MESHES) return {q: qa, box: box(g), meshes: null};
          return {q: qa, meshes: meshes.map(o => {
            const c = o.material && o.material.color ? o.material.color.getHexString() : null;
            return {name: o.name || '', color: c, box: box(o)};
          }).filter(m => m.box)};
        });
      });
      const arms = {};
      ['left', 'right'].forEach(s => {
        const rig = R.arms[s], m = g => g.matrixWorld.elements.map(r6);
        arms[s] = {base: m(rig.gBase), links: rig.gLinks.map(m), fingers: rig.fingers.map(f => m(f.g))};
      });
      res[bp + ' ' + stop] = {parts, arms};
    });
    return res;
  }, [CONFIGS, MAX_MESHES]);
  await browser.close();
  server.close();
  if (errors.length) { console.error('page errors:\n' + errors.join('\n')); process.exit(1); }
  const git = (...a) => execFileSync('git', ['-C', DOCS, ...a], {encoding: 'utf8'}).trim();
  const sha = {};
  SOURCES.forEach(f => { sha['docs/assets/roadmap/' + f] = crypto.createHash('sha256').update(fs.readFileSync(path.join(ROOT, f))).digest('hex'); });
  const doc = {
    about: 'The upper body of the openamrobot-docs roadmap model as its pages build it (scripts/roadmap_parity_dump.js); read by test/test_roadmap_parity.py. Generated, do not edit.',
    repository: 'openAMRobot/openamrobot-docs', commit: git('rev-parse', 'HEAD'), branch: BRANCH, merged: MERGED,
    sources_sha256: sha, frame: 'base_footprint (x forward, y left, z up), metres', head_pitch_deg: 30,
    configurations: Object.fromEntries(CONFIGS.map(([bp, stop, tr]) => [bp + ' ' + stop, {base_plate: bp, lift_joint: tr}])),
    configs,
  };
  fs.writeFileSync(OUT, format(doc));
  JSON.parse(fs.readFileSync(OUT, 'utf8'));
  console.log(OUT, Object.keys(configs).length, 'configurations,', Object.keys(configs['bp000 L1000'].parts).length, 'parts');
})().catch(e => { console.error(e); process.exit(1); });
