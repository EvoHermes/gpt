// Combined DayuWriter Z rack lift + single-pinion dual-rack gripper + user's round claw.
// Units: mm. Source models are read-only; generated files stay in this package.
import fs from 'node:fs/promises';
import path from 'node:path';
import Module from 'manifold-3d';
import sharp from 'sharp';

const api = await Module(); api.setup();
const { Manifold: M, Mesh, CrossSection: C } = api;
const ROOT = path.resolve(import.meta.dirname, '..');
const OUT = path.join(ROOT, 'preview_revision');
const LIB = path.join(ROOT, 'reference');
const LIFT = path.join(LIB, 'servo_rack_lift_60', 'outputs');
const CLAW = path.join(LIB, 'claw_rack_120');
const CLAWO = path.join(CLAW, 'outputs');
const XSLIDER = path.join(LIB, 'servo_rack_lift_60', 'reference', 'Dayu_X_axis_slider.STL');
const SLIDER_ADAPTER = path.join(ROOT, 'reference', 'adapter.stl');
const ROUND = path.join(ROOT, 'reference', '圆爪.STL');
await fs.mkdir(OUT, { recursive: true });

const box = (a, b) => {
  const lo = a.map((v, i) => Math.min(v, b[i]));
  const hi = a.map((v, i) => Math.max(v, b[i]));
  return M.cube(hi.map((v, i) => v - lo[i])).translate(lo);
};
const union = (...xs) => M.union(xs.flat());
const cylZ = (r, z0, z1, x = 0, y = 0, n = 64) => M.cylinder(z1 - z0, r, r, n).translate([x, y, z0]);
const cylY = (r, y0, y1, x = 0, z = 0, n = 48) => M.cylinder(Math.abs(y1 - y0), r, r, n).rotate([90, 0, 0]).translate([x, Math.max(y0, y1), z]);
const cylX = (r, x0, x1, y = 0, z = 0, n = 48) => M.cylinder(Math.abs(x1 - x0), r, r, n).rotate([0, 90, 0]).translate([Math.min(x0, x1), y, z]);
const rod = (r, z0, z1, x, y) => cylZ(r, z0, z1, x, y);
const cut = (m, ...xs) => m.subtract(union(xs));
const normZ = m => { const b = m.boundingBox(); return m.translate([0, 0, -b.min[2]]); };
const extrudeXZ = (cs, t, y = 0) => cs.extrude(t).translate([0, 0, -t / 2]).rotate([90, 0, 0]).translate([0, y, 0]);

function parseBinarySTL(buf) {
  if (buf.length < 84) throw new Error('STL is too short');
  const n = buf.readUInt32LE(80);
  if (buf.length !== 84 + n * 50) throw new Error('Expected binary STL');
  const verts = [], faces = [], index = new Map();
  for (let i = 0; i < n; i++) {
    let o = 84 + i * 50 + 12;
    for (let j = 0; j < 3; j++) {
      const p = [buf.readFloatLE(o), buf.readFloatLE(o + 4), buf.readFloatLE(o + 8)];
      const key = p.map(x => x.toFixed(4)).join(',');
      let id = index.get(key);
      if (id === undefined) { id = verts.length / 3; index.set(key, id); verts.push(...p); }
      faces.push(id); o += 12;
    }
  }
  return { verts, faces };
}
async function importSTL(file) {
  const { verts, faces } = parseBinarySTL(await fs.readFile(file));
  return M.ofMesh(new Mesh({ numProp: 3, vertProperties: new Float32Array(verts), triVerts: new Uint32Array(faces) }));
}
function stlBuffer(m, label) {
  const me = m.getMesh(), b = Buffer.alloc(84 + me.numTri * 50);
  b.write(label.slice(0, 78), 0, 'ascii'); b.writeUInt32LE(me.numTri, 80);
  for (let i = 0; i < me.numTri; i++) {
    let o = 84 + i * 50 + 12;
    for (let j = 0; j < 3; j++) {
      const vi = me.triVerts[i * 3 + j] * me.numProp;
      for (let k = 0; k < 3; k++) { b.writeFloatLE(me.vertProperties[vi + k], o); o += 4; }
    }
  }
  return b;
}
async function exportPart(name, m) {
  if (m.status() !== 'NoError') throw new Error(`${name}: ${m.status()}`);
  const p = normZ(m);
  await fs.writeFile(path.join(OUT, `${name}.stl`), stlBuffer(p, `${name}; mm; prototype`));
  const check = await importSTL(path.join(OUT, `${name}.stl`));
  const bb = check.boundingBox();
  return { status: check.status(), shells: check.decompose().length, volume_mm3: +check.volume().toFixed(2), bbox_mm: { min: bb.min, max: bb.max }, triangles: check.numTri() };
}

const R = Math.PI / 180;
function color(hex) { return hex.match(/\w\w/g).map(x => parseInt(x, 16)); }
async function render(items, file, { width = 1250, height = 1000, az = 138, el = 17, target = [0, -28, 260], label = '', sub = '' } = {}) {
  const ss = 1.1, w = Math.round(width * ss), h = Math.round(height * ss);
  const pix = new Uint8Array(w * h * 3), depth = new Float32Array(w * h).fill(Infinity);
  for (let i = 0; i < w * h; i++) { pix[i * 3] = 247; pix[i * 3 + 1] = 249; pix[i * 3 + 2] = 252; }
  const all = M.compose(items.map(x => x.m)).boundingBox();
  const ex = Math.hypot(all.max[0] - all.min[0], all.max[1] - all.min[1], all.max[2] - all.min[2]);
  const sc = .88 * Math.min(w, h) / ex;
  const ca = Math.cos(az * R), sa = Math.sin(az * R), ce = Math.cos(el * R), se = Math.sin(el * R);
  const project = p => { const x = p[0] - target[0], y = p[1] - target[1], z = p[2] - target[2]; return [w / 2 + sc * (ca * x - sa * y), h * .52 - sc * (se * (sa * x + ca * y) + ce * z), ce * (sa * x + ca * y) - se * z]; };
  for (const it of items) {
    const me = it.m.getMesh(), vp = me.vertProperties, base = color(it.c);
    for (let i = 0; i < me.numTri; i++) {
      const vv = [0, 1, 2].map(j => { const id = me.triVerts[3 * i + j] * me.numProp; return [vp[id], vp[id + 1], vp[id + 2]]; });
      const v = vv.map(project), ab = vv[1].map((q, k) => q - vv[0][k]), ac = vv[2].map((q, k) => q - vv[0][k]);
      const n = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]], len = Math.hypot(...n) || 1;
      const light = .58 + .42 * Math.max(0, (-.35 * n[0] - .6 * n[1] + .72 * n[2]) / len), rgb = base.map(q => Math.round(q * light));
      const [a, b, c] = v, area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
      if (Math.abs(area) < 1e-8) continue;
      const x0 = Math.max(0, Math.floor(Math.min(a[0], b[0], c[0]))), x1 = Math.min(w - 1, Math.ceil(Math.max(a[0], b[0], c[0])));
      const y0 = Math.max(0, Math.floor(Math.min(a[1], b[1], c[1]))), y1 = Math.min(h - 1, Math.ceil(Math.max(a[1], b[1], c[1])));
      for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
        const u = ((b[0] - x - .5) * (c[1] - y - .5) - (b[1] - y - .5) * (c[0] - x - .5)) / area;
        const vv2 = ((c[0] - x - .5) * (a[1] - y - .5) - (c[1] - y - .5) * (a[0] - x - .5)) / area, wa = 1 - u - vv2;
        if (u < 0 || vv2 < 0 || wa < 0) continue;
        const d = u * a[2] + vv2 * b[2] + wa * c[2], id = y * w + x;
        if (d < depth[id]) { depth[id] = d; pix[3 * id] = rgb[0]; pix[3 * id + 1] = rgb[1]; pix[3 * id + 2] = rgb[2]; }
      }
    }
  }
  const overlay = `<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}"><g font-family="Arial,Microsoft YaHei,sans-serif"><text x="34" y="44" font-size="25" fill="#22344c" font-weight="bold">${label}</text><text x="34" y="72" font-size="15" fill="#6d7d92">${sub}</text><rect x="0" y="${height - 39}" width="${width}" height="39" fill="#f4f7fb"/><text x="34" y="${height - 14}" font-size="13" fill="#718198">审阅用总装示意 · 标准件占位 · 需实物核验</text></g></svg>`;
  const bg = await sharp(pix, { raw: { width: w, height: h, channels: 3 } }).resize(width, height).png().toBuffer();
  await sharp(bg).composite([{ input: Buffer.from(overlay) }]).png().toFile(file);
}

export {M,C,fs,path,OUT,box,cylY,cylZ,union,importSTL,render};