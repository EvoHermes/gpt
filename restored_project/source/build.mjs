// Combined DayuWriter Z rack lift + single-pinion dual-rack gripper + user's round claw.
// Units: mm. Source models are read-only; generated files stay in this package.
import fs from 'node:fs/promises';
import path from 'node:path';
import Module from 'manifold-3d';
import sharp from 'sharp';

const api = await Module(); api.setup();
const { Manifold: M, Mesh, CrossSection: C } = api;
const ROOT = path.resolve(import.meta.dirname, '..');
const OUT = path.join(ROOT, 'outputs');
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

// Donor parts and user claw; generated geometry uses millimetres.
const liftPart = {};
for (const name of ['frame', 'rack', 'pinion_block', 'servo_mount', 'pinion', 'horn_spacer']) liftPart[name] = await importSTL(path.join(LIFT, `${name}.stl`));
const clawPart = {};
for (const name of ['frame', 'cover', 'gear_compound', 'servo_pinion', 'jaw_left', 'jaw_right']) clawPart[name] = await importSTL(path.join(CLAWO, `${name}.stl`));
const roundSource = await importSTL(ROUND);
const xSlider = await importSTL(XSLIDER);
const sliderAdapter = await importSTL(SLIDER_ADAPTER);

const P = {
  units: 'mm', stroke: 80, lift_pinion_teeth: 30, lift_module: 2, lift_pinion_pitch_radius: 30,
  lift_pinion_center_z: 190, lift_rack_bottom_retracted_z: 94, lift_rack_length: 105,
  claw_height: 86, claw_bottom_retracted_z: 8, claw_bottom_extended_z: 88,
  guide: { type: 'paired captured rectangular shoes in fixed channels; no Z guide rods', clearance_each_side: 0.5, support_length: 50, spacing: 120 },
  z_servo_rotation_required_deg: +(80 / 30 * 180 / Math.PI).toFixed(2),
  jaw_gap: { open: 90, commanded_min: 52, safe_close_min_checked: 70 },
  overall_top_datum_z: 280,
  top_to_lowest_claw_tip: 272,
  claw_mount: { platform_hole_diameter: 3.4, platform_hole_spacing: 24, hole_x: [-12, 12], hole_y: -74, count: 2 },
  rack_platform_mount: { fastener: 'M3', clearance_diameter: 3.4, hole_x: [34, 40], hole_z: 6, count: 2 },
};

// Symmetric X-slider interface and Z-servo cage. The four M4 holes match the
// existing 50 x 54 x 27 slider adapter plate; side rails hold the servo symmetrically.
function xSliderServoBracket() {
  let m = union(
    // Symmetric plate on the X slider and a rearward cage for the lift servo.
    box([-28, -42, 226], [28, -38, 280]),
    box([-83, -42, 226], [83, -38, 230]),
    box([-83, -144, 176], [-77, -38, 226]),
    box([77, -144, 176], [83, -38, 226]),
    box([-83, -144, 172], [83, -140, 176]),
    box([-83, -144, 208], [83, -140, 212]),
    box([-24, -140, 172], [-20, -120, 208]),
    box([20, -140, 172], [24, -120, 208]),
    box([-24, -140, 172], [24, -136, 176]),
    box([-24, -140, 204], [24, -136, 208]),
  );
  for (const x of [-20, 20]) for (const z of [234.95, 271.05]) m = m.subtract(cylY(2.2, -43, -37, x, z));
  return m;
}

// Compact frame: preserve the Dayu X-slider face and keep the lift envelope under 280 mm.
function stableFrame() {
  let m = union(
    box([-83,-116,100],[-77,-42,266]),
    box([77,-116,100],[83,-42,266]),
    box([-83,-82,260],[83,-42,266]),
    box([-83,-116,260],[83,-112,266]),
    box([-28,-46,226],[28,-42,280]),
  );
  m=m.subtract(box([27,-99,199],[46,-79,281]));
  for(const x of [-20,20])for(const z of [234.95,271.05]) m=m.subtract(cylY(2.2,-47,-41,x,z));
  return m;
}
// Moving rack shoe. The open side faces the pinion; the 50 mm bearing faces
// slide in a fixed channel and resist carriage rotation without guide rods.
function compactGuideCarriage() {
  const z0 = 27, z1 = z0 + P.guide.support_length;
  let m = union(
    // The rack-side bridge and twin captured shoes resist tilt without rods.
    box([35.5, -97, z0], [68, -81, z1]),
    box([51.5, -100.5, z0], [68.5, -97.5, z1]),
    box([51.5, -80.5, z0], [68.5, -77.5, z1]),
    box([64.5, -97.5, z0], [68.5, -80.5, z1]),
    box([-68.5, -100.5, z0], [-51.5, -97.5, z1]),
    box([-68.5, -80.5, z0], [-51.5, -77.5, z1]),
    box([-68.5, -97.5, z0], [-64.5, -80.5, z1]),
    box([54, -95, 8], [66, -80, z0 + 0.2]),
    box([-66, -95, 8], [-54, -80, z0 + 0.2]),
    // Tie the two captured shoes together independently of the claw platform.
    box([35.5,-89,z1-7],[47,-62,z1]),
    box([-44,-89,z1-7],[-35.5,-62,z1]),
    box([-68,-97,z0],[-35.5,-81,z1]),
    box([-44,-69,z1-7],[44,-62,z1]),
  );
  // Rack cradle: clearance pocket and rear bolting flange, no solid overlap.
  m=m.subtract(box([31.6,-97.2,z0-0.1],[44.2,-80.8,z1+0.1]));
  m=m.add(box([35.5,-101,z0],[65,-96.5,z1]));
  for (const z of [z0 + 8, z1 - 8]) m = m.subtract(cylY(1.7, -102, -80, 38, z));
  for(const x of [-60,60])for(const y of [-92,-85]) {
    m=m.subtract(cylZ(1.7,7.9,23,x,y));
    m=m.subtract(cylZ(2.1,7.9,12.5,x,y));
  }
  return m;
}

// YZ rectangular brace beam between two points, extruded across X.
function braceX(xc, width, y0, z0, y1, z1, thick) {
  const dy = y1 - y0, dz = z1 - z0, len = Math.hypot(dy, dz), a = Math.atan2(dz, dy) * 180 / Math.PI;
  return box([-width / 2, -len / 2, -thick / 2], [width / 2, len / 2, thick / 2])
    .rotate([a, 0, 0]).translate([xc, (y0 + y1) / 2, (z0 + z1) / 2]);
}

const frame = stableFrame();
const carriage = compactGuideCarriage();
const zServoBracket = xSliderServoBracket();
function compactRack() {
  // The small claw platform sits below the rack at the local z=0 datum.
  // The rack spine starts at z=7 and reaches z=105, keeping the whole moving
  // envelope below the 280 mm top datum through the full 80 mm travel.
  let m = box([31.875, -97, 4], [44, -81, P.lift_rack_length]);
  const mod = P.lift_module, pitch = Math.PI * mod, halfPitchThickness = Math.PI * mod / 4, ta = Math.tan(20 * Math.PI / 180);
  // Standard 20-degree rack trapezoid: half-tooth thickness is pi*m/4
  // at pitch; tip/root widths account for addendum and 1.25m dedendum.
  // Roots overlap the rack bar by 0.15 mm for a watertight solid union.
  const toothAt = z => extrudeXZ(new C([[[32.025, z - halfPitchThickness - 1.25 * mod * ta], [32.025, z + halfPitchThickness + 1.25 * mod * ta], [28.5, z + halfPitchThickness - mod * ta], [28.5, z - halfPitchThickness + mod * ta]]]), 6, -89);
  for (let k = 0; k <= 18; k++) {
    const zc = 10 + k * pitch;
    if (zc < P.lift_rack_length - 1) m = m.add(toothAt(zc));
  }
  for (const z of [35, 69]) m = m.subtract(cylY(1.7, -98, -80, 38, z));
  // Side-facing M3 cross-bores secure the rack to the front platform flange.
  for (const x of P.rack_platform_mount.hole_x) {
    m = m.subtract(cylY(1.7, -98, -66, x, P.rack_platform_mount.hole_z));
  }
  return m;
}

// Compact mounting platform below the rack. The socket captures the rack base;
// the two lower M3 holes match the downward-facing claw top.
function rackClawPlatform() {
  // Compact plate butts against the rack's front face, outside the pinion plane.
  let m=box([-22,-81,0],[42.5,-67,8]);
  // The outer jaw sweeps past the plate edge; lower this small corner for clearance.
  m=m.subtract(box([32,-81.1,7],[42.6,-66.9,8.1]));
  for(const x of P.claw_mount.hole_x)m=m.subtract(cylZ(1.7,-0.1,8.1,x,P.claw_mount.hole_y));
  for(const x of P.rack_platform_mount.hole_x)m=m.subtract(cylY(1.7,-82,-66,x,P.rack_platform_mount.hole_z));
  return m;
}
function compactRackGuide() {
  const z0 = 100, z1 = 254;
  // Two mirrored C-channels capture the shoes and leave the central pinion
  // and rack teeth open. Each channel has 0.5 mm nominal running clearance.
  let m = union(
    box([69, -101, z0], [73, -77, z1]),
    box([49, -104, z0], [73, -101, z1]),
    box([49, -77, z0], [73, -74, z1]),
    box([73, -104, z0], [77, -101, z1]),
    box([-73, -101, z0], [-69, -77, z1]),
    box([-73, -104, z0], [-49, -101, z1]),
    box([-73, -77, z0], [-49, -74, z1]),
    box([-77, -104, z0], [-73, -101, z1]),
  );
  for (const z of [110, 228]) {
    m = m.subtract(cylX(2.2, 68, 74, -89, z));
    m = m.subtract(cylX(2.2, -74, -68, -89, z));
  }
  return m;
}

function liftPinion() {
  const N = P.lift_pinion_teeth, mod = P.lift_module, rp = P.lift_pinion_pitch_radius;
  const rb = rp * Math.cos(20 * Math.PI / 180), rr = rp - 1.25 * mod, ra = rp + mod;
  const inv = t => t <= rb ? 0 : Math.sqrt((t / rb) ** 2 - 1) - Math.acos(rb / t);
  const half = (Math.PI - 0.2) / (2 * N), a = t => half + inv(rp) - inv(t);
  const pol = (t, q) => [t * Math.cos(q), t * Math.sin(q)];
  const rs = [rr, rb, ...Array.from({ length: 9 }, (_, i) => rb + (ra - rb) * (i + 1) / 9)];
  const tooth = [...rs.map(t => pol(t, -a(t))), ...Array.from({ length: 7 }, (_, i) => pol(ra, -a(ra) + 2 * a(ra) * (i + 1) / 7)), ...rs.slice(0, -1).reverse().map(t => pol(t, a(t)))];
  let cs = C.circle(rr, 192);
  for (let k = 0; k < N; k++) cs = cs.add(new C([tooth]).rotate(360 * k / N));
  let m = extrudeXZ(cs, 8).add(cylY(12, -12, -4));
  const cuts = [cylY(4.1, -13, 5, 0, 0), cylY(3, 2, 6, 0, 0)];
  for (let ang = 45; ang < 360; ang += 90) { const q = ang * Math.PI / 180; cuts.push(cylY(1.7, -13, 6, 6 * Math.cos(q), 6 * Math.sin(q))); }
  cuts.push(cylX(1.5, -4, 4, -8, 0));
  return m.subtract(union(cuts));
}
const rackLong = compactRack();
const liftPinionModel = liftPinion();
const rackGuide = compactRackGuide();
const clawPlatform = rackClawPlatform();
const CLAW_Y = 55;
function spurGearXY(teeth, thickness, bore, module = 1) {
  const rp = teeth * module / 2, rb = rp * Math.cos(20 * Math.PI / 180), rr = rp - 1.25 * module, ra = rp + module;
  const inv = r => r <= rb ? 0 : Math.sqrt((r / rb) ** 2 - 1) - Math.acos(rb / r);
  const half = (Math.PI - 0.2) / (2 * teeth), flank = r => half + inv(rp) - inv(r);
  const pol = (r, a) => [r * Math.cos(a), r * Math.sin(a)];
  const rs = [rr, rb, ...Array.from({ length: 8 }, (_, i) => rb + (ra - rb) * (i + 1) / 8)];
  const tooth = [...rs.map(r => pol(r, -flank(r))), ...Array.from({ length: 7 }, (_, i) => pol(ra, -flank(ra) + 2 * flank(ra) * (i + 1) / 7)), ...rs.slice(0, -1).reverse().map(r => pol(r, flank(r)))];
  let cs = C.circle(rr, 128);
  for (let i = 0; i < teeth; i++) cs = cs.add(new C([tooth]).rotate(360 * i / teeth));
  return cs.extrude(thickness).subtract(cylZ(bore, -0.1, thickness + 0.1));
}
function linkBarXY(a, b, width, z0, z1, holeA = 2.1, holeB = holeA) {
  const dx = b[0] - a[0], dy = b[1] - a[1], len = Math.hypot(dx, dy), ang = Math.atan2(dy, dx) * 180 / Math.PI;
  const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const beam = box([-len / 2, -width / 2, z0], [len / 2, width / 2, z1]).rotate([0, 0, ang]).translate([mid[0], mid[1], 0]);
  return union(beam, cylZ(width / 2, z0, z1, a[0], a[1]), cylZ(width / 2, z0, z1, b[0], b[1]))
    .subtract(cylZ(holeA, z0 - 0.1, z1 + 0.1, a[0], a[1]), cylZ(holeB, z0 - 0.1, z1 + 0.1, b[0], b[1]));
}
const CLAW_PINION_TEETH = 12, CLAW_GEAR_MODULE = 1.5, CLAW_PITCH_RADIUS = 9;
const CLAW_RACK_PITCH = Math.PI * CLAW_GEAR_MODULE;
function jawRack(side, gap, zGear, yCenter) {
  const cx = side * gap / 2;
  const left = side < 0;
  const x0 = left ? cx - 5 : cx - 65, x1 = left ? cx + 65 : cx + 5;
  const rootOffset = CLAW_PITCH_RADIUS + 1.25 * CLAW_GEAR_MODULE;
  const tipOffset = CLAW_PITCH_RADIUS - CLAW_GEAR_MODULE;
  const sign = left ? 1 : -1;
  const yRoot = yCenter + sign * rootOffset, yTip = yCenter + sign * tipOffset;
  const ymin = left ? yRoot : yCenter - 15.5, ymax = left ? yCenter + 15.5 : yRoot;
  let rack = box([x0, ymin, zGear - 2.5], [x1, ymax, zGear + 2.5]);
  // Tooth positions remain fixed to each jaw; the pinion turns 1:1 with rack travel.
  const phaseOrigin = left ? cx + 45 : cx - 45;
  for (let k = -24; k <= 24; k++) {
    const x = phaseOrigin + k * CLAW_RACK_PITCH;
    if (x < x0 + 2 || x > x1 - 2) continue;
    // CrossSection treats winding as outer/hole orientation. Keep both
    // mirrored rack-tooth polygons counter-clockwise before extrusion.
    const points = left
      ? [[x - 1.8, yRoot], [x - 0.9, yTip], [x + 0.9, yTip], [x + 1.8, yRoot]]
      : [[x - 1.8, yRoot], [x + 1.8, yRoot], [x + 0.9, yTip], [x - 0.9, yTip]];
    rack = rack.add(new C([points]).extrude(5).translate([0, 0, zGear - 2.5]));
  }
  // A narrow rearward spine ties each rack to its moving jaw slider.
  const jawBackY = CLAW_Y - 38;
  const spine = box([cx - 2.5, jawBackY - 0.2, zGear - 2.5], [cx + 2.5, yRoot + sign * 0.2, zGear + 2.5]);
  return union(rack, spine);
}
function gripGeometry(gap, z0) {
  const y = CLAW_Y, axisY = y + 20;
  const zGear = z0 + 95.5;
  let plate = box([-62, y - 56, z0 + 78], [62, y + 50, z0 + 86]);
  plate = plate
    .subtract(box([-56, y - 55, z0 + 77.9], [-16, y - 44, z0 + 86.1]))
    .subtract(box([16, y - 55, z0 + 77.9], [56, y - 44, z0 + 86.1]))
    .subtract(box([-21, axisY - 11, z0 + 77.9], [21, axisY + 11, z0 + 86.1]))
    // Matched lightening windows on either side of the centered servo aperture.
    .subtract(box([-53, y - 35, z0 + 77.9], [-21, y + 5, z0 + 86.1]))
    .subtract(box([21, y - 35, z0 + 77.9], [53, y + 5, z0 + 86.1]));
  // Relief slots let the moving claw pass the fixed Z pinion and servo body.
  plate = plate
    .subtract(box([-33, y - 59, z0 + 77.9], [33, y - 49, z0 + 86.1]))
    .subtract(box([-26, y - 41, z0 + 77.9], [26, y - 21, z0 + 86.1]));
  // The removable platform mates to this matching 2 x M3 pattern.
  for (const x of P.claw_mount.hole_x) plate = plate.subtract(cylZ(1.7, z0 + 77.9, z0 + 86.1, x, y - 15));
  let frame = union(
    plate,
    // Open the gear deck like the reference: side cheeks and a rear bridge
    // leave the gears and crank arms visible and clear above the base plate.
    box([-62, y - 8, z0 + 86], [-58, y + 32, z0 + 110]),
    box([58, y - 8, z0 + 86], [62, y + 32, z0 + 110]),
    box([-62, y + 31.8, z0 + 102], [62, y + 50, z0 + 110]),
  );
  for (const yy of [y - 54, y - 44]) {
    frame = union(frame,
      box([-62, yy - 4, z0 + 86], [-56, yy + 4, z0 + 102]),
      box([56, yy - 4, z0 + 86], [62, yy + 4, z0 + 102]),
    );
  }
  for (const yy of [y - 54, y - 44]) frame = frame.subtract(cylX(3.25, -62.1, 62.1, yy, z0 + 92));
  const servo = box([-20, axisY - 10, z0 + 56], [20, axisY + 10, z0 + 92]);
  const servoBoss = union(cylZ(6, z0 + 92, zGear - 2.5, 0, axisY), cylZ(3, zGear - 2.5, z0 + 102.5, 0, axisY));
  const pinionAngle = 15 - (90 - gap) / (2 * CLAW_PITCH_RADIUS) * 180 / Math.PI;
  const servoPinion = spurGearXY(CLAW_PINION_TEETH, 5, 3.1, CLAW_GEAR_MODULE)
    .rotate([0, 0, pinionAngle]).translate([0, axisY, zGear - 2.5]);
  const makeJaw = side => {
    const cx = side * gap / 2, padCx = cx;
    const pad = side < 0
      ? roundSource.translate([padCx - 28, y - 44, z0])
      : roundSource.rotate([0, 0, 180]).translate([padCx + 28, y + 44, z0]);
    let web = side < 0
      ? box([cx - 7, y - 49, z0 + 22], [cx + 5, y - 39, z0 + 96])
      : box([cx - 5, y - 49, z0 + 22], [cx + 7, y - 39, z0 + 96]);
    for (const yy of [y - 54, y - 44]) web = web.subtract(cylX(3.25, cx - 7.1, cx + 7.1, yy, z0 + 92));
    let slider = box([cx - 9, y - 62, z0 + 88], [cx + 9, y - 38, z0 + 108]);
    for (const yy of [y - 54, y - 44]) slider = slider.subtract(cylX(3.25, cx - 9.1, cx + 9.1, yy, z0 + 92));
    slider = slider.subtract(cylZ(2.1, z0 + 87.9, z0 + 108.1, cx, y - 50));
    return union(pad, web, slider, jawRack(side, gap, zGear, axisY));
  };
  return { frame, servo, servoBoss, servoPinion,
    jawL: makeJaw(-1), jawR: makeJaw(1), guideA: cylX(3, -58, 58, y - 54, z0 + 92), guideB: cylX(3, -58, 58, y - 44, z0 + 92) };
}
const report = {
  kind: 'DayuWriter Z gear-rack lift + single-pinion dual-rack gripper + mirrored round claws',
  status: 'prototype mesh assembly; physical loads, fits, and servo performance remain unverified',
  parameters: P, parts: {}, poses: [], collision_checks: [], ball35: {},
};

// Export newly designed printable pieces and copy the unchanged donor parts into this package.
report.parts.z_lift_frame = await exportPart('z_lift_frame', frame);
report.parts.z_lift_carriage = await exportPart('z_lift_carriage', carriage);
report.parts.z_rack = await exportPart('z_rack', rackLong);
report.parts.z_pinion = await exportPart('z_pinion', liftPinionModel);
report.parts.z_rack_guide_pair_reference = await exportPart('z_rack_guide', rackGuide);
for(const [i,part] of rackGuide.decompose().entries())report.parts['z_rack_guide_'+i]=await exportPart('z_rack_guide_'+i,part);
report.parts.z_claw_platform = await exportPart('z_claw_platform_2xM3', clawPlatform);
report.parts.x_slider_adapter = await exportPart('x_slider_adapter', sliderAdapter);
report.parts.z_servo_mount_symmetric = await exportPart('z_servo_mount_symmetric', zServoBracket);
const gripForPrint = gripGeometry(90, 0);
for (const [name, model] of Object.entries({
  linkage_frame: gripForPrint.frame,
  linkage_servo_pinion: gripForPrint.servoPinion,
  linkage_jaw_left: gripForPrint.jawL, linkage_jaw_right: gripForPrint.jawR,
})) report.parts[name] = await exportPart(name, model);
for (const stale of ['linkage_gear_left.stl', 'linkage_gear_right.stl', 'linkage_crank_left.stl', 'linkage_crank_right.stl', 'linkage_gear_crank_left.stl', 'linkage_gear_crank_right.stl', 'linkage_rod_left.stl', 'linkage_rod_right.stl'])
  await fs.rm(path.join(OUT, stale), { force: true });
const copied = [
  ['z_horn_spacer', LIFT, 'horn_spacer'],
];
for (const [dest, dir, src] of copied) await fs.copyFile(path.join(dir, `${src}.stl`), path.join(OUT, `${dest}.stl`));
await fs.copyFile(XSLIDER, path.join(OUT, 'x_axis_slider_reference.stl'));

// Assembly placements use the source models' native coordinate frames.
function itemsFor(stroke, gap, includeBall = false) {
  const z0 = P.claw_bottom_retracted_z + stroke;
  const clawRaw = gripGeometry(gap, z0);
  const claw = Object.fromEntries(Object.entries(clawRaw).map(([name, model]) => [name, model.translate([0, -114, 0])]));
  const items = [
    { name: 'z_frame', m: frame, c: '#657f9d' },
    { name: 'x_slider', m: xSlider.rotate([90, 0, 0]).translate([-25, -3, 226]), c: '#4d5968' },
    { name: 'x_slider_adapter', m: sliderAdapter.translate([0, 0, 27]).rotate([90, 0, 0]).translate([-25, -3, 226]), c: '#d99a3e' },
    { name: 'z_servo_mount', m: zServoBracket, c: '#6d8199' },
    // Keep the original assembly datum here; only the exported STL is normalized.
    { name: 'z_carriage', m: carriage.translate([0, 0, P.lift_rack_bottom_retracted_z + stroke]), c: '#d39a42' },
    { name: 'z_claw_platform', m: clawPlatform.translate([0, 0, P.lift_rack_bottom_retracted_z + stroke]), c: '#d39a42' },
    { name: 'z_rack', m: rackLong.translate([0, 0, P.lift_rack_bottom_retracted_z + stroke]), c: '#d3a04a' },
    { name: 'z_rack_guide', m: rackGuide, c: '#6d8199' },
    { name: 'z_pinion', m: liftPinionModel.rotate([0, -(stroke / P.lift_pinion_pitch_radius) * 180 / Math.PI, 0]).translate([0, -89, P.lift_pinion_center_z]), c: '#d3a04a' },
    { name: 'z_servo', m: box([-20, -140, 172], [20, -120, 208]), c: '#32768d' },
    { name: 'z_servo_shaft', m: cylY(4, -122, -84, 0, P.lift_pinion_center_z), c: '#b6c1cb' },
    { name: 'gripper_frame', m: claw.frame, c: '#7890ad' },
    { name: 'grip_servo', m: claw.servo, c: '#32768d' },
    { name: 'grip_servo_boss', m: claw.servoBoss, c: '#32768d' },
    { name: 'grip_servo_pinion', m: claw.servoPinion, c: '#31343a' },
    { name: 'jaw_left_linkage', m: claw.jawL, c: '#d49a43' },
    { name: 'jaw_right_linkage', m: claw.jawR, c: '#d49a43' },
    { name: 'grip_guide_front', m: claw.guideA, c: '#b6c1cb' },
    { name: 'grip_guide_rear', m: claw.guideB, c: '#b6c1cb' },
  ];
  if (includeBall) items.push({ name: 'target_ball_35', m: M.sphere(17.5, 48).translate([0, -41, 36 + z0]), c: '#d95a4e' });
  return items;
}

// Save review-only assemblies at the key jaw and lift poses.
const poseDefs = [
  ['open_retracted', 0, 90, false],
  ['grip35_retracted', 0, 86, true],
  ['closed_retracted', 0, 70, false],
  ['open_extended', 80, 90, false],
  ['closed_extended', 80, 70, false],
];
for (const [label, s, gap, ball] of poseDefs) {
  const items = itemsFor(s, gap, ball), assembly = M.compose(items.map(x => x.m));
  await fs.writeFile(path.join(OUT, `assembly_${label}_REVIEW_ONLY.stl`), stlBuffer(assembly, `${label}; review only; mm`));
  const bb = assembly.boundingBox();
  const verticalSpan = bb.max[2] - bb.min[2];
  if (verticalSpan >= 280) throw new Error(`Assembly height exceeds the 280 mm limit: ${verticalSpan.toFixed(2)} mm (${label})`);
  report.poses.push({ label, lift_stroke_mm: s, jaw_base_gap_mm: gap, included_35mm_sphere: ball, bbox_mm: { min: bb.min, max: bb.max, size: bb.max.map((x, i) => +(x - bb.min[i]).toFixed(2)) } });
}

// Collision checks: intended rack/gear and bolted contacts are excluded by name.
const intended = new Set([
  'z_pinion|z_rack',
  'z_servo|z_servo_shaft',
  'jaw_left_linkage|grip_servo_pinion', 'jaw_right_linkage|grip_servo_pinion',
  'grip_guide_front|jaw_left_linkage', 'grip_guide_rear|jaw_left_linkage',
  'grip_guide_front|jaw_right_linkage', 'grip_guide_rear|jaw_right_linkage',
]);
function collides(items) {
  const out = [];
  for (let i = 0; i < items.length; i++) for (let j = i + 1; j < items.length; j++) {
    const a = items[i], b = items[j];
    const key = `${a.name}|${b.name}`, rev = `${b.name}|${a.name}`;
    if (intended.has(key) || intended.has(rev)) continue;
    const A = a.m.boundingBox(), B = b.m.boundingBox();
    if (A.max.some((v, k) => v <= B.min[k] + 1e-4) || B.max.some((v, k) => v <= A.min[k] + 1e-4)) continue;
    const overlap = a.m.intersect(b.m), vol = overlap.volume();
    if (vol > 0.05) out.push({ a: a.name, b: b.name, volume_mm3: +vol.toFixed(2), bbox_mm: overlap.boundingBox() });
  }
  return out;
}
for (let s=0;s<=80;s+=5) {
  for(const gap of [70,80,90]) {
    const c = collides(itemsFor(s, gap, false));
    report.collision_checks.push({ lift_stroke_mm: s, jaw_gap_mm: gap, unexpected: c });
  }
}
// Also check the worst-case closed-jaw envelope at low, middle, and high lift.
for (const s of [0, 40, 80]) {
  const c = collides(itemsFor(s, 70, false));
  report.collision_checks.push({ lift_stroke_mm: s, jaw_gap_mm: 70, unexpected: c });
}
for (const gap of [90, 86, 80, 70]) {
  const items = itemsFor(0, gap, false).filter(x => x.name.startsWith('jaw_'));
  report.collision_checks.push({ lift_stroke_mm: 0, jaw_gap_mm: gap, pair_clearance: collides(items) });
}
// Locate a geometry-only contact interval for the 35 mm reference sphere.
const sphere = M.sphere(17.5, 48).translate([0, 49, 44]);
let first = null, maxIntersection = 0;
for (let gap = 90; gap >= 52; gap -= 1) {
  const geometry = gripGeometry(gap, 0);
  const v = geometry.jawL.intersect(sphere).volume() + geometry.jawR.intersect(sphere).volume();
  if (v > 0.05 && first === null) first = { jaw_base_gap_mm: gap, intersection_mm3: +v.toFixed(3) };
  maxIntersection = Math.max(maxIntersection, v);
}
report.ball35 = { source: 'geometry-only reference sphere, Ø35 mm', center_relative_to_claw_top_mm: [0, -6, 36], first_sampled_contact: first, max_sampled_intersection_mm3: +maxIntersection.toFixed(3), step_mm: 1, note: 'Contact is not proof of holding force or grasp reliability.' };

// Mesh-based raster preview of three orientations.
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
await render(itemsFor(0, 90, false), path.join(OUT, 'preview_open_low.png'), { width: 1250, height: 1400, target: [0, -65, 140], label: '紧凑双齿条圆爪 · 张开 / 低位', sub: '80 mm 齿轮齿条升降 · 齿条下置 2×M3 夹爪安装平台' });
await render(itemsFor(0, 86, true), path.join(OUT, 'preview_ball35.png'), { width: 1250, height: 1400, target: [0, -65, 140], label: 'Ø35 mm 圆球试放 · 夹爪低位', sub: '球体仅作几何参考，未代表实物承载测试' });
await render(itemsFor(80, 70, false), path.join(OUT, 'preview_closed_high.png'), { width: 1250, height: 1400, target: [0, -65, 140], label: '双齿条夹爪 / 升降行程80 mm', sub: '升降与夹爪各由一只舵机驱动；闭合限位需按工件校准' });
const clawPreviewParts = itemsFor(0, 90, false).filter(x => x.name.startsWith('grip_') || x.name.startsWith('gripper_') || x.name.startsWith('jaw_'));
await render(clawPreviewParts, path.join(OUT, 'preview_claw_open.png'), { width: 1250, height: 1000, target: [0, -74, 54], az: 138, el: 52, label: '双齿条夹爪 · 张开', sub: '两侧用户圆爪保持槽口朝下；中心齿轮同步驱动两侧齿条' });
await render(itemsFor(0, 70, true).filter(x => x.name.startsWith('grip_') || x.name.startsWith('gripper_') || x.name.startsWith('jaw_') || x.name === 'target_ball_35'), path.join(OUT, 'preview_claw_ball35.png'), { width: 1250, height: 1000, target: [0, -74, 62], az: 138, el: 18, label: '双齿条夹爪 · Ø35 mm 几何试放', sub: '接触位置尚未验证；按真实工件校准闭合间隙和舵机限位' });

await fs.writeFile(path.join(OUT, 'validation.json'), JSON.stringify(report,null,2),'utf8');
// Final QA: exported printable meshes must be single closed solids.
for (const [name, m] of [['z_lift_frame', frame], ['z_lift_carriage', carriage], ['linkage_frame', gripForPrint.frame], ['z_servo_mount_symmetric', zServoBracket],
  ['linkage_jaw_left', gripForPrint.jawL], ['linkage_jaw_right', gripForPrint.jawR]]) {
  if (m.status() !== 'NoError' || m.decompose().length !== 1) throw new Error(`${name}: status=${m.status()} components=${JSON.stringify(m.decompose().map(c => ({ bbox: c.boundingBox(), volume: c.volume() })))}`);
}
await fs.writeFile(path.join(OUT, 'validation.json'), JSON.stringify(report, null, 2), 'utf8');
console.log(JSON.stringify({ parts: report.parts, poses: report.poses, ball35: report.ball35, nonemptyCollisionChecks: report.collision_checks.filter(x => (x.unexpected || x.pair_clearance || []).length) }, null, 2));


