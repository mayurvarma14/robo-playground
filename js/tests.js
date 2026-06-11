/**
 * tests.js — browser test harness. Load app with ?test=1 to run.
 * Results go to console and a fixed banner.
 */
import * as K from './kinematics.js';
import { ROBOTS, armDHRows, ARM_FLANGE, dexArmChainFK } from './robots.js';

let passed = 0, failed = 0;
const failures = [];

function test(name, fn) {
  try { fn(); passed++; }
  catch (e) { failed++; failures.push(`${name}: ${e.message}`); console.error(`FAIL ${name}`, e); }
}

function approx(actual, expected, tol = 1e-6, label = '') {
  if (Math.abs(actual - expected) > tol)
    throw new Error(`${label} expected ${expected}, got ${actual} (tol ${tol})`);
}

function approxArr(actual, expected, tol = 1e-6, label = '') {
  expected.forEach((e, i) => approx(actual[i], e, tol, `${label}[${i}]`));
}

// ── Task 1 tests: primitives
test('dhMatrix identity', () => {
  approxArr(K.dhMatrix(0, 0, 0, 0).flat(), K.matIdentity().flat());
});

test('dhMatrix pure d translation', () => {
  const T = K.dhMatrix(0, 5, 0, 0);
  approxArr(K.matPoint(T, [0, 0, 0]), [0, 0, 5]);
});

test('dhMatrix a along rotated x', () => {
  const T = K.dhMatrix(Math.PI / 2, 0, 3, 0);
  approxArr(K.matPoint(T, [0, 0, 0]), [0, 3, 0], 1e-9);
});

test('dhMatrix alpha twists z', () => {
  // alpha = 90deg: old Y -> new Z
  const T = K.dhMatrix(0, 0, 0, Math.PI / 2);
  approxArr(K.matPoint(T, [0, 1, 0]), [0, 0, 1], 1e-9);
});

test('matMul composes', () => {
  const T = K.matMul(K.matTrans(1, 0, 0), K.matRotZ(Math.PI / 2));
  approxArr(K.matPoint(T, [1, 0, 0]), [1, 1, 0], 1e-9);
});

test('solveLinear 3x3', () => {
  const x = K.solveLinear([[2,0,0],[0,3,0],[0,0,4]], [2, 6, 8]);
  approxArr(x, [1, 2, 2]);
});

test('solveLinear singular returns null', () => {
  if (K.solveLinear([[1,1],[1,1]], [1, 2]) !== null) throw new Error('expected null');
});

test('world/DH mapping roundtrip', () => {
  approxArr(K.dhToWorld([1, 2, 3]), [1, 3, -2]);
  approxArr(K.worldToDH(K.dhToWorld([1, 2, 3])), [1, 2, 3]);
});

// ── Task 2 tests: DHChain FK on the 6-DOF arm table
const ARM_P = { l1: 81, l2: 213, l3: 196, l4: 67 };
const FLANGE = 30;
const armRows = (q) => [
  [q[0],                ARM_P.l1 + 20, 0,        Math.PI / 2],
  [q[1] + Math.PI / 2,  0,             ARM_P.l2, 0],
  [q[2],                0,             ARM_P.l3, 0],
  [q[3] + Math.PI / 2,  0,             0,        Math.PI / 2],
  [q[4],                ARM_P.l4,      0,       -Math.PI / 2],
  [q[5] - Math.PI / 2,  0,             FLANGE,   0],
];
const armChain = new K.DHChain(armRows);

test('arm FK zero pose: straight up', () => {
  const H = 20 + ARM_P.l1 + ARM_P.l2 + ARM_P.l3 + ARM_P.l4 + FLANGE;
  approxArr(armChain.eePos([0,0,0,0,0,0]), [0, 0, H], 1e-6, 'eePos');
});

test('arm FK shoulder tilt 30deg', () => {
  const phi = 30 * K.DEG;
  const L = ARM_P.l2 + ARM_P.l3 + ARM_P.l4 + FLANGE;
  approxArr(
    armChain.eePos([0, phi, 0, 0, 0, 0]),
    [-L * Math.sin(phi), 0, 20 + ARM_P.l1 + L * Math.cos(phi)],
    1e-6, 'eePos'
  );
});

test('arm FK base yaw moves EE in DH -y', () => {
  // tilt shoulder then yaw base 90°: x-reach rotates to DH y axis
  const phi = 30 * K.DEG;
  const p0 = armChain.eePos([0, phi, 0, 0, 0, 0]);
  const p1 = armChain.eePos([Math.PI / 2, phi, 0, 0, 0, 0]);
  approxArr(p1, [0, p0[0], p0[2]], 1e-6, 'rotated'); // RotZ(90): x->y
});

test('manipulability: stretched pose is near-singular', () => {
  const wSing = armChain.manipulability([0, 0, 0, 0, 0, 0]);     // fully extended
  const wGood = armChain.manipulability([0, 40*K.DEG, -70*K.DEG, 20*K.DEG, 30*K.DEG, 0]);
  if (!(wSing < 1e-3)) throw new Error(`expected near-zero wSing, got ${wSing}`);
  if (!(wGood > 1)) throw new Error(`expected healthy wGood, got ${wGood}`);
});

// ── Task 3 tests: DLS IK
test('IK position roundtrip within 0.1mm', () => {
  const qTrue = [20*K.DEG, 35*K.DEG, -60*K.DEG, 15*K.DEG, 40*K.DEG, 10*K.DEG];
  const target = armChain.eePos(qTrue);
  const seed = [0, 20*K.DEG, -40*K.DEG, 0, 0, 0];
  const r = armChain.solveIK(seed, target);
  if (!r.converged) throw new Error(`did not converge, posErr=${r.posErr}`);
  approxArr(armChain.eePos(r.q), target, 0.1, 'fk(ik)');
});

test('IK full pose roundtrip', () => {
  const qTrue = [-15*K.DEG, 50*K.DEG, -80*K.DEG, 25*K.DEG, -30*K.DEG, 45*K.DEG];
  const { ee } = armChain.fk(qTrue);
  const targetPos = [ee[0][3], ee[1][3], ee[2][3]];
  const targetRot = [
    [ee[0][0], ee[0][1], ee[0][2]],
    [ee[1][0], ee[1][1], ee[1][2]],
    [ee[2][0], ee[2][1], ee[2][2]],
  ];
  const seed = [0, 30*K.DEG, -60*K.DEG, 10*K.DEG, 0, 0];
  const r = armChain.solveIK(seed, targetPos, targetRot);
  if (!r.converged) throw new Error(`did not converge, posErr=${r.posErr}`);
  approxArr(armChain.eePos(r.q), targetPos, 0.2, 'fk(ik) pos');
});

test('IK unreachable returns best-effort, no NaN', () => {
  const r = armChain.solveIK([0, 30*K.DEG, -60*K.DEG, 0, 0, 0], [0, 0, 5000]);
  if (r.converged) throw new Error('should not converge');
  r.q.forEach(v => { if (!Number.isFinite(v)) throw new Error('NaN in solution'); });
});

test('IK respects joint limits', () => {
  const lim = Array(6).fill({ min: -Math.PI / 2, max: Math.PI / 2 });
  const limited = new K.DHChain(armRows, lim);
  const r = limited.solveIK([0,0,0,0,0,0], [200, 200, 100]);
  r.q.forEach((v, i) => {
    if (v < lim[i].min - 1e-9 || v > lim[i].max + 1e-9) throw new Error(`joint ${i} out of limits: ${v}`);
  });
});

test('poseError finite and nonzero at 180deg rotation error', () => {
  const { ee } = armChain.fk([0, 0, 0, 0, 0, 0]);
  // target rotated pi about world Z relative to current EE orientation
  const flipped = [
    [-ee[0][0], -ee[0][1], ee[0][2]],
    [-ee[1][0], -ee[1][1], ee[1][2]],
    [-ee[2][0], -ee[2][1], ee[2][2]],
  ];
  const e = armChain.poseError(ee, [ee[0][3], ee[1][3], ee[2][3]], flipped, 20);
  e.forEach(v => { if (!Number.isFinite(v)) throw new Error('non-finite error'); });
  const mag = Math.hypot(e[3], e[4], e[5]);
  approx(mag, Math.PI * 20, 1e-6, 'rot error magnitude');
});

test('solvePositionIK on generic chain', () => {
  // simple 2-link planar chain as fk function
  const fk = (q) => [
    100 * Math.cos(q[0]) + 80 * Math.cos(q[0] + q[1]),
    100 * Math.sin(q[0]) + 80 * Math.sin(q[0] + q[1]),
    0,
  ];
  const target = fk([0.6, -0.9]);
  const r = K.solvePositionIK(fk, [0.1, -0.1], target);
  if (!r.converged) throw new Error(`posErr=${r.posErr}`);
  approxArr(fk(r.q), target, 0.1);
});

// ── Task 4 tests: analytical solvers
test('twoLink roundtrip both elbows', () => {
  for (const elbow of [1, -1]) {
    const t = K.twoLinkFK(0.7, elbow * 1.1, 100, 80);
    const r = K.twoLinkIK(t.x, t.y, 100, 80, elbow);
    if (Math.sign(r.q2) !== elbow) throw new Error(`elbow ${elbow}: q2=${r.q2} has wrong sign`);
    const back = K.twoLinkFK(r.q1, r.q2, 100, 80);
    approx(back.x, t.x, 1e-6); approx(back.y, t.y, 1e-6);
  }
});

test('twoLink unreachable flagged + clamped finite', () => {
  const r = K.twoLinkIK(500, 0, 100, 80);
  if (r.reachable) throw new Error('should be unreachable');
  if (!Number.isFinite(r.q1) || !Number.isFinite(r.q2)) throw new Error('NaN');
});

test('scara analytical agrees with FK', () => {
  const q1 = 0.5, q2 = -0.8, l1 = 150, l2 = 130;
  const p = K.scaraFK(q1, q2, l1, l2);
  const r = K.scaraIK(p.x, p.z, l1, l2, -1);
  const back = K.scaraFK(r.q1, r.q2, l1, l2);
  approx(back.x, p.x, 1e-6); approx(back.z, p.z, 1e-6);
});

test('scara analytical agrees with numerical DLS', () => {
  // numerical solution of same planar chain must land on same point
  const l1 = 150, l2 = 130;
  const target = K.scaraFK(0.4, 0.7, l1, l2);
  const a = K.scaraIK(target.x, target.z, l1, l2, 1);
  const fk = (q) => { const p = K.scaraFK(q[0], q[1], l1, l2); return [p.x, 0, p.z]; };
  const n = K.solvePositionIK(fk, [0.1, 0.2], [target.x, 0, target.z]);
  if (!n.converged) throw new Error('numeric failed');
  const pa = K.scaraFK(a.q1, a.q2, l1, l2);
  const pn = K.scaraFK(n.q[0], n.q[1], l1, l2);
  approx(pa.x, pn.x, 0.2); approx(pa.z, pn.z, 0.2);
});

test('leg IK roundtrip all 4 leg signs', () => {
  const C = 40, F = 80, T = 87;
  for (const side of [1, -1]) {
    for (const q of [[0.2, 0.4, -1.0], [-0.3, 0.1, -0.6]]) {
      const p = K.legFK(q[0], q[1], q[2], C, F, T, side);
      const r = K.legIK(p.x, p.y, p.z, C, F, T, side);
      if (!r.reachable) throw new Error('should be reachable');
      const back = K.legFK(r.q0, r.q1, r.q2, C, F, T, side);
      approx(back.x, p.x, 1e-4, 'x'); approx(back.y, p.y, 1e-4, 'y'); approx(back.z, p.z, 1e-4, 'z');
    }
  }
});

// ── Task 5 tests: ackermann + mixer
test('ackermann straight = zero angles', () => {
  const a = K.ackermann(Infinity, 300, 180);
  approxArr([a.fl, a.fr, a.rl, a.rr], [0, 0, 0, 0]);
});

test('ackermann left turn: inner (left) wheel steers more', () => {
  const a = K.ackermann(400, 300, 180);
  if (!(a.fl > a.fr && a.fl > 0)) throw new Error(`fl=${a.fl} fr=${a.fr}`);
  approx(a.rl, -a.fl, 1e-9); approx(a.rr, -a.fr, 1e-9);
});

test('quadMix pure thrust = equal motors', () => {
  const m = K.quadMix(0.6, 0, 0, 0);
  approxArr(m, [0.6, 0.6, 0.6, 0.6]);
});

test('quadMix roll right: left motors faster', () => {
  const [fr, fl, bl, br] = K.quadMix(0.5, 1, 0, 0);
  if (!(fl > fr && bl > br)) throw new Error('roll mixing wrong');
});

test('quadMix pitch forward: rear motors faster', () => {
  const [fr, fl, bl, br] = K.quadMix(0.5, 0, 1, 0);
  if (!(bl > fl && br > fr)) throw new Error('pitch mixing wrong');
});

test('quadMix yaw cw: CCW props faster', () => {
  const [fr, fl, bl, br] = K.quadMix(0.5, 0, 0, 1);
  if (!(fr > fl && bl > br)) throw new Error('yaw mixing wrong');
});

test('quadMix clamps 0..1', () => {
  K.quadMix(1, 1, 1, 1).forEach(v => { if (v < 0 || v > 1) throw new Error('unclamped'); });
});

// ── Task 6 tests: robots.js arm config consistency
test('arm config has 7 joints (6 revolute + gripper)', () => {
  if (ROBOTS.arm.joints.length !== 7) throw new Error(`got ${ROBOTS.arm.joints.length}`);
  if (ROBOTS.arm.jointNames.length !== 7) throw new Error('names mismatch');
  if (ROBOTS.arm.jointLimits.length !== 7) throw new Error('limits mismatch');
});

test('arm config DH rows match test chain at zero pose', () => {
  const p = ROBOTS.arm.params;
  const chain = new K.DHChain((q) => armDHRows(q, p));
  const H = 20 + p.l1 + p.l2 + p.l3 + p.l4 + ARM_FLANGE;
  approxArr(chain.eePos([0,0,0,0,0,0]), [0, 0, H], 1e-6);
});

test('armFK returns world coords (Y up)', () => {
  const p = ROBOTS.arm.params;
  const f = ROBOTS.arm.fk([0,0,0,0,0,0,18], p);
  approx(f.x, 0, 1e-6); approx(f.z, 0, 1e-6);
  approx(f.y, 20 + p.l1 + p.l2 + p.l3 + p.l4 + ARM_FLANGE, 1e-6);
});

test('armFK bent pose matches independent planar geometry', () => {
  // pitch joints only => chain stays in the world XY plane; derive EE from
  // cumulative link angles measured from vertical (+ toward -x), not from DH
  const p = ROBOTS.arm.params;
  const q1 = 30 * K.DEG, q2 = -60 * K.DEG, q5 = 45 * K.DEG;
  const f = ROBOTS.arm.fk([0, q1, q2, 0, 0, q5, 18], p);
  const a1 = q1, a2 = q1 + q2, a3 = a2, a4 = a2 + q5; // l2, l3, l4, flange angles
  const x = -(p.l2 * Math.sin(a1) + p.l3 * Math.sin(a2) + p.l4 * Math.sin(a3) + ARM_FLANGE * Math.sin(a4));
  const y = 20 + p.l1 + p.l2 * Math.cos(a1) + p.l3 * Math.cos(a2) + p.l4 * Math.cos(a3) + ARM_FLANGE * Math.cos(a4);
  approx(f.x, x, 1e-6, 'x'); approx(f.y, y, 1e-6, 'y'); approx(f.z, 0, 1e-6, 'z');
});

// ── Task 7 tests: per-robot kinematics configs
test('every robot has a kinematics block', () => {
  for (const [key, cfg] of Object.entries(ROBOTS))
    if (!cfg.kinematics) throw new Error(`${key} missing kinematics`);
});

test('scara config FK matches analytical scaraFK', () => {
  const p = ROBOTS.scara.params;
  const f = ROBOTS.scara.fk([0.4, -0.6, 50, 0], p);
  const a = K.scaraFK(0.4, -0.6, p.l1, p.l2);
  approx(f.x, a.x, 1e-6); approx(f.z, a.z, 1e-6);
  approx(f.y, p.pedestalH - 50, 1e-6);
});

test('dexarm chain FK: zero pose hands out in T-pose', () => {
  const p = ROBOTS.dexarm.params;
  const right = dexArmChainFK([0, 0, 0], p, 1);
  const left = dexArmChainFK([0, 0, 0], p, -1);
  // T-pose: arm extends outward along ±X at shoulder height
  const reach = p.upperArmLen + 8 + p.forearmLen + 20 + p.palmLen / 2;
  approx(right[0], 58 + reach, 1e-6, 'right x');
  approx(left[0], -(58 + reach), 1e-6, 'left x');
  approx(right[1], 150 + 180 * 0.82, 1e-6, 'shoulder height');
});

test('dexarm numeric IK reaches forward target', () => {
  const p = ROBOTS.dexarm.params;
  const target = dexArmChainFK([0.3, 0.5, 1.0], p, 1);
  const r = K.solvePositionIK((q) => dexArmChainFK(q, p, 1), [0, 0, 0.5], target);
  if (!r.converged) throw new Error(`posErr=${r.posErr}`);
});

// ── summary (keep at end of file; later tasks insert tests ABOVE this block)
const summary = `TESTS: ${passed} passed, ${failed} failed`;
console.log(summary);
failures.forEach(f => console.log('  FAIL', f));
const banner = document.createElement('div');
banner.id = 'test-banner';
banner.style.cssText = `position:fixed;top:8px;left:50%;transform:translateX(-50%);z-index:9999;
  padding:8px 18px;border-radius:8px;font:600 14px monospace;color:#fff;
  background:${failed ? '#c0392b' : '#27ae60'}`;
banner.textContent = summary;
document.body.appendChild(banner);
