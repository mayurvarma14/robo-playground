/**
 * tests.js — browser test harness. Load app with ?test=1 to run.
 * Results go to console and a fixed banner.
 */
import * as K from './kinematics.js';

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
  if (!(wGood > wSing)) throw new Error(`expected wGood ${wGood} > wSing ${wSing}`);
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
