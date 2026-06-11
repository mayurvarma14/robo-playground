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
