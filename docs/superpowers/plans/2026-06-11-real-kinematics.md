# Real Kinematics Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace ad-hoc joint control and 2-link IK with industry-correct kinematics (DH/FK/DLS-IK, analytical solvers, Ackermann, motor mixing) across all 7 robots, with an IK target gizmo and an expert mode exposing the math.

**Architecture:** New pure-math module `js/kinematics.js` (no dependencies — own 4×4 matrix code, so it is readable teaching material and testable without tooling). Robot definitions in `js/robots.js` gain `kinematics` config blocks. New `js/ik-control.js` wires per-robot IK UI (TransformControls gizmo, sticks, steering). New `js/expert-panel.js` renders DH tables / pose / manipulability. Browser-run test harness `js/tests.js` activated by `?test=1`.

**Tech Stack:** Vanilla ES modules, Three.js 0.160 via CDN importmap (TransformControls addon), no build step.

**Spec:** `docs/superpowers/specs/2026-06-11-real-kinematics-design.md`

---

## Conventions used throughout (READ FIRST)

**DH frame vs world frame.** The DH math uses the textbook convention: joint axes along Z, base Z up. Three.js world is Y-up. Mapping (fixed, used everywhere):

- DH `(x, y, z)` → world `(x, z, -y)`
- world `(x, y, z)` → DH `(x, -z, y)`

**Standard DH transform:** `T_i = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)`.

**6-DOF arm DH table** (derived against the mesh hierarchy; `q` = joints array radians, `p` = params; `FLANGE = 30` constant):

| i | θ        | d       | a   | α    | matches mesh rotation        |
|---|----------|---------|-----|------|------------------------------|
| 1 | q0       | p.l1+20 | 0   | π/2  | `turret.rotation.y = q0`     |
| 2 | q1 + π/2 | 0       | p.l2| 0    | `upperGroup.rotation.z = q1` |
| 3 | q2       | 0       | p.l3| 0    | `forearmGroup.rotation.z = q2` |
| 4 | q3 + π/2 | 0       | 0   | π/2  | `wristGroup.rotation.z = q3` |
| 5 | q4       | p.l4    | 0   | -π/2 | `rollGroup.rotation.y = q4` (NEW) |
| 6 | q5 − π/2 | 0       | FLANGE | 0 | `wrist2Group.rotation.z = q5` (NEW) |

(The flange is along tool X after the final RotZ, hence `a6 = FLANGE`, not `d6`.) **Zero-pose check (the test anchor):** all q = 0 → EE in DH frame = `(0, 0, 20 + l1 + l2 + l3 + l4 + 30)`, i.e. straight up. Second anchor: q1 = φ, others 0 → EE_DH = `(-(l2+l3+l4+30)·sin φ, 0, 20 + l1 + (l2+l3+l4+30)·cos φ)` (arm tilts toward −X for positive φ, matching mesh `rotation.z`).

**UR5e anchor data** (Universal Robots published spec, half-scale to fit scene; implementer: verify against ur.com spec sheet via WebSearch before Task 6 commit):
- DH link values (full scale, mm): d1=162.5, a2=425, a3=392.2, d4=133.3, d5=99.7, d6=99.6
- Half-scale defaults used here: l1=81, l2=213, l3=196, l4=67 (FLANGE fixed 30)
- Joint range all 6 joints: ±360°; max joint speed: 180°/s each → stored as `speeds` metadata
- Payload 5 kg, reach 850 mm — display-only facts for expert panel header

**Test harness:** browser-run. Serve with `npx serve . -l 8080` (run in background once), open `http://localhost:8080/?test=1`, read console / page banner. Use chrome-devtools MCP (`new_page` → `navigate_page` → `list_console_messages`) to verify. Expected line format: `TESTS: N passed, 0 failed`.

---

### Task 1: Test harness + math primitives

**Files:**
- Create: `js/kinematics.js`
- Create: `js/tests.js`
- Modify: `js/main.js` (test-mode dynamic import, top of file)

- [ ] **Step 1: Create `js/kinematics.js` with matrix primitives**

```js
/**
 * kinematics.js — pure math. No Three.js, no DOM.
 * 4x4 matrices are row-major nested arrays: M[row][col].
 */
export const DEG = Math.PI / 180;

export function matIdentity() {
  return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
}

export function matMul(A, B) {
  const C = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  return C;
}

export function matPoint(A, p) {
  return [
    A[0][0]*p[0] + A[0][1]*p[1] + A[0][2]*p[2] + A[0][3],
    A[1][0]*p[0] + A[1][1]*p[1] + A[1][2]*p[2] + A[1][3],
    A[2][0]*p[0] + A[2][1]*p[1] + A[2][2]*p[2] + A[2][3],
  ];
}

export function matTrans(x, y, z) {
  return [[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]];
}

export function matRotX(t) {
  const c = Math.cos(t), s = Math.sin(t);
  return [[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]];
}

export function matRotY(t) {
  const c = Math.cos(t), s = Math.sin(t);
  return [[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]];
}

export function matRotZ(t) {
  const c = Math.cos(t), s = Math.sin(t);
  return [[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]];
}

// Standard DH: RotZ(theta) · TransZ(d) · TransX(a) · RotX(alpha)
export function dhMatrix(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    [ct, -st*ca,  st*sa, a*ct],
    [st,  ct*ca, -ct*sa, a*st],
    [0,   sa,     ca,    d   ],
    [0,   0,      0,     1   ],
  ];
}

// Solve A·x = b (n×n) by Gaussian elimination with partial pivoting.
// Returns x, or null if singular.
export function solveLinear(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++)
      if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-12) return null;
    [M[col], M[piv]] = [M[piv], M[col]];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col] / M[col][col];
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}

// DH-frame (Z-up) <-> Three.js world (Y-up)
export function dhToWorld([x, y, z]) { return [x, z, -y]; }
export function worldToDH([x, y, z]) { return [x, -z, y]; }
```

- [ ] **Step 2: Create `js/tests.js` with harness + primitive tests**

```js
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
```

- [ ] **Step 3: Wire harness into `js/main.js`** — add immediately after the existing imports:

```js
if (new URLSearchParams(location.search).has('test')) import('./tests.js');
```

- [ ] **Step 4: Run tests**

Run once in background: `npx serve . -l 8080`
Then with chrome-devtools MCP: navigate to `http://localhost:8080/?test=1`, read console.
Expected: `TESTS: 8 passed, 0 failed` and green banner.

- [ ] **Step 5: Commit**

```bash
git add js/kinematics.js js/tests.js js/main.js
git commit -m "feat: add pure-math kinematics primitives and browser test harness"
```

---

### Task 2: DHChain — FK, numeric Jacobian, manipulability

**Files:**
- Modify: `js/kinematics.js` (append)
- Modify: `js/tests.js` (insert tests above summary block)

- [ ] **Step 1: Append to `js/kinematics.js`**

```js
// ─────────────────────────────────────────────────────────────
// DH CHAIN — FK, Jacobian, manipulability, DLS IK
// ─────────────────────────────────────────────────────────────

/**
 * rowsFn(q) → array of [theta, d, a, alpha] (radians/units), one per joint.
 * limits: optional [{min, max}] radians, same length as q.
 */
export class DHChain {
  constructor(rowsFn, limits = null) {
    this.rowsFn = rowsFn;
    this.limits = limits;
  }

  fk(q) {
    let T = matIdentity();
    const frames = [];
    for (const [theta, d, a, alpha] of this.rowsFn(q)) {
      T = matMul(T, dhMatrix(theta, d, a, alpha));
      frames.push(T);
    }
    return { frames, ee: T };
  }

  eePos(q) {
    const { ee } = this.fk(q);
    return [ee[0][3], ee[1][3], ee[2][3]];
  }

  // 6×n numeric Jacobian: rows = [vx vy vz wx wy wz] per unit joint velocity
  jacobian(q, eps = 1e-5) {
    const n = q.length;
    const T0 = this.fk(q).ee;
    const J = Array.from({ length: 6 }, () => new Array(n).fill(0));
    for (let i = 0; i < n; i++) {
      const qp = q.slice();
      qp[i] += eps;
      const T1 = this.fk(qp).ee;
      J[0][i] = (T1[0][3] - T0[0][3]) / eps;
      J[1][i] = (T1[1][3] - T0[1][3]) / eps;
      J[2][i] = (T1[2][3] - T0[2][3]) / eps;
      // angular velocity ≈ vee(R1·R0ᵀ skew part)/eps
      const R = [[0,0,0],[0,0,0],[0,0,0]];
      for (let r = 0; r < 3; r++)
        for (let c = 0; c < 3; c++) {
          let s = 0;
          for (let k = 0; k < 3; k++) s += T1[r][k] * T0[c][k];
          R[r][c] = s;
        }
      J[3][i] = (R[2][1] - R[1][2]) / (2 * eps);
      J[4][i] = (R[0][2] - R[2][0]) / (2 * eps);
      J[5][i] = (R[1][0] - R[0][1]) / (2 * eps);
    }
    return J;
  }

  // Yoshikawa manipulability w = sqrt(det(J·Jᵀ)). Near 0 ⇒ singularity.
  manipulability(q) {
    const J = this.jacobian(q);
    const m = 6;
    const A = Array.from({ length: m }, () => new Array(m).fill(0));
    for (let i = 0; i < m; i++)
      for (let j = 0; j < m; j++) {
        let s = 0;
        for (let k = 0; k < J[0].length; k++) s += J[i][k] * J[j][k];
        A[i][j] = s;
      }
    // determinant via elimination
    let det = 1;
    const M = A.map(r => r.slice());
    for (let col = 0; col < m; col++) {
      let piv = col;
      for (let r = col + 1; r < m; r++)
        if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
      if (Math.abs(M[piv][col]) < 1e-12) return 0;
      if (piv !== col) { [M[col], M[piv]] = [M[piv], M[col]]; det = -det; }
      det *= M[col][col];
      for (let r = col + 1; r < m; r++) {
        const f = M[r][col] / M[col][col];
        for (let c = col; c < m; c++) M[r][c] -= f * M[col][c];
      }
    }
    return Math.sqrt(Math.max(0, det));
  }
}
```

- [ ] **Step 2: Add tests** (insert above summary block in `js/tests.js`)

```js
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
```

- [ ] **Step 3: Run tests** — reload `http://localhost:8080/?test=1`. Expected: `TESTS: 12 passed, 0 failed`.

- [ ] **Step 4: Commit**

```bash
git add js/kinematics.js js/tests.js
git commit -m "feat: add DHChain forward kinematics, numeric Jacobian, manipulability"
```

---

### Task 3: DLS IK — full-pose and generic position-only

**Files:**
- Modify: `js/kinematics.js` (append + add methods)
- Modify: `js/tests.js` (insert tests above summary block)

- [ ] **Step 1: Append helpers + `solveIK` method to `js/kinematics.js`**

Add free function:

```js
// Damped-least-squares step: dq = Jᵀ (J·Jᵀ + λ²I)⁻¹ e
export function dlsStep(J, e, lambda) {
  const m = J.length, n = J[0].length;
  const A = Array.from({ length: m }, () => new Array(m).fill(0));
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < m; j++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += J[i][k] * J[j][k];
      A[i][j] = s;
    }
    A[i][i] += lambda * lambda;
  }
  const y = solveLinear(A, e);
  if (!y) return new Array(n).fill(0);
  const dq = new Array(n).fill(0);
  for (let k = 0; k < n; k++)
    for (let i = 0; i < m; i++) dq[k] += J[i][k] * y[i];
  return dq;
}
```

Add methods inside `DHChain`:

```js
  // 6-vector pose error. targetRot: 3×3 row-major or null (position-only).
  // rotWeight balances radians against mm so one error norm drives both.
  poseError(T, targetPos, targetRot, rotWeight = 60) {
    const e = [
      targetPos[0] - T[0][3],
      targetPos[1] - T[1][3],
      targetPos[2] - T[2][3],
      0, 0, 0,
    ];
    if (targetRot) {
      // Re = Rt · Rᵀ ; axis-angle of Re
      const Re = [[0,0,0],[0,0,0],[0,0,0]];
      for (let i = 0; i < 3; i++)
        for (let j = 0; j < 3; j++) {
          let s = 0;
          for (let k = 0; k < 3; k++) s += targetRot[i][k] * T[j][k];
          Re[i][j] = s;
        }
      const tr = Re[0][0] + Re[1][1] + Re[2][2];
      const angle = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
      if (angle > 1e-9) {
        const f = angle / (2 * Math.sin(angle)) * rotWeight;
        e[3] = (Re[2][1] - Re[1][2]) * f;
        e[4] = (Re[0][2] - Re[2][0]) * f;
        e[5] = (Re[1][0] - Re[0][1]) * f;
      }
    }
    return e;
  }

  /**
   * DLS IK. targetPos: [x,y,z] DH frame. targetRot: 3×3 or null.
   * Returns { q, converged, iterations, posErr }.
   * Never NaNs: damping keeps steps bounded near singularities.
   */
  solveIK(qStart, targetPos, targetRot = null, opts = {}) {
    const { maxIter = 100, tolPos = 0.05, tolRot = 0.01, lambda = 6, rotWeight = 60 } = opts;
    let q = qStart.slice();
    let best = { q: q.slice(), posErr: Infinity };
    for (let iter = 0; iter < maxIter; iter++) {
      const { ee } = this.fk(q);
      const e = this.poseError(ee, targetPos, targetRot, rotWeight);
      const posErr = Math.hypot(e[0], e[1], e[2]);
      const rotErr = Math.hypot(e[3], e[4], e[5]) / rotWeight;
      if (posErr < best.posErr) best = { q: q.slice(), posErr };
      if (posErr < tolPos && (!targetRot || rotErr < tolRot))
        return { q, converged: true, iterations: iter, posErr };
      const J = this.jacobian(q);
      const dq = dlsStep(J, e, lambda);
      for (let i = 0; i < q.length; i++) {
        q[i] += dq[i];
        if (this.limits) {
          const L = this.limits[i];
          q[i] = Math.max(L.min, Math.min(L.max, q[i]));
        }
      }
    }
    return { q: best.q, converged: false, iterations: opts.maxIter ?? 100, posErr: best.posErr };
  }
```

Add free function for non-DH chains (dexarm/humanoid generic chains):

```js
/**
 * Generic position-only DLS IK for arbitrary FK functions.
 * fkFn(q) → [x,y,z] world. limits: [{min,max}] or null.
 */
export function solvePositionIK(fkFn, qStart, target, limits = null, opts = {}) {
  const { maxIter = 80, tol = 0.1, lambda = 6, eps = 1e-5 } = opts;
  let q = qStart.slice();
  let best = { q: q.slice(), posErr: Infinity };
  for (let iter = 0; iter < maxIter; iter++) {
    const p = fkFn(q);
    const e = [target[0] - p[0], target[1] - p[1], target[2] - p[2]];
    const posErr = Math.hypot(...e);
    if (posErr < best.posErr) best = { q: q.slice(), posErr };
    if (posErr < tol) return { q, converged: true, iterations: iter, posErr };
    const n = q.length;
    const J = [new Array(n), new Array(n), new Array(n)];
    for (let i = 0; i < n; i++) {
      const qp = q.slice();
      qp[i] += eps;
      const pp = fkFn(qp);
      J[0][i] = (pp[0] - p[0]) / eps;
      J[1][i] = (pp[1] - p[1]) / eps;
      J[2][i] = (pp[2] - p[2]) / eps;
    }
    const dq = dlsStep(J, e, lambda);
    for (let i = 0; i < n; i++) {
      q[i] += dq[i];
      if (limits) q[i] = Math.max(limits[i].min, Math.min(limits[i].max, q[i]));
    }
  }
  return { q: best.q, converged: false, iterations: maxIter, posErr: best.posErr };
}
```

- [ ] **Step 2: Add tests** (above summary block)

```js
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
```

- [ ] **Step 3: Run tests** — reload. Expected: `TESTS: 17 passed, 0 failed`.

- [ ] **Step 4: Commit**

```bash
git add js/kinematics.js js/tests.js
git commit -m "feat: add damped-least-squares IK (full pose + generic position-only)"
```

---

### Task 4: Analytical solvers — two-link, SCARA, quadruped leg

**Files:**
- Modify: `js/kinematics.js` (append)
- Modify: `js/tests.js` (insert tests above summary block)

- [ ] **Step 1: Append to `js/kinematics.js`**

```js
// ─────────────────────────────────────────────────────────────
// ANALYTICAL SOLVERS
// ─────────────────────────────────────────────────────────────

/**
 * Planar 2R IK. Link1 along +x at q1=0; x = l1·c1 + l2·c12, y = l1·s1 + l2·s12.
 * elbow: +1 (elbow-up / positive q2) or -1.
 */
export function twoLinkIK(x, y, l1, l2, elbow = -1) {
  const d2 = x * x + y * y;
  const d = Math.sqrt(d2);
  const reachable = d <= l1 + l2 + 1e-9 && d >= Math.abs(l1 - l2) - 1e-9;
  const c2 = Math.max(-1, Math.min(1, (d2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)));
  const q2 = elbow * Math.acos(c2);
  const q1 = Math.atan2(y, x) - Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2));
  return { q1, q2, reachable };
}

export function twoLinkFK(q1, q2, l1, l2) {
  return {
    x: l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2),
    y: l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2),
  };
}

/**
 * SCARA planar IK in Three.js world XZ plane (joints rotate about world Y).
 * Mesh convention: RotY(q)·(l,0,0) = (l·cos q, 0, −l·sin q) ⇒ planar v = −z.
 */
export function scaraFK(q1, q2, l1, l2) {
  const p = twoLinkFK(q1, q2, l1, l2);
  return { x: p.x, z: -p.y };
}

export function scaraIK(x, z, l1, l2, elbow = 1) {
  return twoLinkIK(x, -z, l1, l2, elbow);
}

/**
 * Quadruped 3-DOF leg IK, leg-frame coordinates (origin at hip yaw axis,
 * Y up, X lateral outward-positive for right legs, Z forward).
 * Mesh chain: RotY(q0) → translate (side·coxa, 0, 0) → RotX(q1) → femur −Y
 *           → RotX(q2) → tibia −Y.
 * side: +1 right legs (x > 0), −1 left legs.
 */
export function legFK(q0, q1, q2, coxa, femur, tibia, side) {
  const c = side * coxa;
  const fy = -(femur * Math.cos(q1) + tibia * Math.cos(q1 + q2));
  const fz = -(femur * Math.sin(q1) + tibia * Math.sin(q1 + q2));
  const c0 = Math.cos(q0), s0 = Math.sin(q0);
  return { x: c * c0 + fz * s0, y: fy, z: -c * s0 + fz * c0 };
}

export function legIK(x, y, z, coxa, femur, tibia, side) {
  const c = side * coxa;
  const R = Math.hypot(x, z);
  const reachableYaw = R >= Math.abs(c) - 1e-9;
  const fzMag = Math.sqrt(Math.max(0, R * R - c * c));
  const fz = (z >= 0 ? 1 : -1) * fzMag;
  const q0 = Math.atan2(fz, c) - Math.atan2(z, x);
  // planar: w = −y (down-positive reach), u = −fz
  const { q1, q2, reachable } = twoLinkIK(-y, -fz, femur, tibia, -1);
  return { q0, q1, q2, reachable: reachable && reachableYaw };
}
```

- [ ] **Step 2: Add tests** (above summary block)

```js
// ── Task 4 tests: analytical solvers
test('twoLink roundtrip both elbows', () => {
  for (const elbow of [1, -1]) {
    const t = twoLinkFKHelper(0.7, elbow * -1.1);
    const r = K.twoLinkIK(t.x, t.y, 100, 80, elbow * -1 > 0 ? 1 : -1);
    const back = K.twoLinkFK(r.q1, r.q2, 100, 80);
    approx(back.x, t.x, 1e-6); approx(back.y, t.y, 1e-6);
  }
  function twoLinkFKHelper(a, b) { return K.twoLinkFK(a, b, 100, 80); }
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
```

- [ ] **Step 3: Run tests** — reload. Expected: `TESTS: 22 passed, 0 failed`. If `leg IK roundtrip` fails on a sign, the q0 branch (`z >= 0` choice of `fz`) does not match that quadrant — debug with printed intermediate values until roundtrip is exact; do NOT loosen tolerance.

- [ ] **Step 4: Commit**

```bash
git add js/kinematics.js js/tests.js
git commit -m "feat: add analytical IK solvers (two-link, SCARA, quadruped leg)"
```

---

### Task 5: Ackermann steering + quad motor mixer

**Files:**
- Modify: `js/kinematics.js` (append)
- Modify: `js/tests.js` (insert tests above summary block)

- [ ] **Step 1: Append to `js/kinematics.js`**

```js
// ─────────────────────────────────────────────────────────────
// MOBILE ROBOTS
// ─────────────────────────────────────────────────────────────

/**
 * 4-wheel-steer Ackermann (Perseverance-style: 4 corner wheels steer,
 * middle pair fixed). radius: signed turn radius to vehicle centre,
 * +ve = left turn, Infinity = straight. wheelbase = front↔rear corner
 * distance, track = left↔right distance. Returns radians per corner.
 */
export function ackermann(radius, wheelbase, track) {
  if (!Number.isFinite(radius) || Math.abs(radius) < track / 2 + 1)
    return { fl: 0, fr: 0, rl: 0, rr: 0 };
  const h = wheelbase / 2;
  const fl = Math.atan(h / (radius - track / 2));
  const fr = Math.atan(h / (radius + track / 2));
  return { fl, fr, rl: -fl, rr: -fr };
}

/**
 * X-configuration quad mixer. Inputs: thrust 0..1, roll/pitch/yaw −1..1.
 * Conventions: +roll = roll right, +pitch = nose down (forward),
 * +yaw = clockwise from above. Props: FR & BL spin CCW, FL & BR CW.
 * Returns motor outputs [fr, fl, bl, br] clamped 0..1.
 */
export function quadMix(thrust, roll, pitch, yaw) {
  const k = 0.25; // control authority per axis
  const m = [
    thrust - k * roll - k * pitch + k * yaw, // FR
    thrust + k * roll - k * pitch - k * yaw, // FL
    thrust + k * roll + k * pitch + k * yaw, // BL
    thrust - k * roll + k * pitch - k * yaw, // BR
  ];
  return m.map(v => Math.max(0, Math.min(1, v)));
}
```

- [ ] **Step 2: Add tests** (above summary block)

```js
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
```

- [ ] **Step 3: Run tests** — reload. Expected: `TESTS: 29 passed, 0 failed`.

- [ ] **Step 4: Commit**

```bash
git add js/kinematics.js js/tests.js
git commit -m "feat: add Ackermann steering and quadcopter motor mixer"
```

---

### Task 6: Upgrade arm to true 6-DOF with UR5e-anchored kinematics config

The current "6-DOF" arm has only 4 revolute joints + gripper. Add wrist roll (J5) and wrist 2 pitch (J6) to the mesh, switch defaults to UR5e half-scale proportions, and attach the DH kinematics config.

**Files:**
- Modify: `js/robots.js` (buildArm wrist section ~lines 192–231, armFK ~lines 234–244, ROBOTS.arm config ~lines 1018–1039)
- Modify: `js/tests.js` (insert tests above summary block)

- [ ] **Step 1: Replace the WRIST + END EFFECTOR section of `buildArm`**

Replace everything from `// ── WRIST` down to (excluding) `return root;` with:

```js
  // ── WRIST 1 (pitch)
  const wristGroup = new THREE.Group();
  wristGroup.position.y = l3;
  wristGroup.rotation.z = joints[3];
  forearmGroup.add(wristGroup);

  const wristActuator = cyl(12, 12, 20, 20, MAT.chrome, 'Wrist 1 Actuator');
  wristActuator.rotation.z = Math.PI / 2;
  wristActuator.position.y = 2;
  wristGroup.add(wristActuator);

  // ── WRIST 2 (roll about the link axis)
  const rollGroup = new THREE.Group();
  rollGroup.rotation.y = joints[4];
  wristGroup.add(rollGroup);

  const wristLink = roundedLink(l4, 20, 16, MAT.aluminium, 'Wrist Link');
  wristLink.position.y = l4 / 2;
  rollGroup.add(wristLink);

  const rollRing = bearing(14, 6);
  rollRing.position.y = l4 * 0.45;
  rollGroup.add(rollRing);

  // ── WRIST 3 (pitch)
  const wrist2Group = new THREE.Group();
  wrist2Group.position.y = l4;
  wrist2Group.rotation.z = joints[5];
  rollGroup.add(wrist2Group);

  const wrist2Actuator = cyl(10, 10, 18, 20, MAT.chrome, 'Wrist 3 Actuator');
  wrist2Actuator.rotation.z = Math.PI / 2;
  wrist2Group.add(wrist2Actuator);

  // ── FLANGE + GRIPPER (FLANGE = 30 must match the DH table)
  const gripGroup = new THREE.Group();
  gripGroup.position.y = ARM_FLANGE;
  wrist2Group.add(gripGroup);

  const gripBase = cyl(14, 14, 18, 20, MAT.blackAnodised, 'Gripper Base');
  gripBase.position.y = -9;
  gripGroup.add(gripBase);

  const clawGap = joints[6] !== undefined ? joints[6] : 18;
  for (let side of [-1, 1]) {
    const claw = new THREE.Group();
    claw.position.set(side * clawGap / 2, 0, 0);

    const finger = box(6, 30, 8, MAT.darkSteel, side > 0 ? 'Gripper Finger Right' : 'Gripper Finger Left');
    finger.position.y = 15;
    claw.add(finger);

    const tip = box(8, 10, 10, MAT.rubber, 'Gripper Tip');
    tip.position.y = 34;
    claw.add(tip);
    gripGroup.add(claw);
  }
```

And add near the top of `js/robots.js` (after imports):

```js
import { DHChain, dhToWorld } from './kinematics.js';

export const ARM_FLANGE = 30;
```

- [ ] **Step 2: Replace `armFK` with DH-driven version**

```js
export function armDHRows(q, p) {
  return [
    [q[0],                p.l1 + 20, 0,    Math.PI / 2],
    [q[1] + Math.PI / 2,  0,         p.l2, 0],
    [q[2],                0,         p.l3, 0],
    [q[3] + Math.PI / 2,  0,         0,    Math.PI / 2],
    [q[4],                p.l4,      0,   -Math.PI / 2],
    [q[5] - Math.PI / 2,  0,         ARM_FLANGE, 0],
  ];
}

// FK for telemetry — world (Y-up) coordinates
export function armFK(joints, params) {
  const chain = new DHChain((q) => armDHRows(q, params));
  const [x, y, z] = dhToWorld(chain.eePos(joints.slice(0, 6)));
  return { x, y, z };
}
```

- [ ] **Step 3: Replace `ROBOTS.arm` config**

```js
  arm: {
    name: '6-DOF Robotic Arm',
    builder: buildArm,
    fk: armFK,
    joints: [0, Math.PI / 6, -Math.PI / 3, -Math.PI / 6, 0, 0, 18],
    jointNames: [
      'Base (J1)', 'Shoulder (J2)', 'Elbow (J3)',
      'Wrist 1 Pitch (J4)', 'Wrist 2 Roll (J5)', 'Wrist 3 Pitch (J6)',
      'Gripper Gap',
    ],
    jointLimits: [
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: 8,    max: 45,  step: 1, isAngle: false },
    ],
    params: { l1: 81, l2: 213, l3: 196, l4: 67 },
    paramDefs: [
      { label: 'Base Height (d1)', key: 'l1', min: 50,  max: 150, step: 1, unit: 'mm' },
      { label: 'Upper Arm (a2)',   key: 'l2', min: 100, max: 280, step: 1, unit: 'mm' },
      { label: 'Forearm (a3)',     key: 'l3', min: 80,  max: 260, step: 1, unit: 'mm' },
      { label: 'Wrist (d5)',       key: 'l4', min: 40,  max: 120, step: 1, unit: 'mm' },
    ],
    ikSupported: true,
    kinematics: {
      type: 'dh',
      dof: 6,
      rows: armDHRows,
      // UR5e anchor (half scale). Verify values against the published UR5e
      // spec sheet before committing this task.
      anchor: 'Universal Robots UR5e (half scale)',
      speeds: [180, 180, 180, 180, 180, 180], // deg/s, UR5e max joint speed
      facts: 'UR5e: 5 kg payload · 850 mm reach · ±360° joints',
    },
  },
```

- [ ] **Step 4: Verify UR5e data online**

Use WebSearch: "UR5e DH parameters site:universal-robots.com" and the UR5e technical spec sheet. Confirm: d1=162.5 mm, a2=425 mm, a3=392.2 mm, d4=133.3 mm, d5=99.7 mm, d6=99.6 mm; joint ranges ±360°; max joint speed 180°/s. If a speed differs per joint (e.g. wrists faster), correct the `speeds` array accordingly.

- [ ] **Step 5: Add tests** (above summary block in `js/tests.js`)

```js
// ── Task 6 tests: robots.js arm config consistency
import { ROBOTS, armDHRows, ARM_FLANGE } from './robots.js';

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
```

Note: `tests.js` now imports `robots.js`, which imports Three.js — fine, harness is browser-only.

- [ ] **Step 6: Run tests + visual check**

Reload `http://localhost:8080/?test=1`. Expected: `TESTS: 32 passed, 0 failed`.
Visual (chrome-devtools): load app without `?test`, select arm, confirm 7 sliders, J5 roll spins wrist link, J6 pitches gripper, robot stands ~600 mm tall, telemetry Y ≈ 607 at zero pose (set all sliders 0).

- [ ] **Step 7: Commit**

```bash
git add js/robots.js js/tests.js
git commit -m "feat: upgrade arm to true 6-DOF with UR5e-anchored DH kinematics"
```

---

### Task 7: Kinematics config blocks for the other 6 robots (+ rover steering mesh)

**Files:**
- Modify: `js/robots.js` (configs; buildRover steering; buildQuadruped bodyGroup; buildDexArm FK export)
- Modify: `js/tests.js` (insert tests above summary block)

- [ ] **Step 1: Rover — steering joints in mesh**

In `buildRover`, replace the wheel loop with steerable corners. The 6 joints become `[spin, flSteer, frSteer, rlSteer, rrSteer, unused]` (spin shared by all wheels; corner steer applied to 4 corner wheels; middle pair fixed):

```js
  // joints: [0]=wheel spin angle, [1]=FL steer, [2]=FR steer, [3]=RL steer, [4]=RR steer
  const steerByIndex = [joints[1], joints[2], 0, 0, joints[3], joints[4]];

  for (let i = 0; i < 6; i++) {
    const [sx, , wz] = wheelPositions[i];
    const wg = new THREE.Group();
    wg.position.set(sx * (chassisW / 2 + wheelW / 2), wheelR, wz);
    wg.rotation.y = steerByIndex[i]; // steer about vertical axis
    wg.name = `WheelGroup${i}`;

    const tyre = cyl(wheelR, wheelR, wheelW, 24, MAT.rubber, `${wheelNames[i]} Tyre`);
    tyre.rotation.z = Math.PI / 2;
    tyre.rotation.y = joints[0]; // spin
    wg.add(tyre);

    const hub = cyl(wheelR * 0.45, wheelR * 0.45, wheelW + 4, 16, MAT.aluminium, `${wheelNames[i]} Hub`);
    hub.rotation.z = Math.PI / 2;
    wg.add(hub);

    for (let s = 0; s < 5; s++) {
      const spoke = box(wheelR * 0.75, 3, 3, MAT.darkSteel);
      spoke.rotation.z = (s / 5) * Math.PI;
      wg.add(spoke);
    }

    root.add(wg);
  }
```

Replace `ROBOTS.rover` joints config:

```js
    joints: [0, 0, 0, 0, 0],
    jointNames: ['Wheel Spin', 'FL Steer', 'FR Steer', 'RL Steer', 'RR Steer'],
    jointLimits: [
      { min: -360, max: 360, step: 5, isAngle: true },
      { min: -50,  max: 50,  step: 1, isAngle: true },
      { min: -50,  max: 50,  step: 1, isAngle: true },
      { min: -50,  max: 50,  step: 1, isAngle: true },
      { min: -50,  max: 50,  step: 1, isAngle: true },
    ],
```

and add to the rover config:

```js
    kinematics: {
      type: 'ackermann',
      // corner-wheel geometry from chassis params
      geometry: (p) => ({ wheelbase: p.chassisL - 50, track: p.chassisW + p.wheelW }),
      anchor: 'NASA Perseverance — 4 corner-wheel steering',
    },
```

- [ ] **Step 2: Quadruped — bodyGroup wrapper + config**

In `buildQuadruped`, wrap body meshes AND leg groups in one `bodyGroup` so a body pose transform moves hips while leg IK keeps feet planted:

```js
  const bodyGroup = new THREE.Group();
  bodyGroup.name = 'BodyGroup';
  root.add(bodyGroup);
  // ...add body, cover, sensorHead, lidar, and each legGroup to bodyGroup
  // instead of root (change the five `root.add(...)` calls accordingly).

  // Optional body pose injected by the IK controller:
  const bp = params._bodyPose;
  if (bp) {
    bodyGroup.position.set(bp.x || 0, bp.y || 0, bp.z || 0);
    bodyGroup.rotation.set(bp.rx || 0, bp.ry || 0, bp.rz || 0);
  }
```

Add to `ROBOTS.quadruped`:

```js
    kinematics: {
      type: 'quad-legs',
      // legs: name, hip offset from body centre (local), side sign for coxa
      legs: (p) => {
        const H = p.femur + p.tibia + 10;
        return [
          { name: 'FR', joint0: 0, hip: [ p.bodyW / 2, H,  p.bodyLen / 2 - 20], side:  1 },
          { name: 'FL', joint0: 3, hip: [-p.bodyW / 2, H,  p.bodyLen / 2 - 20], side: -1 },
          { name: 'BL', joint0: 6, hip: [-p.bodyW / 2, H, -p.bodyLen / 2 + 20], side: -1 },
          { name: 'BR', joint0: 9, hip: [ p.bodyW / 2, H, -p.bodyLen / 2 + 20], side:  1 },
        ];
      },
      footOffset: 2, // foot sphere centre sits 2 below tibia end — add to tibia in IK calls
      anchor: 'Boston Dynamics Spot-class leg geometry',
    },
```

- [ ] **Step 3: SCARA config**

Add to `ROBOTS.scara`:

```js
    ikSupported: true,
    kinematics: {
      type: 'scara',
      // DH for expert panel + FK; q = [q1, q2, zSlide(mm), q4]
      rows: (q, p) => [
        [q[0], p.pedestalH, p.l1, 0],
        [q[1], 0,           p.l2, 0],
        [0,   -q[2],        0,    0],   // prismatic Z slide
        [q[3], 0,           0,    0],
      ],
      prismatic: [2],
      zTravel: 130,
      anchor: 'Epson LS6-B class (600 mm reach family, scaled)',
    },
```

Also set scara `fk`:

```js
import { scaraFK } from './kinematics.js';
// in ROBOTS.scara:
    fk: (joints, params) => {
      const p = scaraFK(joints[0], joints[1], params.l1, params.l2);
      return { x: p.x, y: params.pedestalH - joints[2], z: p.z };
    },
```

- [ ] **Step 4: Humanoid config**

The mesh applies arm joints about Z (frontal-plane swing) and leg joints about X (sagittal). Note `Shoulder Roll` joints (2 and 5) are not applied to the mesh — leave as-is this cycle. Add:

```js
    kinematics: {
      type: 'limbs',
      arm: (p) => ({ upper: 82, fore: 82, shoulderY: 120, shoulderX: p.hipSpacing / 2 + 30 }),
      leg: (p) => ({ thigh: p.thigh, shin: p.shin, hipY: -15, hipX: p.hipSpacing / 2 }),
      rootY: (p) => p.thigh + p.shin + p.footH + 15,
      // mass fractions for ground-projected COM (coarse anthropomorphic split)
      masses: { torso: 0.50, head: 0.07, arm: 0.05, thigh: 0.10, shin: 0.06, foot: 0.015 },
      anchor: 'Optimus-class proportions',
    },
```

- [ ] **Step 5: Dexarm config — pure-math single-arm FK for numeric IK**

Add to `js/robots.js` (uses kinematics.js mat helpers; mirrors `buildSingleArm` transforms exactly — shoulder euler order is Three.js default 'XYZ', i.e. R = RotX·RotY·RotZ):

```js
import { matMul, matRotX, matRotY, matRotZ, matTrans, matPoint } from './kinematics.js';

// World position of one palm centre. q = [shoulderPitch, shoulderYaw, elbowFlex]
export function dexArmChainFK(q, params, side) {
  const torsoH = 180, torsoW = 80;
  const mount = matTrans(side * (torsoW / 2 + 18), 150 + torsoH * 0.82, 0);
  // shoulderGroup: rotation.z = q0*side, rotation.y = q1*side (euler XYZ => RX·RY·RZ, RX=0)
  const shoulder = matMul(matRotY(q[1] * side), matRotZ(q[0] * side));
  const tPose = matRotZ(side * Math.PI / 2);
  const upper = matTrans(0, -params.upperArmLen, 0);
  const elbow = matRotZ(-q[2] * side);
  const fore = matTrans(0, -8 - params.forearmLen, 0);
  const palm = matTrans(0, -20 - params.palmLen / 2, 0);
  let T = matMul(mount, shoulder);
  T = matMul(T, tPose);
  T = matMul(T, upper);
  T = matMul(T, elbow);
  T = matMul(T, fore);
  T = matMul(T, palm);
  return matPoint(T, [0, 0, 0]);
}
```

Add to `ROBOTS.dexarm`:

```js
    ikSupported: true,
    kinematics: {
      type: 'numeric-arms',
      fkFn: dexArmChainFK,
      ikJoints: [0, 1, 2], // shoulder pitch, shoulder yaw, elbow drive position IK
      anchor: 'Shadow Hand finger ranges; bimanual torso layout',
    },
```

- [ ] **Step 6: Drone config**

```js
    kinematics: {
      type: 'mixer',
      anchor: 'X-quad convention (FR/BL CCW, FL/BR CW)',
    },
```

- [ ] **Step 7: Add tests** (above summary block)

```js
// ── Task 7 tests: per-robot kinematics configs
import { dexArmChainFK } from './robots.js';

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
```

If `dexarm chain FK` T-pose test fails, the shoulder euler composition order is wrong — check `Euler` default order 'XYZ' composes as RotX·RotY·RotZ in Three.js and adjust `matMul(matRotY, matRotZ)` ordering to match (`new THREE.Euler(0, b, c, 'XYZ')` equals RY(b)·RZ(c) when x=0).

- [ ] **Step 8: Run tests + visual check**

Reload `?test=1`. Expected: `TESTS: 36 passed, 0 failed`.
Visual: rover steer sliders pivot corner wheels about vertical axis; quadruped looks unchanged.

- [ ] **Step 9: Commit**

```bash
git add js/robots.js js/tests.js
git commit -m "feat: add kinematics configs for all robots, steerable rover wheels"
```

---

### Task 8: Viewport — gizmo, frame triads, EE trace

**Files:**
- Modify: `js/viewport.js`

No unit tests (rendering); verified visually in Tasks 9–11.

- [ ] **Step 1: Add TransformControls + helper groups to `js/viewport.js`**

Import at top:

```js
import { TransformControls } from 'three/addons/controls/TransformControls.js';
```

In `init()`, after `this.controls.update();`:

```js
    // IK target gizmo
    this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
    this.gizmo.size = 0.8;
    this.gizmo.addEventListener('dragging-changed', (e) => {
      this.controls.enabled = !e.value;
    });
    this.scene.add(this.gizmo);

    // joint frame triads + other kinematic overlays
    this.kinHelpers = new THREE.Group();
    this.scene.add(this.kinHelpers);

    // end-effector trace (preallocated line buffer)
    this.traceMax = 500;
    this.traceCount = 0;
    const traceGeo = new THREE.BufferGeometry();
    traceGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(this.traceMax * 3), 3));
    traceGeo.setDrawRange(0, 0);
    this.traceLine = new THREE.Line(traceGeo, new THREE.LineBasicMaterial({ color: 0x22d3ee }));
    this.traceLine.frustumCulled = false;
    this.scene.add(this.traceLine);
```

- [ ] **Step 2: Add public methods to the `Viewport` class**

```js
  attachGizmo(obj, mode = 'translate') {
    this.gizmo.attach(obj);
    this.gizmo.setMode(mode);
  }
  detachGizmo() { this.gizmo.detach(); }
  setGizmoMode(mode) { this.gizmo.setMode(mode); }

  clearKinHelpers() {
    while (this.kinHelpers.children.length) this.kinHelpers.remove(this.kinHelpers.children[0]);
  }
  addKinHelper(obj) { this.kinHelpers.add(obj); }

  // small RGB axes triad for joint frames
  makeTriad(size = 22) {
    const triad = new THREE.Group();
    const dirs = [
      [[1, 0, 0], 0xef4444],
      [[0, 1, 0], 0x22c55e],
      [[0, 0, 1], 0x3b82f6],
    ];
    for (const [d, color] of dirs) {
      const geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0),
        new THREE.Vector3(d[0] * size, d[1] * size, d[2] * size),
      ]);
      triad.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color })));
    }
    return triad;
  }

  pushTracePoint(x, y, z) {
    const attr = this.traceLine.geometry.attributes.position;
    if (this.traceCount >= this.traceMax) {
      attr.array.copyWithin(0, 3);
      this.traceCount = this.traceMax - 1;
    }
    attr.array[this.traceCount * 3] = x;
    attr.array[this.traceCount * 3 + 1] = y;
    attr.array[this.traceCount * 3 + 2] = z;
    this.traceCount++;
    attr.needsUpdate = true;
    this.traceLine.geometry.setDrawRange(0, this.traceCount);
  }

  clearTrace() {
    this.traceCount = 0;
    this.traceLine.geometry.setDrawRange(0, 0);
  }
```

- [ ] **Step 3: Guard wireframe toggle** — `setWireframe` traverses the whole scene; exclude helper lines (LineBasicMaterial has no meaningful wireframe but gizmo internals do). Change the traverse callback:

```js
    this.scene.traverse(obj => {
      if (obj.isMesh && obj.material && !this._isHelper(obj)) obj.material.wireframe = v;
    });
```

with:

```js
  _isHelper(obj) {
    let o = obj;
    while (o) {
      if (o === this.gizmo || o === this.kinHelpers || o === this.traceLine) return true;
      o = o.parent;
    }
    return false;
  }
```

- [ ] **Step 4: Smoke check** — load app, no console errors, robots render as before.

- [ ] **Step 5: Commit**

```bash
git add js/viewport.js
git commit -m "feat: add IK gizmo, frame triads, and EE trace to viewport"
```

---

### Task 9: IK controller — arm, SCARA, dexarm target IK with gizmo

**Files:**
- Create: `js/ik-control.js`
- Modify: `index.html` (replace IK section markup, ~lines 184–211)
- Modify: `js/main.js` (delete old IK handler ~lines 217–263; wire controller; add `syncJointInputs`)
- Modify: `css/styles.css` (append styles)

- [ ] **Step 1: Replace the Inverse Kinematics section in `index.html`**

Replace the whole `<div class="pane-section">` containing the IK grid with:

```html
            <div class="pane-section" id="ik-section">
              <div class="section-header">
                <span class="section-title">Inverse Kinematics</span>
                <button class="mini-btn" id="btn-ik-mode" style="display:none">Rotate</button>
              </div>
              <div id="ik-controls"></div>
              <div class="ik-warning" id="ik-warning" style="display:none">
                <i class="fa-solid fa-triangle-exclamation"></i> <span>Target out of workspace</span>
              </div>
            </div>
```

- [ ] **Step 2: Create `js/ik-control.js`**

```js
/**
 * ik-control.js — per-robot IK interaction: target gizmo, numeric fields,
 * solver wiring. Owns the IK target objects in the scene.
 */
import * as THREE from 'three';
import {
  DHChain, solvePositionIK, scaraIK, legIK, ackermann, quadMix,
  worldToDH, dhToWorld, DEG,
} from './kinematics.js';

// DH <-> world basis change for orientations: C maps DH coords to world
const C = new THREE.Matrix4().set(
  1, 0, 0, 0,
  0, 0, 1, 0,
  0, -1, 0, 0,
  0, 0, 0, 1,
);
const CT = C.clone().transpose();

export class IKController {
  /**
   * deps: { viewport, robots, getActiveKey, onJointsChanged }
   * onJointsChanged(): re-applies cfg.joints to the mesh + syncs sliders.
   */
  constructor(deps) {
    this.deps = deps;
    this.target = new THREE.Mesh(
      new THREE.SphereGeometry(7, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0x22d3ee, transparent: true, opacity: 0.85 })
    );
    this.target.visible = false;
    deps.viewport.addObject(this.target);
    this.solvePending = false;
    deps.viewport.registerCallback(() => this._tick());

    deps.viewport.gizmo.addEventListener('objectChange', () => { this.solvePending = true; });

    document.getElementById('btn-ik-mode').addEventListener('click', () => {
      const btn = document.getElementById('btn-ik-mode');
      const next = btn.textContent === 'Rotate' ? 'rotate' : 'translate';
      deps.viewport.setGizmoMode(next);
      btn.textContent = next === 'rotate' ? 'Translate' : 'Rotate';
    });
  }

  // Called on robot switch. Builds the per-robot control UI.
  activate(key) {
    const { viewport, robots } = this.deps;
    const cfg = robots[key];
    const host = document.getElementById('ik-controls');
    host.innerHTML = '';
    viewport.detachGizmo();
    viewport.clearTrace();
    this.target.visible = false;
    this._setWarning(false);
    document.getElementById('btn-ik-mode').style.display = 'none';
    this.mode = cfg.kinematics?.type ?? null;

    if (this.mode === 'dh') this._activateArm(cfg, host);
    else if (this.mode === 'scara') this._activateScara(cfg, host);
    else if (this.mode === 'numeric-arms') this._activateDexarm(cfg, host);
    else host.innerHTML = '<p class="help-text">IK target control for this robot is on the dedicated panel below.</p>';
  }

  _activateArm(cfg, host) {
    host.innerHTML = `
      <div class="ik-grid" id="ik-fields"></div>
      <p class="help-text">Drag the gizmo or type a pose. DLS solver tracks live.</p>`;
    this._buildPoseFields(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']);
    document.getElementById('btn-ik-mode').style.display = '';

    // place target at current EE
    const f = cfg.fk(cfg.joints, cfg.params);
    this.target.position.set(f.x, f.y, f.z);
    this.target.rotation.set(0, 0, 0);
    this.target.visible = true;
    this.deps.viewport.attachGizmo(this.target);
    this._writeFieldsFromTarget();

    this.solver = (t) => {
      const chain = new DHChain((q) => cfg.kinematics.rows(q, cfg.params));
      const dhPos = worldToDH([t.position.x, t.position.y, t.position.z]);
      // orientation: R_dh = Cᵀ · R_world · C
      const Rw = new THREE.Matrix4().makeRotationFromQuaternion(t.quaternion);
      const Rdh = CT.clone().multiply(Rw).multiply(C);
      const e = Rdh.elements; // column-major
      const targetRot = [
        [e[0], e[4], e[8]],
        [e[1], e[5], e[9]],
        [e[2], e[6], e[10]],
      ];
      const r = chain.solveIK(cfg.joints.slice(0, 6), dhPos, targetRot);
      for (let i = 0; i < 6; i++) cfg.joints[i] = r.q[i];
      return r.converged;
    };
  }

  _activateScara(cfg, host) {
    host.innerHTML = `
      <div class="ik-grid" id="ik-fields"></div>
      <div class="param-row">
        <label>Elbow Config <span class="param-val" id="scara-elbow-label">Right</span></label>
        <button class="mini-btn" id="btn-scara-elbow">Flip Elbow</button>
      </div>
      <p class="help-text">Annular workspace: reach between |l1−l2| and l1+l2.</p>`;
    this._buildPoseFields(['X', 'Y', 'Z']);
    this.scaraElbow = 1;
    document.getElementById('btn-scara-elbow').addEventListener('click', () => {
      this.scaraElbow = -this.scaraElbow;
      document.getElementById('scara-elbow-label').textContent = this.scaraElbow > 0 ? 'Right' : 'Left';
      this.solvePending = true;
    });

    const f = cfg.fk(cfg.joints, cfg.params);
    this.target.position.set(f.x, f.y, f.z);
    this.target.visible = true;
    this.deps.viewport.attachGizmo(this.target, 'translate');
    this._writeFieldsFromTarget();

    this.solver = (t) => {
      const { l1, l2, pedestalH } = cfg.params;
      const r = scaraIK(t.position.x, t.position.z, l1, l2, this.scaraElbow);
      const z = Math.max(0, Math.min(cfg.kinematics.zTravel, pedestalH - t.position.y));
      cfg.joints[0] = r.q1;
      cfg.joints[1] = r.q2;
      cfg.joints[2] = z;
      return r.reachable;
    };
  }

  _activateDexarm(cfg, host) {
    host.innerHTML = `
      <div class="param-row">
        <label>Active Arm <span class="param-val" id="dex-side-label">Right</span></label>
        <button class="mini-btn" id="btn-dex-side">Switch Arm</button>
      </div>
      <p class="help-text">Position IK on shoulder pitch/yaw + elbow. Joints are mirrored L/R.</p>`;
    this.dexSide = 1;
    document.getElementById('btn-dex-side').addEventListener('click', () => {
      this.dexSide = -this.dexSide;
      document.getElementById('dex-side-label').textContent = this.dexSide > 0 ? 'Right' : 'Left';
      const p = cfg.kinematics.fkFn(cfg.joints.slice(0, 3), cfg.params, this.dexSide);
      this.target.position.set(p[0], p[1], p[2]);
    });

    const p0 = cfg.kinematics.fkFn(cfg.joints.slice(0, 3), cfg.params, this.dexSide);
    this.target.position.set(p0[0], p0[1], p0[2]);
    this.target.visible = true;
    this.deps.viewport.attachGizmo(this.target, 'translate');

    this.solver = (t) => {
      const lim = cfg.kinematics.ikJoints.map(i => ({
        min: cfg.jointLimits[i].min * DEG,
        max: cfg.jointLimits[i].max * DEG,
      }));
      const r = solvePositionIK(
        (q) => cfg.kinematics.fkFn(q, cfg.params, this.dexSide),
        cfg.joints.slice(0, 3),
        [t.position.x, t.position.y, t.position.z],
        lim
      );
      for (let i = 0; i < 3; i++) cfg.joints[i] = r.q[i];
      return r.converged;
    };
  }

  _buildPoseFields(labels) {
    const grid = document.getElementById('ik-fields');
    grid.innerHTML = labels.map(l => `
      <div class="ik-field">
        <label for="ikf-${l}">${l}</label>
        <input type="number" id="ikf-${l}" step="${'XYZ'.includes(l) ? 5 : 5}" value="0">
        <span class="ik-unit">${'XYZ'.includes(l) ? 'mm' : '°'}</span>
      </div>`).join('');
    labels.forEach(l => {
      document.getElementById(`ikf-${l}`).addEventListener('change', () => {
        this._readFieldsToTarget(labels);
        this.solvePending = true;
      });
    });
    this.fieldLabels = labels;
  }

  _writeFieldsFromTarget() {
    if (!this.fieldLabels) return;
    const t = this.target;
    const vals = {
      X: t.position.x, Y: t.position.y, Z: t.position.z,
      Roll: t.rotation.x / DEG, Pitch: t.rotation.y / DEG, Yaw: t.rotation.z / DEG,
    };
    for (const l of this.fieldLabels) {
      const el = document.getElementById(`ikf-${l}`);
      if (el && document.activeElement !== el) el.value = vals[l].toFixed(1);
    }
  }

  _readFieldsToTarget(labels) {
    const v = (l) => parseFloat(document.getElementById(`ikf-${l}`).value) || 0;
    this.target.position.set(v('X'), v('Y'), v('Z'));
    if (labels.includes('Roll'))
      this.target.rotation.set(v('Roll') * DEG, v('Pitch') * DEG, v('Yaw') * DEG);
  }

  _setWarning(on, msg = 'Target out of workspace') {
    const w = document.getElementById('ik-warning');
    w.style.display = on ? 'flex' : 'none';
    if (on) w.querySelector('span:last-child').textContent = ` ${msg}`;
    this.target.material.color.setHex(on ? 0xef4444 : 0x22d3ee);
    document.getElementById('ik-status-text').textContent = on ? 'IK Unreachable' : 'IK Tracking';
  }

  _tick() {
    if (!this.solvePending || !this.solver) return;
    this.solvePending = false;
    const ok = this.solver(this.target);
    this._setWarning(!ok);
    this._writeFieldsFromTarget();
    this.deps.onJointsChanged();
    const cfg = this.deps.robots[this.deps.getActiveKey()];
    if (cfg.fk) {
      const f = cfg.fk(cfg.joints, cfg.params);
      this.deps.viewport.pushTracePoint(f.x, f.y, f.z);
    }
  }
}
```

- [ ] **Step 3: Wire into `js/main.js`**

Delete the entire old `btn-ik-solve` handler block. Add import:

```js
import { IKController } from './ik-control.js';
```

Add after `buildAndShowRobot` is defined:

```js
function syncJointInputs() {
  const cfg = robots[state.activeRobot];
  cfg.joints.forEach((val, i) => {
    const limits = cfg.jointLimits[i];
    const disp = limits.isAngle ? val / Math.PI * 180 : val;
    const slider = document.getElementById(`jslider-${i}`);
    const numIn = document.getElementById(`jnum-${i}`);
    if (slider) slider.value = disp;
    if (numIn && document.activeElement !== numIn) numIn.value = Math.round(disp);
  });
}

const ik = new IKController({
  viewport,
  robots,
  getActiveKey: () => state.activeRobot,
  onJointsChanged: () => { syncJointInputs(); rebuildCurrentRobot(); },
});
```

In `buildAndShowRobot`, after `renderJointControls();` add `ik.activate(key);`.

- [ ] **Step 4: Append CSS** (`css/styles.css`)

```css
/* ── IK controls */
#ik-controls .ik-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-bottom: 10px; }
```

(Existing `.ik-field`, `.ik-grid`, `.ik-warning` styles are reused.)

- [ ] **Step 5: Browser verification (chrome-devtools MCP)**

1. Load app. Arm selected: cyan target sphere with translate gizmo at EE tip; 6 pose fields.
2. Drag gizmo: joints track smoothly, sliders update live, EE trace draws cyan line.
3. Drag far outside reach: sphere turns red, warning shows, robot holds best-effort pose, no console errors, no NaN slider values.
4. Click Rotate, rotate ring: wrist reorients.
5. SCARA: gizmo XZ drag tracks; below-min reach (inside annulus) flags red; Flip Elbow snaps to mirror solution.
6. Dexarm: drag target near front of torso — right arm reaches it; Switch Arm moves target to left palm.
7. Sequencer + STL export still work on arm (record 2 poses, play).

- [ ] **Step 6: Commit**

```bash
git add js/ik-control.js js/main.js index.html css/styles.css
git commit -m "feat: add live IK target gizmo for arm, SCARA, and bimanual robots"
```

---

### Task 10: Quadruped body-pose mode + humanoid limb IK + COM marker

**Files:**
- Modify: `js/ik-control.js` (add `_activateQuad`, `_activateHumanoid`)
- Modify: `js/main.js` (pass body pose through params)

- [ ] **Step 1: Quadruped body-pose mode in `ik-control.js`**

Add branch in `activate()`: `else if (this.mode === 'quad-legs') this._activateQuad(cfg, host);`

```js
  _activateQuad(cfg, host) {
    host.innerHTML = `
      <p class="help-text">Drag the body — feet stay planted (per-leg analytical IK). Toggle gizmo mode for body rotation.</p>
      <button class="mini-btn" id="btn-quad-reset">Reset Stance</button>`;
    document.getElementById('btn-ik-mode').style.display = '';

    const legs = cfg.kinematics.legs(cfg.params);
    const F = cfg.params.femur, T = cfg.params.tibia + cfg.kinematics.footOffset;
    const Cx = cfg.params.coxa;

    // record stance: world foot positions at current joints (legFK from kinematics.js)
    const stance = legs.map(leg => {
      const q = cfg.joints.slice(leg.joint0, leg.joint0 + 3);
      const p = legFK(q[0], q[1], q[2], Cx, F, T, leg.side);
      return [leg.hip[0] + p.x, leg.hip[1] + p.y, leg.hip[2] + p.z];
    });

    this.target.position.set(0, legs[0].hip[1], 0);
    this.target.rotation.set(0, 0, 0);
    this.target.visible = true;
    this.deps.viewport.attachGizmo(this.target, 'translate');

    document.getElementById('btn-quad-reset').addEventListener('click', () => {
      this.target.position.set(0, legs[0].hip[1], 0);
      this.target.rotation.set(0, 0, 0);
      this.solvePending = true;
    });

    const H0 = legs[0].hip[1];
    this.solver = (t) => {
      // body transform B relative to neutral: translation (t.pos − (0,H0,0)) + rotation
      const B = new THREE.Matrix4().makeRotationFromEuler(t.rotation);
      const bodyPos = new THREE.Vector3(t.position.x, t.position.y, t.position.z);
      const Binv = B.clone().invert();
      let allReachable = true;
      legs.forEach((leg, i) => {
        // hip world = bodyPos + B·(hipLocal − bodyCentreLocal)
        const hipLocal = new THREE.Vector3(leg.hip[0], 0, leg.hip[2]); // relative to body centre
        const hipWorld = hipLocal.clone().applyMatrix4(B).add(bodyPos);
        const foot = new THREE.Vector3(...stance[i]);
        // target in (rotated) hip frame
        const rel = foot.clone().sub(hipWorld).applyMatrix4(Binv);
        const r = legIK(rel.x, rel.y, rel.z, Cx, F, T, leg.side);
        if (!r.reachable) allReachable = false;
        cfg.joints[leg.joint0] = r.q0;
        cfg.joints[leg.joint0 + 1] = r.q1;
        cfg.joints[leg.joint0 + 2] = r.q2;
      });
      // pass body pose to the builder
      cfg.params._bodyPose = {
        x: bodyPos.x, y: bodyPos.y - H0, z: bodyPos.z,
        rx: t.rotation.x, ry: t.rotation.y, rz: t.rotation.z,
      };
      return allReachable;
    };
  }
```

The hip local offsets in `solver` use `(leg.hip[0], 0, leg.hip[2])` because hips sit at body height (the body centre is the rotation origin).

Note `_bodyPose` y is stored relative (gizmo starts at hip height H0). The builder (Task 7 Step 2) applies it to `bodyGroup`.

**Cleanup on robot switch:** in `activate()`, before branching: `delete this.deps.robots[this.deps.getActiveKey()]?.params?._bodyPose;` — actually clear it for ALL robots: iterate `Object.values(this.deps.robots)` and `delete cfg.params._bodyPose`.

- [ ] **Step 2: Humanoid limb IK + COM marker**

Add branch: `else if (this.mode === 'limbs') this._activateHumanoid(cfg, host);`

```js
  _activateHumanoid(cfg, host) {
    host.innerHTML = `
      <div class="param-row">
        <label>Target Limb <span class="param-val" id="limb-label">R Arm</span></label>
        <select class="styled-select" id="limb-select">
          <option value="rarm">Right Arm</option>
          <option value="larm">Left Arm</option>
          <option value="rleg">Right Leg (foot)</option>
          <option value="lleg">Left Leg (foot)</option>
        </select>
      </div>
      <p class="help-text">2-link analytical IK per limb. Green disc = ground-projected COM; red = outside support polygon.</p>`;

    const kin = cfg.kinematics;
    const arm = kin.arm(cfg.params), leg = kin.leg(cfg.params);
    const rootY = kin.rootY(cfg.params);

    // COM marker
    this.comMarker = new THREE.Mesh(
      new THREE.CylinderGeometry(12, 12, 2, 24),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.comMarker.position.y = 1;
    this.deps.viewport.addKinHelper(this.comMarker);

    const jointMap = {
      rarm: { s: 4, e: 6, side: 1 },
      larm: { s: 1, e: 3, side: -1 },
      rleg: { hip: 10, knee: 11, ankle: 12, side: 1 },
      lleg: { hip: 7, knee: 8, ankle: 9, side: -1 },
    };
    let limb = 'rarm';
    const sel = document.getElementById('limb-select');
    sel.addEventListener('change', () => { limb = sel.value; placeTarget(); });

    const placeTarget = () => {
      const m = jointMap[limb];
      if (m.s !== undefined) {
        const qs = cfg.joints[m.s], qe = cfg.joints[m.e];
        // arm FK (frontal plane): hangs −Y, RotZ swing
        const x = arm.upper * Math.sin(qs) + arm.fore * Math.sin(qs + qe);
        const y = -(arm.upper * Math.cos(qs) + arm.fore * Math.cos(qs + qe));
        this.target.position.set(m.side * arm.shoulderX + x, rootY + arm.shoulderY + y, 0);
      } else {
        const qh = cfg.joints[m.hip], qk = cfg.joints[m.knee];
        const y = -(leg.thigh * Math.cos(qh) + leg.shin * Math.cos(qh + qk));
        const z = -(leg.thigh * Math.sin(qh) + leg.shin * Math.sin(qh + qk));
        this.target.position.set(m.side * leg.hipX, rootY + leg.hipY + y, z);
      }
    };
    placeTarget();
    this.target.visible = true;
    this.deps.viewport.attachGizmo(this.target, 'translate');

    this.solver = (t) => {
      const m = jointMap[limb];
      let ok;
      if (m.s !== undefined) {
        const lx = t.position.x - m.side * arm.shoulderX;
        const ly = t.position.y - (rootY + arm.shoulderY);
        // planar map: w = −ly, u = lx (see Task 4 conventions)
        const r = twoLinkIK(-ly, lx, arm.upper, arm.fore, -1);
        cfg.joints[m.s] = r.q1;
        cfg.joints[m.e] = r.q2;
        ok = r.reachable;
      } else {
        const ly = t.position.y - (rootY + leg.hipY);
        const lz = t.position.z;
        const r = twoLinkIK(-ly, -lz, leg.thigh, leg.shin, 1);
        cfg.joints[m.hip] = r.q1;
        cfg.joints[m.knee] = r.q2;
        cfg.joints[m.ankle] = -(r.q1 + r.q2); // keep foot level
        ok = r.reachable;
      }
      this._updateCOM(cfg);
      return ok;
    };
    this._updateCOM(cfg);
  }

  _updateCOM(cfg) {
    const kin = cfg.kinematics;
    const m = kin.masses;
    const p = cfg.params;
    const rootY = kin.rootY(p);
    const leg = kin.leg(p);
    // segment COM points (world, coarse): torso/head fixed to torso frame,
    // limb segment midpoints from the same trig the mesh uses
    const pts = [];
    pts.push([0, rootY + 70, 0, m.torso]);
    pts.push([0, rootY + 190, 0, m.head]);
    for (const side of [-1, 1]) {
      const s = side > 0 ? 4 : 1, e = side > 0 ? 6 : 3;
      const arm = kin.arm(p);
      const qs = cfg.joints[s], qe = cfg.joints[e];
      const midX = arm.upper * 0.5 * Math.sin(qs);
      const midY = -arm.upper * 0.5 * Math.cos(qs);
      pts.push([side * arm.shoulderX + midX, rootY + arm.shoulderY + midY, 0, m.arm * 2]);
      const hip = side > 0 ? 10 : 7, knee = side > 0 ? 11 : 8;
      const qh = cfg.joints[hip], qk = cfg.joints[knee];
      const thighMid = [0, -p.thigh * 0.5 * Math.cos(qh), -p.thigh * 0.5 * Math.sin(qh)];
      pts.push([side * leg.hipX, rootY + leg.hipY + thighMid[1], thighMid[2], m.thigh]);
      const footY = -(p.thigh * Math.cos(qh) + p.shin * Math.cos(qh + qk));
      const footZ = -(p.thigh * Math.sin(qh) + p.shin * Math.sin(qh + qk));
      pts.push([side * leg.hipX, rootY + leg.hipY + footY, footZ, m.shin + m.foot]);
    }
    let cx = 0, cz = 0, mt = 0;
    for (const [x, , z, w] of pts) { cx += x * w; cz += z * w; mt += w; }
    cx /= mt; cz /= mt;
    this.comMarker.position.set(cx, 1, cz);
    // support polygon: union of two foot rects (40 × 90, centred at foot, +20 z offset)
    const feet = [];
    for (const side of [-1, 1]) {
      const hip = side > 0 ? 10 : 7, knee = side > 0 ? 11 : 8;
      const qh = cfg.joints[hip], qk = cfg.joints[knee];
      const fz = -(p.thigh * Math.sin(qh) + p.shin * Math.sin(qh + qk)) + 20;
      feet.push({ x: side * kin.leg(p).hipX, z: fz });
    }
    const minX = Math.min(...feet.map(f => f.x)) - 20, maxX = Math.max(...feet.map(f => f.x)) + 20;
    const minZ = Math.min(...feet.map(f => f.z)) - 45, maxZ = Math.max(...feet.map(f => f.z)) + 45;
    const inside = cx >= minX && cx <= maxX && cz >= minZ && cz <= maxZ;
    this.comMarker.material.color.setHex(inside ? 0x22c55e : 0xef4444);
  }
```

Import `twoLinkIK`, `legFK`, and `legIK` from `kinematics.js` at the top of `ik-control.js` (already in the Task 9 import list).

Also in `activate()`: `viewport.clearKinHelpers();` and `this.comMarker = null;` at the start, so markers don't leak across robots.

- [ ] **Step 3: Browser verification**

1. Quadruped: drag body down — robot squats, feet stay planted on the grid (the signature Spot demo). Rotate mode: body pitches/rolls, feet planted. Beyond reach: warning + red target.
2. Humanoid: select R Arm, drag target — arm tracks in frontal plane. Select R Leg, drag down/forward — leg bends, foot stays level. COM disc visible on ground, turns red when leaning legs far forward.
3. Switching robots leaves no stray markers/gizmos.

- [ ] **Step 4: Commit**

```bash
git add js/ik-control.js js/main.js
git commit -m "feat: add quadruped body-pose IK and humanoid limb IK with COM marker"
```

---

### Task 11: Drone stick pads + rover steering UI

**Files:**
- Modify: `js/ik-control.js` (add `_activateDrone`, `_activateRover`)
- Modify: `css/styles.css` (stick pad styles)

- [ ] **Step 1: Drone — virtual sticks + mixer-driven props + tilt**

Add branch: `else if (this.mode === 'mixer') this._activateDrone(cfg, host);`

```js
  _activateDrone(cfg, host) {
    host.innerHTML = `
      <div class="stick-row">
        <div class="stick-pad" id="stick-left"><div class="stick-nub"></div><span class="stick-label">Thr / Yaw</span></div>
        <div class="stick-pad" id="stick-right"><div class="stick-nub"></div><span class="stick-label">Pitch / Roll</span></div>
      </div>
      <div id="motor-bars">
        ${['FR', 'FL', 'BL', 'BR'].map(n => `
          <div class="motor-bar"><span>${n}</span><div class="motor-fill" id="mbar-${n}"></div><span id="mval-${n}">0%</span></div>`).join('')}
      </div>`;
    this.droneInput = { thrust: 0.5, yaw: 0, pitch: 0, roll: 0 };
    this._bindStick('stick-left', (x, y) => {
      this.droneInput.yaw = x;
      this.droneInput.thrust = (1 - y) / 2; // top = full thrust
    }, false);
    this._bindStick('stick-right', (x, y) => {
      this.droneInput.roll = x;
      this.droneInput.pitch = -y; // pad up = nose down (forward)
    }, true);

    // continuous animation: spin props by motor output, tilt body
    this.droneTick = () => {
      const { thrust, roll, pitch, yaw } = this.droneInput;
      const m = quadMix(thrust, roll, pitch, yaw);
      ['FR', 'FL', 'BL', 'BR'].forEach((n, i) => {
        const fill = document.getElementById(`mbar-${n}`);
        const val = document.getElementById(`mval-${n}`);
        if (fill) { fill.style.width = `${m[i] * 100}%`; val.textContent = `${Math.round(m[i] * 100)}%`; }
        cfg.joints[i] = (cfg.joints[i] + m[i] * 0.9) % (Math.PI * 2);
      });
      // kinematic attitude response
      const mesh = this.deps.getCurrentMesh();
      if (mesh) {
        mesh.rotation.x = pitch * 0.35;
        mesh.rotation.z = -roll * 0.35;
        mesh.rotation.y = mesh.rotation.y - yaw * 0.02;
      }
      this.deps.applyJointsLight(); // rebuild for prop spin
      this.lastMix = m;
    };
  }

  _bindStick(id, onMove, snapBack) {
    const pad = document.getElementById(id);
    const nub = pad.querySelector('.stick-nub');
    const setNub = (x, y) => {
      nub.style.left = `${50 + x * 38}%`;
      nub.style.top = `${50 + y * 38}%`;
    };
    setNub(0, snapBack ? 0 : 0);
    let dragging = false;
    const update = (ev) => {
      const r = pad.getBoundingClientRect();
      const x = Math.max(-1, Math.min(1, ((ev.clientX - r.left) / r.width - 0.5) * 2));
      const y = Math.max(-1, Math.min(1, ((ev.clientY - r.top) / r.height - 0.5) * 2));
      setNub(x, y);
      onMove(x, y);
    };
    pad.addEventListener('pointerdown', (e) => { dragging = true; pad.setPointerCapture(e.pointerId); update(e); });
    pad.addEventListener('pointermove', (e) => { if (dragging) update(e); });
    pad.addEventListener('pointerup', () => {
      dragging = false;
      if (snapBack) { setNub(0, 0); onMove(0, 0); }
    });
  }
```

Wiring notes:
- Add to `_tick()`: `if (this.droneTick && this.mode === 'mixer') this.droneTick();`
- In `activate()`: `this.droneTick = null;` at the start.
- `deps` gains two callbacks from `main.js`:
  - `getCurrentMesh: () => currentMesh`
  - `applyJointsLight: () => rebuildCurrentRobot()` — for the drone this is called every frame; current models rebuild in well under a frame, acceptable. If profiling shows jank, throttle to every 2nd frame.
- Drone tilt is applied to the rebuilt mesh each frame AFTER rebuild — store tilt in `this.droneAttitude` and apply inside `droneTick` after `applyJointsLight()` (rebuild resets rotation; reapply).

- [ ] **Step 2: Rover — turn-radius steering UI**

Add branch: `else if (this.mode === 'ackermann') this._activateRover(cfg, host);`

```js
  _activateRover(cfg, host) {
    host.innerHTML = `
      <div class="param-row">
        <label>Turn Radius <span class="param-val" id="rover-radius-label">∞ (straight)</span></label>
        <input type="range" id="rover-radius" min="-100" max="100" step="1" value="0">
      </div>
      <div class="param-row">
        <label>Drive Speed <span class="param-val" id="rover-speed-label">0</span></label>
        <input type="range" id="rover-speed" min="-100" max="100" step="5" value="0">
      </div>
      <div class="help-text" id="rover-readout"></div>`;
    this.roverSpeed = 0;

    const applySteer = () => {
      const v = parseFloat(document.getElementById('rover-radius').value);
      const g = cfg.kinematics.geometry(cfg.params);
      // slider − maps to curvature: 0 = straight, ±100 = tightest (radius = track)
      const radius = v === 0 ? Infinity : (Math.sign(v) * (g.track / 2 + 20 + (100 - Math.abs(v)) * 8));
      const a = ackermann(radius, g.wheelbase, g.track);
      cfg.joints[1] = a.fl; cfg.joints[2] = a.fr; cfg.joints[3] = a.rl; cfg.joints[4] = a.rr;
      document.getElementById('rover-radius-label').textContent =
        Number.isFinite(radius) ? `${Math.round(radius)} mm ${radius > 0 ? '(left)' : '(right)'}` : '∞ (straight)';
      document.getElementById('rover-readout').textContent =
        `Steer °: FL ${(a.fl / DEG).toFixed(1)} · FR ${(a.fr / DEG).toFixed(1)} · RL ${(a.rl / DEG).toFixed(1)} · RR ${(a.rr / DEG).toFixed(1)}`;
      this.deps.onJointsChanged();
    };
    document.getElementById('rover-radius').addEventListener('input', applySteer);
    document.getElementById('rover-speed').addEventListener('input', (e) => {
      this.roverSpeed = parseFloat(e.target.value) / 100;
      document.getElementById('rover-speed-label').textContent = e.target.value;
    });
    applySteer();

    this.roverTickFn = () => {
      if (!this.roverSpeed) return;
      cfg.joints[0] = (cfg.joints[0] + this.roverSpeed * 0.15) % (Math.PI * 2);
      this.deps.applyJointsLight();
    };
  }
```

Add to `_tick()`: `if (this.roverTickFn && this.mode === 'ackermann') this.roverTickFn();` and reset `this.roverTickFn = null;` in `activate()`.

- [ ] **Step 3: Append CSS**

```css
/* ── Drone sticks */
.stick-row { display: flex; gap: 12px; margin-bottom: 12px; }
.stick-pad {
  position: relative; flex: 1; aspect-ratio: 1; border-radius: 12px;
  background: var(--bg-inset, rgba(0,0,0,0.25)); border: 1px solid var(--border, #334);
  touch-action: none; cursor: crosshair;
}
.stick-nub {
  position: absolute; width: 22px; height: 22px; border-radius: 50%;
  background: var(--accent, #3b82f6); transform: translate(-50%, -50%);
  left: 50%; top: 50%; pointer-events: none;
}
.stick-label {
  position: absolute; bottom: 4px; left: 0; right: 0; text-align: center;
  font-size: 10px; opacity: 0.6; pointer-events: none;
}
#motor-bars { display: flex; flex-direction: column; gap: 4px; }
.motor-bar { display: grid; grid-template-columns: 28px 1fr 40px; align-items: center; gap: 6px; font-size: 11px; }
.motor-bar > div { height: 8px; border-radius: 4px; background: var(--accent, #3b82f6); width: 0%; transition: width 80ms; }
```

- [ ] **Step 4: Browser verification**

1. Drone: left stick up → all motor bars rise, props spin faster. Right stick right → FL/BL bars > FR/BR, body banks right. Right stick releases → level. Yaw → body slowly rotates.
2. Rover: radius slider left → left wheels steer harder than right (Ackermann), readout shows 4 angles, rear mirrors front negated. Drive speed spins wheels continuously.

- [ ] **Step 5: Commit**

```bash
git add js/ik-control.js css/styles.css
git commit -m "feat: add drone stick pads with motor mixing and rover Ackermann steering"
```

---

### Task 12: Expert mode panel

**Files:**
- Create: `js/expert-panel.js`
- Modify: `index.html` (header button + panel markup)
- Modify: `js/main.js` (wire updates)
- Modify: `css/styles.css` (append)

- [ ] **Step 1: Header toggle in `index.html`** — add before the theme button:

```html
        <button class="icon-btn" id="btn-expert" title="Expert Mode (show the math)">
          <i class="fa-solid fa-square-root-variable"></i>
        </button>
```

Add panel container at the end of `#viewport-wrap` (after the robot name badge):

```html
        <div id="expert-panel" class="hidden">
          <div class="expert-title">Kinematics <span id="expert-anchor"></span></div>
          <div id="expert-content"></div>
        </div>
```

- [ ] **Step 2: Create `js/expert-panel.js`**

```js
/**
 * expert-panel.js — live DH table, EE pose, transform matrix,
 * manipulability, per-robot kinematic readouts.
 */
import { DHChain, dhToWorld, legFK, DEG } from './kinematics.js';

const fmt = (v, d = 1) => (Math.abs(v) < 1e-9 ? 0 : v).toFixed(d);

export class ExpertPanel {
  constructor() {
    this.enabled = false;
    this.panel = document.getElementById('expert-panel');
    document.getElementById('btn-expert').addEventListener('click', () => {
      this.enabled = !this.enabled;
      this.panel.classList.toggle('hidden', !this.enabled);
      document.getElementById('btn-expert').classList.toggle('active', this.enabled);
      if (this.enabled && this.lastArgs) this.update(...this.lastArgs);
    });
  }

  update(cfg, extras = {}) {
    this.lastArgs = [cfg, extras];
    if (!this.enabled) return;
    const kin = cfg.kinematics;
    document.getElementById('expert-anchor').textContent = kin?.anchor ? `· ${kin.anchor}` : '';
    const host = document.getElementById('expert-content');
    if (!kin) { host.innerHTML = '<p class="help-text">No kinematic model.</p>'; return; }

    if (kin.type === 'dh' || kin.type === 'scara') host.innerHTML = this._dhView(cfg);
    else if (kin.type === 'quad-legs') host.innerHTML = this._quadView(cfg);
    else if (kin.type === 'mixer') host.innerHTML = this._mixerView(extras.mix);
    else if (kin.type === 'ackermann') host.innerHTML = this._roverView(cfg);
    else if (kin.type === 'limbs') host.innerHTML = this._humanoidView(extras);
    else if (kin.type === 'numeric-arms') host.innerHTML = this._dexView(cfg);
  }

  _dhView(cfg) {
    const kin = cfg.kinematics;
    const q = cfg.joints;
    const rows = kin.rows(q, cfg.params);
    const chain = new DHChain((qq) => kin.rows(qq, cfg.params));
    const { ee } = chain.fk(q.slice(0, rows.length));
    const [wx, wy, wz] = dhToWorld([ee[0][3], ee[1][3], ee[2][3]]);
    let html = `<table class="dh-table">
      <tr><th>i</th><th>θ (°)</th><th>d (mm)</th><th>a (mm)</th><th>α (°)</th><th>v<sub>max</sub></th></tr>`;
    rows.forEach((r, i) => {
      const prismatic = kin.prismatic?.includes(i);
      html += `<tr${prismatic ? ' class="prismatic"' : ''}>
        <td>${i + 1}${prismatic ? ' P' : ''}</td>
        <td>${fmt(r[0] / DEG)}</td><td>${fmt(r[1])}</td>
        <td>${fmt(r[2])}</td><td>${fmt(r[3] / DEG, 0)}</td>
        <td>${kin.speeds ? kin.speeds[i] + '°/s' : '—'}</td></tr>`;
    });
    html += '</table>';
    html += `<div class="expert-row"><b>EE (world)</b> X ${fmt(wx)} Y ${fmt(wy)} Z ${fmt(wz)} mm</div>`;
    html += `<div class="expert-row mono"><b>T<sub>0→EE</sub></b><br>${ee.map(r => r.map(v => fmt(v, 2).padStart(8)).join(' ')).join('<br>')}</div>`;
    if (kin.type === 'dh') {
      const w = chain.manipulability(q.slice(0, 6));
      const pct = Math.min(100, w / 5e9 * 100); // empirical full-scale; tune after first run
      const danger = pct < 12;
      html += `<div class="expert-row"><b>Manipulability</b>
        <div class="manip-bar"><div class="manip-fill${danger ? ' danger' : ''}" style="width:${pct}%"></div></div>
        ${danger ? '<span class="manip-warn">⚠ near singularity</span>' : ''}</div>`;
    }
    if (kin.facts) html += `<div class="expert-row help-text">${kin.facts}</div>`;
    return html;
  }

  _quadView(cfg) {
    const kin = cfg.kinematics;
    const p = cfg.params;
    const legs = kin.legs(p);
    let html = `<table class="dh-table"><tr><th>Leg</th><th>q0°</th><th>q1°</th><th>q2°</th><th>Foot (x,y,z)</th></tr>`;
    for (const leg of legs) {
      const q = cfg.joints.slice(leg.joint0, leg.joint0 + 3);
      const f = legFK(q[0], q[1], q[2], p.coxa, p.femur, p.tibia + kin.footOffset, leg.side);
      html += `<tr><td>${leg.name}</td><td>${fmt(q[0] / DEG)}</td><td>${fmt(q[1] / DEG)}</td><td>${fmt(q[2] / DEG)}</td>
        <td>${fmt(leg.hip[0] + f.x)}, ${fmt(leg.hip[1] + f.y)}, ${fmt(leg.hip[2] + f.z)}</td></tr>`;
    }
    return html + '</table>';
  }

  _mixerView(mix) {
    let html = `<div class="expert-row mono"><b>Mixer (X-quad)</b><br>
      FR = T − 0.25R − 0.25P + 0.25Y<br>FL = T + 0.25R − 0.25P − 0.25Y<br>
      BL = T + 0.25R + 0.25P + 0.25Y<br>BR = T − 0.25R + 0.25P − 0.25Y</div>`;
    if (mix) html += `<div class="expert-row"><b>Motors</b> ${mix.map((m, i) => `${['FR','FL','BL','BR'][i]} ${Math.round(m * 100)}%`).join(' · ')}</div>`;
    return html;
  }

  _roverView(cfg) {
    const g = cfg.kinematics.geometry(cfg.params);
    return `<div class="expert-row"><b>Geometry</b> wheelbase ${fmt(g.wheelbase)} mm · track ${fmt(g.track)} mm</div>
      <div class="expert-row"><b>Steer</b> FL ${fmt(cfg.joints[1] / DEG)}° FR ${fmt(cfg.joints[2] / DEG)}° RL ${fmt(cfg.joints[3] / DEG)}° RR ${fmt(cfg.joints[4] / DEG)}°</div>
      <div class="expert-row help-text">4-wheel Ackermann: all wheel axes intersect at one turn centre — no scrubbing.</div>`;
  }

  _humanoidView(extras) {
    return `<div class="expert-row"><b>COM (ground)</b> ${extras.com ? `${fmt(extras.com.x)}, ${fmt(extras.com.z)} mm — ${extras.com.inside ? 'inside support ✓' : 'OUTSIDE support ⚠'}` : '—'}</div>
      <div class="expert-row help-text">Static stability: ground-projected COM must stay inside the foot support polygon.</div>`;
  }

  _dexView(cfg) {
    const p = cfg.params;
    const kin = cfg.kinematics;
    const r = kin.fkFn(cfg.joints.slice(0, 3), p, 1);
    const l = kin.fkFn(cfg.joints.slice(0, 3), p, -1);
    return `<div class="expert-row"><b>R palm</b> ${fmt(r[0])}, ${fmt(r[1])}, ${fmt(r[2])} mm</div>
      <div class="expert-row"><b>L palm</b> ${fmt(l[0])}, ${fmt(l[1])}, ${fmt(l[2])} mm</div>`;
  }
}
```

`_updateCOM` in `ik-control.js` should stash `{x: cx, z: cz, inside}` on `this.lastCOM` and `main.js` passes it through `extras`.

- [ ] **Step 3: Wire in `js/main.js`**

```js
import { ExpertPanel } from './expert-panel.js';
const expertPanel = new ExpertPanel();
```

In `rebuildCurrentRobot()` and `buildAndShowRobot()` (after telemetry update):

```js
  expertPanel.update(robots[state.activeRobot], { mix: ik.lastMix, com: ik.lastCOM });
```

(Declare `ik` before first `buildAndShowRobot` call, or guard with `typeof ik !== 'undefined'` — order init: viewport → ik controller → expert panel → buildAndShowRobot.)

- [ ] **Step 4: Append CSS**

```css
/* ── Expert panel */
#expert-panel {
  position: absolute; left: 12px; bottom: 12px; max-width: 420px; max-height: 55%;
  overflow: auto; padding: 12px 14px; border-radius: 10px; z-index: 5;
  background: color-mix(in srgb, var(--bg-panel, #16202e) 88%, transparent);
  border: 1px solid var(--border, #334); backdrop-filter: blur(6px);
  font-size: 12px;
}
#expert-panel.hidden { display: none; }
.expert-title { font-weight: 600; margin-bottom: 8px; }
.expert-row { margin-top: 8px; }
.mono { font-family: 'JetBrains Mono', monospace; white-space: pre; font-size: 11px; }
.dh-table { width: 100%; border-collapse: collapse; font-family: 'JetBrains Mono', monospace; font-size: 11px; }
.dh-table th, .dh-table td { padding: 2px 6px; text-align: right; border-bottom: 1px solid var(--border, #334); }
.dh-table tr.prismatic td { color: var(--accent, #3b82f6); }
.manip-bar { height: 8px; border-radius: 4px; background: rgba(127,127,127,0.25); overflow: hidden; margin-top: 4px; }
.manip-fill { height: 100%; background: #22c55e; transition: width 100ms; }
.manip-fill.danger { background: #ef4444; }
.manip-warn { color: #ef4444; font-size: 11px; }
```

- [ ] **Step 5: Browser verification**

1. Expert button toggles panel; persists across robot switches.
2. Arm: DH table θ column updates live while dragging gizmo; EE world readout matches telemetry overlay; manipulability bar drops + reddens when arm fully stretched (singular).
3. SCARA: 4-row table with prismatic row highlighted.
4. Quadruped: per-leg foot table updates while dragging body.
5. Drone: mixer equations + live motor %; rover: geometry + steer °; humanoid: COM readout matches disc color; dexarm: palm positions.
6. Tune the manipulability full-scale constant (`5e9`) so the default arm pose reads ~40–70%.

- [ ] **Step 6: Commit**

```bash
git add js/expert-panel.js js/main.js index.html css/styles.css
git commit -m "feat: add expert mode panel with live DH table, pose, and manipulability"
```

---

### Task 13: Frame triads toggle + telemetry via kinematics

**Files:**
- Modify: `index.html` (toolbar button)
- Modify: `js/main.js` (triad rendering + telemetry)

- [ ] **Step 1: Toolbar button** — add next to `btn-wireframe`:

```html
            <button class="toolbar-btn" id="btn-frames" title="Joint Frames">
              <i class="fa-solid fa-location-crosshairs"></i> Frames
            </button>
```

- [ ] **Step 2: Triad rendering in `js/main.js`**

```js
import { DHChain, dhToWorld } from './kinematics.js';

let showFrames = false;
document.getElementById('btn-frames').addEventListener('click', () => {
  showFrames = !showFrames;
  document.getElementById('btn-frames').classList.toggle('active', showFrames);
  renderFrameTriads();
});

function renderFrameTriads() {
  viewport.clearKinHelpers();
  if (ik.comMarker) viewport.addKinHelper(ik.comMarker); // preserve COM marker
  if (!showFrames) return;
  const cfg = robots[state.activeRobot];
  const kin = cfg.kinematics;
  if (!kin?.rows) return; // triads only for DH robots this cycle
  const chain = new DHChain((q) => kin.rows(q, cfg.params));
  const n = kin.rows(cfg.joints, cfg.params).length;
  const { frames } = chain.fk(cfg.joints.slice(0, n));
  for (const T of frames) {
    const triad = viewport.makeTriad();
    const [x, y, z] = dhToWorld([T[0][3], T[1][3], T[2][3]]);
    triad.position.set(x, y, z);
    // orientation: world R = C · R_dh · Cᵀ
    const m = new THREE.Matrix4().set(
      T[0][0], T[0][1], T[0][2], 0,
      T[2][0], T[2][1], T[2][2], 0,
      -T[1][0], -T[1][1], -T[1][2], 0,
      0, 0, 0, 1
    );
    triad.quaternion.setFromRotationMatrix(m);
    viewport.addKinHelper(triad);
  }
}
```

Requires `import * as THREE from 'three';` in `main.js`. Call `renderFrameTriads()` at the end of `rebuildCurrentRobot()`.

Note on the rotation rows: world basis columns are images of DH basis vectors under `(x,y,z)→(x,z,−y)` — row 2 of the world matrix takes DH row 3 values and row 3 takes negated DH row 2, as written.

- [ ] **Step 3: Telemetry already DH-driven** — `updateTelemetry` uses `cfg.fk` which Task 6/7 made kinematics-backed for arm + scara. Confirm it shows live values for both, and `0,0,0` placeholder is gone for scara.

- [ ] **Step 4: Browser verification** — Frames button shows RGB triads at every arm joint, following the mesh as joints move; off for non-DH robots; no leaks after robot switch.

- [ ] **Step 5: Commit**

```bash
git add index.html js/main.js
git commit -m "feat: add joint frame triad visualization"
```

---

### Task 14: README, final regression pass

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Full regression in browser (chrome-devtools MCP)**

1. `?test=1` → all tests green (final count from Task 7: 36 passed, 0 failed).
2. Golden path per robot: select → joints move → IK interaction works → export list renders → sequencer records/plays → theme toggle → wireframe (gizmo unaffected).
3. Param sliders rebuild without breaking active IK (target stays, solver re-tracks).
4. No console errors across all 7 robots.

- [ ] **Step 2: Update `README.md`**

- Robot table: arm row becomes true 6-DOF; add note on UR5e anchoring.
- Replace "Inverse Kinematics (IK Solver)" feature section: describe DLS Jacobian IK with drag gizmo, analytical SCARA/leg IK, quadruped body-pose mode, humanoid COM, rover Ackermann, drone motor mixing.
- Add "Expert Mode" feature section: live DH table, EE pose/matrix, manipulability + singularity warning.
- Architecture section: add `kinematics.js`, `ik-control.js`, `expert-panel.js`, `tests.js` with one-line responsibilities; update line counts.
- Educational Use: add bullets for DH parameters, Jacobian/DLS, singularities, static stability (COM), Ackermann steering.
- Contributing: remove "Collision detection and workspace boundary visualization"? No — keep; remove nothing except items now done (none were).

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: document real kinematics features in README"
```

---

## Plan Self-Review Notes (already applied)

- Spec coverage: DH/FK/DLS (Tasks 2–3), analytical SCARA + legs (Task 4), Ackermann + mixer (Task 5), per-robot configs incl. real-spec anchors (Tasks 6–7), gizmo + numeric sync + unreachable red (Task 9), quadruped body pose + humanoid limbs + COM (Task 10), drone sticks + rover steering (Task 11), expert mode w/ DH table, pose, matrix, manipulability (Task 12), frame triads + EE trace (Tasks 8, 13), tests via `?test=1` (Task 1, grown through Task 7), README (Task 14). Error handling (red gizmo / clamps / no-NaN) covered in Tasks 3, 9.
- Known judgment calls baked in: simplified wrist (no UR lateral d4 offset) — labeled honestly via `anchor` strings; humanoid Shoulder Roll joints remain cosmetic (mesh never used them); drone rebuild-per-frame accepted with throttle fallback noted.
- Type consistency: `kinematics.type` strings used by `ik-control.js` and `expert-panel.js` match Task 7 definitions (`dh`, `scara`, `quad-legs`, `limbs`, `numeric-arms`, `ackermann`, `mixer`). `deps` callbacks: `getActiveKey`, `onJointsChanged`, `getCurrentMesh`, `applyJointsLight` — all defined in Task 9/11 wiring.
