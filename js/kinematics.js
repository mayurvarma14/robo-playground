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

  // 6-vector pose error. targetRot: 3×3 row-major or null (position-only).
  // rotWeight balances radians against mm so one error norm drives both.
  poseError(T, targetPos, targetRot, rotWeight = 20) {
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
    const { maxIter = 100, tolPos = 0.05, tolRot = 0.01, lambda = 6, rotWeight = 20 } = opts;
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
}
