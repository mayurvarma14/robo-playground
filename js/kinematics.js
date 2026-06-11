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
