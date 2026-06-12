/**
 * ik-control.js — per-robot IK interaction: target gizmo, numeric fields,
 * solver wiring. Owns the IK target objects in the scene.
 */
import * as THREE from 'three';
import {
  DHChain, solvePositionIK, scaraIK, twoLinkIK, legIK, legFK, ackermann, quadMix,
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
   * deps: { viewport, robots, getActiveKey, onJointsChanged, getCurrentMesh, applyJointsLight }
   * onJointsChanged(): re-applies cfg.joints to the mesh + syncs sliders.
   * applyJointsLight(): rebuilds the mesh only (no slider sync) — per-frame animation path.
   * getCurrentMesh(): the live robot mesh, for drone attitude tilt after rebuild.
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
    this.solver = null;        // drop the previous robot's solver closure
    this.solvePending = false; // and any solve queued during the switch
    this.fieldLabels = null;
    this.droneTick = null;
    this.roverTickFn = null;
    this._setWarning(false);
    document.getElementById('btn-ik-mode').style.display = 'none';
    this.mode = cfg.kinematics?.type ?? null;
    // _bodyPose is controller-owned: clear it everywhere so a robot switched
    // away from rebuilds in its neutral stance next time
    for (const c of Object.values(robots)) delete c.params._bodyPose;
    if (this.comMarker) {
      this.comMarker.geometry.dispose();
      this.comMarker.material.dispose();
    }
    viewport.clearKinHelpers();
    this.comMarker = null;
    // gizmo always starts a robot in translate mode; keep the toggle in sync
    viewport.setGizmoMode('translate');
    document.getElementById('btn-ik-mode').textContent = 'Rotate';

    if (this.mode === 'dh') this._activateArm(cfg, host);
    else if (this.mode === 'scara') this._activateScara(cfg, host);
    else if (this.mode === 'numeric-arms') this._activateDexarm(cfg, host);
    else if (this.mode === 'quad-legs') this._activateQuad(cfg, host);
    else if (this.mode === 'limbs') this._activateHumanoid(cfg, host);
    else if (this.mode === 'mixer') this._activateDrone(cfg, host);
    else if (this.mode === 'ackermann') this._activateRover(cfg, host);
    else {
      host.innerHTML = '<p class="help-text">IK target control for this robot is on the dedicated panel below.</p>';
      document.getElementById('ik-status-text').textContent = 'IK Idle';
    }
  }

  _activateArm(cfg, host) {
    host.innerHTML = `
      <div class="ik-grid" id="ik-fields"></div>
      <p class="help-text">Drag the gizmo or type a pose. DLS solver tracks live.</p>`;
    this._buildPoseFields(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']);
    document.getElementById('btn-ik-mode').style.display = '';

    // limits in radians so the solver clamps to what the sliders allow
    const limits = cfg.jointLimits.slice(0, 6).map(L =>
      L.isAngle ? { min: L.min * DEG, max: L.max * DEG } : { min: L.min, max: L.max });
    const chain = new DHChain((q) => cfg.kinematics.rows(q, cfg.params), limits);

    // place target at the current EE pose (position AND orientation)
    const f = cfg.fk(cfg.joints, cfg.params);
    this.target.position.set(f.x, f.y, f.z);
    const { ee } = chain.fk(cfg.joints.slice(0, 6));
    const e0 = ee;
    const Rdh0 = new THREE.Matrix4().set(
      e0[0][0], e0[0][1], e0[0][2], 0,
      e0[1][0], e0[1][1], e0[1][2], 0,
      e0[2][0], e0[2][1], e0[2][2], 0,
      0, 0, 0, 1,
    );
    const Rw0 = C.clone().multiply(Rdh0).multiply(CT);
    this.target.quaternion.setFromRotationMatrix(Rw0);
    this.target.visible = true;
    this.deps.viewport.attachGizmo(this.target);
    this._writeFieldsFromTarget();

    this.solver = (t) => {
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
      // gizmo tracking re-solves from the last pose each frame; holding the
      // target orientation while position moves needs more iterations than
      // the solver default to settle both error terms
      const r = chain.solveIK(cfg.joints.slice(0, 6), dhPos, targetRot, { maxIter: 300 });
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

  _activateQuad(cfg, host) {
    host.innerHTML = `
      <p class="help-text">Drag the body — feet stay planted (per-leg analytical IK). Toggle gizmo mode for body rotation.</p>
      <button class="mini-btn" id="btn-quad-reset">Reset Stance</button>`;
    document.getElementById('btn-ik-mode').style.display = '';

    // geometry read live from cfg.params so dimension sliders stay accurate
    const geom = () => ({
      legs: cfg.kinematics.legs(cfg.params),
      F: cfg.params.femur,
      T: cfg.params.tibia + cfg.kinematics.footOffset,
      Cx: cfg.params.coxa,
    });

    // record stance: world foot positions at current joints (legFK from kinematics.js)
    const captureStance = () => {
      const { legs, F, T, Cx } = geom();
      return legs.map(leg => {
        const q = cfg.joints.slice(leg.joint0, leg.joint0 + 3);
        const p = legFK(q[0], q[1], q[2], Cx, F, T, leg.side);
        return [leg.hip[0] + p.x, leg.hip[1] + p.y, leg.hip[2] + p.z];
      });
    };
    let stance = captureStance();

    this.target.position.set(0, geom().legs[0].hip[1], 0);
    this.target.rotation.set(0, 0, 0);
    this.target.visible = true;
    this.deps.viewport.attachGizmo(this.target, 'translate');

    document.getElementById('btn-quad-reset').addEventListener('click', () => {
      // re-plant the feet where they stand now (joints may have been slid)
      stance = captureStance();
      this.target.position.set(0, geom().legs[0].hip[1], 0);
      this.target.rotation.set(0, 0, 0);
      this.solvePending = true;
    });

    this.solver = (t) => {
      const { legs, F, T, Cx } = geom();
      const H0 = legs[0].hip[1];
      // builder applies _bodyPose on bodyGroup at the root origin (ground
      // level), so model the same pivot: hips at local (hipX, H0, hipZ),
      // body translation relative to neutral = t.position − (0, H0, 0)
      const B = new THREE.Matrix4().makeRotationFromEuler(t.rotation);
      const shift = new THREE.Vector3(t.position.x, t.position.y - H0, t.position.z);
      const Binv = B.clone().invert();
      let allReachable = true;
      legs.forEach((leg, i) => {
        const hipLocal = new THREE.Vector3(leg.hip[0], leg.hip[1], leg.hip[2]);
        const hipWorld = hipLocal.clone().applyMatrix4(B).add(shift);
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
        x: shift.x, y: shift.y, z: shift.z,
        rx: t.rotation.x, ry: t.rotation.y, rz: t.rotation.z,
      };
      return allReachable;
    };
  }

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
    // dimensions read live so param-slider changes stay accurate
    const geom = () => ({
      arm: kin.arm(cfg.params),
      leg: kin.leg(cfg.params),
      rootY: kin.rootY(cfg.params),
    });

    // the planar limb model assumes torso yaw and shoulder rolls are zero —
    // zero them on entry so targets and COM match the mesh
    cfg.joints[0] = 0;
    cfg.joints[2] = 0;
    cfg.joints[5] = 0;
    this.deps.onJointsChanged();

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
      const { arm, leg, rootY } = geom();
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
      const { arm, leg, rootY } = geom();
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
    this.droneAttitude = { yaw: 0 };
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
      this.droneAttitude.yaw -= yaw * 0.02;
      this.deps.applyJointsLight(); // rebuild for prop spin
      // kinematic attitude response — applied AFTER rebuild (rebuild resets rotation)
      const mesh = this.deps.getCurrentMesh();
      if (mesh) {
        mesh.rotation.x = pitch * 0.35;
        mesh.rotation.z = -roll * 0.35;
        mesh.rotation.y = this.droneAttitude.yaw;
      }
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
    setNub(0, 0);
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
      // slider maps to curvature: 0 = straight, ±100 = tightest turn whose
      // inner-wheel angle stays within the ±50° steer joint limit
      const maxSteer = cfg.jointLimits[1].max * DEG;
      const rTight = g.track / 2 + (g.wheelbase / 2) / Math.tan(maxSteer);
      const radius = v === 0 ? Infinity : (Math.sign(v) * (rTight + (100 - Math.abs(v)) * 8));
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

  _buildPoseFields(labels) {
    const grid = document.getElementById('ik-fields');
    grid.innerHTML = labels.map(l => `
      <div class="ik-field">
        <label for="ikf-${l}">${l}</label>
        <input type="number" id="ikf-${l}" step="${'XYZ'.includes(l) ? 5 : 2}" value="0">
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
    if (this.droneTick && this.mode === 'mixer') this.droneTick();
    if (this.roverTickFn && this.mode === 'ackermann') this.roverTickFn();
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
