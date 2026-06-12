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
    this.solver = null;        // drop the previous robot's solver closure
    this.solvePending = false; // and any solve queued during the switch
    this.fieldLabels = null;
    this._setWarning(false);
    document.getElementById('btn-ik-mode').style.display = 'none';
    this.mode = cfg.kinematics?.type ?? null;

    if (this.mode === 'dh') this._activateArm(cfg, host);
    else if (this.mode === 'scara') this._activateScara(cfg, host);
    else if (this.mode === 'numeric-arms') this._activateDexarm(cfg, host);
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
