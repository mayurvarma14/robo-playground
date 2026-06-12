/**
 * expert-panel.js — live DH table, EE pose, transform matrix,
 * manipulability, per-robot kinematic readouts.
 */
import { DHChain, dhToWorld, legFK, DEG } from './kinematics.js';

// Empirical full-scale for the manipulability bar: the half-scale UR5e arm
// measures w ≈ 5×10⁵ in healthy mid-workspace poses (mm³/rad³ units), so 10⁶
// puts those around half-bar and the <12% danger zone near real singularities.
const MANIP_FULL_SCALE = 1e6;

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
      const pct = Math.min(100, w / MANIP_FULL_SCALE * 100);
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
