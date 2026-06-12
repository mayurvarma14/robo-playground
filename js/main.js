/**
 * main.js — App entry point and controller
 * Manages UI state, robot switching, joint controls, sequencer, and exports.
 */
import * as THREE from 'three';
import { Viewport } from './viewport.js';
import { ROBOTS } from './robots.js';
import { DHChain, dhToWorld } from './kinematics.js';
import { IKController } from './ik-control.js';
import { ExpertPanel } from './expert-panel.js';
import { STLExporter } from 'three/addons/exporters/STLExporter.js';

if (new URLSearchParams(location.search).has('test')) import('./tests.js');

// ─────────────────────────────────────────────────────────────
// STATE
// ─────────────────────────────────────────────────────────────
const state = {
  activeRobot: 'arm',
  theme: localStorage.getItem('robo-playground-theme') || 'dark',
  wireframe: false,
  sequence: {
    frames: [],
    playing: false,
    currentFrame: 0,
    speed: 1.0,
    lastTime: 0,
    progress: 0,
  },
};

// Deep-clone robot configs into app state so we can modify joints/params
const robots = {};
for (const [key, cfg] of Object.entries(ROBOTS)) {
  robots[key] = {
    ...cfg,
    joints: [...cfg.joints],
    params: { ...cfg.params },
  };
}

// ─────────────────────────────────────────────────────────────
// VIEWPORT
// ─────────────────────────────────────────────────────────────
const viewport = new Viewport();
viewport.init('canvas-container');

let currentMesh = null;

function buildAndShowRobot(key) {
  if (currentMesh) {
    viewport.removeObject(currentMesh);
    currentMesh = null;
  }

  const cfg = robots[key];
  currentMesh = cfg.builder(cfg.joints, cfg.params);
  viewport.addObject(currentMesh);

  // Update name badge
  document.getElementById('active-robot-name').textContent = cfg.name;

  // Rebuild right-panel controls
  renderJointControls();
  ik.activate(key);
  renderParamControls();
  renderExportList();
  updateTelemetry();
  renderFrameTriads(); // ik.activate cleared kinHelpers — rebuild for new robot
  expertPanel.update(robots[state.activeRobot], { mix: ik.lastMix, com: ik.lastCOM });

  // recorded frames belong to the previous robot's joint layout
  state.sequence.frames = [];
  state.sequence.playing = false;
  state.sequence.currentFrame = 0;
  const playIcon = document.getElementById('seq-play-icon');
  if (playIcon) playIcon.className = 'fa-solid fa-play';
  updateSeqUI();
}

// ─────────────────────────────────────────────────────────────
// THEME
// ─────────────────────────────────────────────────────────────
function applyTheme(theme) {
  state.theme = theme;
  document.documentElement.setAttribute('data-theme', theme);
  localStorage.setItem('robo-playground-theme', theme);
  viewport.setTheme(theme);

  const icon = document.getElementById('theme-icon');
  icon.className = theme === 'light' ? 'fa-solid fa-moon' : 'fa-solid fa-sun';
}

// Apply saved theme on load
applyTheme(state.theme);

document.getElementById('btn-theme').addEventListener('click', () => {
  applyTheme(state.theme === 'dark' ? 'light' : 'dark');
});

// ─────────────────────────────────────────────────────────────
// ROBOT PICKER
// ─────────────────────────────────────────────────────────────
document.querySelectorAll('.robot-card').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.robot-card').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    state.activeRobot = btn.dataset.robot;
    buildAndShowRobot(state.activeRobot);
  });
});

// ─────────────────────────────────────────────────────────────
// TOOLBAR
// ─────────────────────────────────────────────────────────────
['iso','front','side','top'].forEach(view => {
  const btn = document.getElementById(`btn-view-${view}`);
  if (!btn) return;
  btn.addEventListener('click', () => {
    document.querySelectorAll('#viewport-toolbar .toolbar-btn').forEach(b => {
      if (['btn-view-iso','btn-view-front','btn-view-side','btn-view-top'].includes(b.id)) {
        b.classList.remove('active');
      }
    });
    btn.classList.add('active');
    viewport.setView(view);
  });
});

const btnGrid = document.getElementById('btn-grid');
btnGrid.addEventListener('click', () => {
  btnGrid.classList.toggle('active');
  viewport.setGridVisible(btnGrid.classList.contains('active'));
});

const btnAxes = document.getElementById('btn-axes');
btnAxes.addEventListener('click', () => {
  btnAxes.classList.toggle('active');
  viewport.setAxesVisible(btnAxes.classList.contains('active'));
});

const btnWire = document.getElementById('btn-wireframe');
btnWire.addEventListener('click', () => {
  btnWire.classList.toggle('active');
  state.wireframe = btnWire.classList.contains('active');
  viewport.setWireframe(state.wireframe);
});

// ── Joint frame triads ──
// Triads live in their own cached subgroup inside viewport.kinHelpers so
// toggling/refreshing them never touches the IK controller's COM marker
// (which shares kinHelpers), and triad geometries/materials are reused
// across joint changes instead of being recreated (and leaked) per rebuild.
let showFrames = false;
const triadGroup = new THREE.Group();
const triadPool = [];
const triadMat = new THREE.Matrix4();

const btnFrames = document.getElementById('btn-frames');
btnFrames.addEventListener('click', () => {
  showFrames = !showFrames;
  btnFrames.classList.toggle('active', showFrames);
  renderFrameTriads();
});

function renderFrameTriads() {
  triadGroup.visible = showFrames;
  if (!showFrames) return;
  // ik.activate() clears kinHelpers on robot switch — re-attach if detached
  if (!triadGroup.parent) viewport.addKinHelper(triadGroup);

  const cfg = robots[state.activeRobot];
  const kin = cfg.kinematics;
  if (!kin?.rows) { // triads only for DH robots this cycle
    for (const t of triadPool) t.visible = false;
    return;
  }

  const chain = new DHChain((q) => kin.rows(q, cfg.params));
  const n = kin.rows(cfg.joints, cfg.params).length;
  const { frames } = chain.fk(cfg.joints.slice(0, n));

  while (triadPool.length < frames.length) {
    const t = viewport.makeTriad();
    triadPool.push(t);
    triadGroup.add(t);
  }
  triadPool.forEach((t, i) => { t.visible = i < frames.length; });

  frames.forEach((T, i) => {
    const triad = triadPool[i];
    const [x, y, z] = dhToWorld([T[0][3], T[1][3], T[2][3]]);
    triad.position.set(x, y, z);
    // orientation: world R = C · R_dh (row permutation), so the triad's
    // columns are the DH frame axes mapped through (x,y,z)→(x,z,−y) —
    // blue stays the joint's DH z-axis
    triadMat.set(
      T[0][0], T[0][1], T[0][2], 0,
      T[2][0], T[2][1], T[2][2], 0,
      -T[1][0], -T[1][1], -T[1][2], 0,
      0, 0, 0, 1
    );
    triad.quaternion.setFromRotationMatrix(triadMat);
  });
}

document.getElementById('btn-reset-cam').addEventListener('click', () => {
  viewport.setView('iso');
  document.getElementById('btn-view-iso').classList.add('active');
  ['btn-view-front','btn-view-side','btn-view-top'].forEach(id => {
    document.getElementById(id)?.classList.remove('active');
  });
});

// ─────────────────────────────────────────────────────────────
// PANEL TABS
// ─────────────────────────────────────────────────────────────
document.querySelectorAll('.panel-tab').forEach(tab => {
  tab.addEventListener('click', () => {
    document.querySelectorAll('.panel-tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById(`pane-${tab.dataset.tab}`).classList.add('active');
  });
});

// ─────────────────────────────────────────────────────────────
// JOINT CONTROLS
// ─────────────────────────────────────────────────────────────
function renderJointControls() {
  const container = document.getElementById('joint-controls-container');
  container.innerHTML = '';

  const cfg = robots[state.activeRobot];

  cfg.joints.forEach((val, i) => {
    const limits = cfg.jointLimits[i];
    const name   = cfg.jointNames[i];
    let dispVal  = limits.isAngle ? Math.round(val * 180 / Math.PI) : Math.round(val);

    const row = document.createElement('div');
    row.className = 'joint-row';
    row.innerHTML = `
      <div class="joint-label-row">
        <span class="joint-name">${name}</span>
        <input type="number" class="joint-val-input" id="jnum-${i}"
          min="${limits.min}" max="${limits.max}" step="${limits.step}" value="${dispVal}">
      </div>
      <input type="range" id="jslider-${i}"
        min="${limits.min}" max="${limits.max}" step="${limits.step}" value="${dispVal}">
    `;
    container.appendChild(row);

    const slider = row.querySelector(`#jslider-${i}`);
    const numIn  = row.querySelector(`#jnum-${i}`);

    const updateJoint = (v) => {
      v = Math.max(limits.min, Math.min(limits.max, v));
      cfg.joints[i] = limits.isAngle ? v * Math.PI / 180 : v;
      slider.value = v;
      numIn.value  = v;
      const span = limits.max - limits.min;
      row.classList.toggle('at-limit', v <= limits.min + span * 0.002 || v >= limits.max - span * 0.002);
      rebuildCurrentRobot();
    };

    slider.addEventListener('input', e => updateJoint(parseFloat(e.target.value)));
    numIn.addEventListener('change', e => updateJoint(parseFloat(e.target.value) || dispVal));
  });
}

function rebuildCurrentRobot() {
  if (currentMesh) {
    viewport.removeObject(currentMesh);
    currentMesh = null;
  }
  const cfg = robots[state.activeRobot];
  currentMesh = cfg.builder(cfg.joints, cfg.params);
  viewport.addObject(currentMesh);
  if (state.wireframe) viewport.setWireframe(true);
  updateTelemetry();
  renderExportList();
  renderFrameTriads();
  ik.refreshCOM(cfg);
  expertPanel.update(robots[state.activeRobot], { mix: ik.lastMix, com: ik.lastCOM });
}

// Reset pose
document.getElementById('btn-reset-pose').addEventListener('click', () => {
  const cfg = robots[state.activeRobot];
  const orig = ROBOTS[state.activeRobot].joints;
  cfg.joints = [...orig];
  delete cfg.params._bodyPose; // body returns to neutral along with the legs
  renderJointControls();
  rebuildCurrentRobot();
});

// ─────────────────────────────────────────────────────────────
// IK CONTROLS
// ─────────────────────────────────────────────────────────────
function syncJointInputs() {
  const cfg = robots[state.activeRobot];
  cfg.joints.forEach((val, i) => {
    const limits = cfg.jointLimits[i];
    const disp = limits.isAngle ? val / Math.PI * 180 : val;
    const slider = document.getElementById(`jslider-${i}`);
    const numIn = document.getElementById(`jnum-${i}`);
    if (slider) slider.value = disp;
    if (numIn && document.activeElement !== numIn) numIn.value = Math.round(disp);
    // limit badge: flag the row when the solver clamped this joint at a limit
    if (slider) {
      const span = limits.max - limits.min;
      const atLimit = disp <= limits.min + span * 0.002 || disp >= limits.max - span * 0.002;
      slider.closest('.joint-row')?.classList.toggle('at-limit', atLimit);
    }
  });
}

const ik = new IKController({
  viewport,
  robots,
  getActiveKey: () => state.activeRobot,
  onJointsChanged: () => { syncJointInputs(); rebuildCurrentRobot(); },
  getCurrentMesh: () => currentMesh,
  applyJointsLight: () => rebuildCurrentRobot(),
});

// ─────────────────────────────────────────────────────────────
// EXPERT PANEL
// ─────────────────────────────────────────────────────────────
const expertPanel = new ExpertPanel();

// ─────────────────────────────────────────────────────────────
// PARAM CONTROLS (Config tab)
// ─────────────────────────────────────────────────────────────
function renderParamControls() {
  const container = document.getElementById('param-controls-container');
  container.innerHTML = '';

  const cfg = robots[state.activeRobot];
  for (const pd of cfg.paramDefs) {
    const val = cfg.params[pd.key];
    const row = document.createElement('div');
    row.className = 'param-row';
    row.innerHTML = `
      <label>${pd.label} <span class="param-val" id="pval-${pd.key}">${val}${pd.unit}</span></label>
      <input type="range" id="pslider-${pd.key}"
        min="${pd.min}" max="${pd.max}" step="${pd.step}" value="${val}">
    `;
    container.appendChild(row);

    row.querySelector(`#pslider-${pd.key}`).addEventListener('input', e => {
      const v = parseFloat(e.target.value);
      cfg.params[pd.key] = v;
      document.getElementById(`pval-${pd.key}`).textContent = `${v}${pd.unit}`;
      rebuildCurrentRobot();
    });
  }
}

// Hardware tolerances
const boltSlider = document.getElementById('bolt-slider');
boltSlider.addEventListener('input', e => {
  document.getElementById('val-bolt').textContent = `${parseFloat(e.target.value).toFixed(1)}mm`;
});

const wallSlider = document.getElementById('wall-slider');
wallSlider.addEventListener('input', e => {
  document.getElementById('val-wall').textContent = `${parseFloat(e.target.value).toFixed(1)}mm`;
});

// Motor select description
const motorDescriptions = {
  sg90:   '22.8×12.2mm servo · ~1.8kg·cm torque',
  mg996r: '40.7×19.7mm servo · ~10kg·cm torque',
  nema17: '42.3×42.3mm stepper · NEMA standard',
};
document.getElementById('motor-select').addEventListener('change', e => {
  document.getElementById('motor-desc').textContent = motorDescriptions[e.target.value] || '';
});

// Brightness
document.getElementById('brightness-slider').addEventListener('input', e => {
  const v = parseFloat(e.target.value);
  document.getElementById('val-brightness').textContent = v.toFixed(1);
  viewport.setBrightness(v);
});

// ─────────────────────────────────────────────────────────────
// TELEMETRY
// ─────────────────────────────────────────────────────────────
function updateTelemetry() {
  const cfg = robots[state.activeRobot];
  let pos = { x: 0, y: 0, z: 0 };

  if (cfg.fk) {
    try { pos = cfg.fk(cfg.joints, cfg.params); } catch(e) {}
  }

  document.getElementById('tel-x').textContent = pos.x.toFixed(1);
  document.getElementById('tel-y').textContent = pos.y.toFixed(1);
  document.getElementById('tel-z').textContent = pos.z.toFixed(1);
}

// ─────────────────────────────────────────────────────────────
// STL EXPORT
// ─────────────────────────────────────────────────────────────
const exporter = new STLExporter();

function exportMesh(mesh, filename) {
  const stl = exporter.parse(mesh, { binary: true });
  const blob = new Blob([stl], { type: 'application/octet-stream' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `${filename}.stl`;
  a.click();
  URL.revokeObjectURL(a.href);
}

function renderExportList() {
  const container = document.getElementById('export-parts-list');
  container.innerHTML = '';

  if (!currentMesh) return;

  const parts = [];
  currentMesh.traverse(obj => {
    if (obj.isMesh && obj.name && !parts.some(p => p.name === obj.name)) {
      if (!obj.name.toLowerCase().includes('prop blade') &&
          !obj.name.toLowerCase().includes('spoke')) {
        parts.push(obj);
      }
    }
  });

  if (parts.length === 0) {
    container.innerHTML = '<p class="help-text">No named parts found.</p>';
    return;
  }

  for (const part of parts.slice(0, 15)) {
    const item = document.createElement('div');
    item.className = 'export-item';
    item.innerHTML = `
      <div>
        <div class="export-item-name">${part.name}</div>
        <div class="export-item-spec">${wallSlider.value}mm walls · M${Math.floor(parseFloat(boltSlider.value))} holes</div>
      </div>
      <button class="export-dl-btn" title="Download STL"><i class="fa-solid fa-download"></i></button>
    `;
    item.querySelector('.export-dl-btn').addEventListener('click', () => {
      exportMesh(part, `${state.activeRobot}_${part.name.replace(/\s+/g, '_').toLowerCase()}`);
    });
    container.appendChild(item);
  }
}

document.getElementById('btn-export-all').addEventListener('click', () => {
  if (currentMesh) {
    exportMesh(currentMesh, `${state.activeRobot}_full_assembly`);
  }
});

// ─────────────────────────────────────────────────────────────
// POSE SEQUENCER
// ─────────────────────────────────────────────────────────────
const seq = state.sequence;

function updateSeqUI() {
  const timeline = document.getElementById('seq-timeline');
  const empty    = document.getElementById('seq-empty-msg');

  // Rebuild chips
  timeline.querySelectorAll('.keyframe-chip').forEach(c => c.remove());

  if (seq.frames.length === 0) {
    empty.style.display = '';
    return;
  }
  empty.style.display = 'none';
  seq.frames.forEach((frame, idx) => {
    const chip = document.createElement('div');
    chip.className = `keyframe-chip${idx === seq.currentFrame ? ' active' : ''}`;
    chip.innerHTML = `
      <span>Frame ${idx + 1}</span>
      <span class="keyframe-del" data-idx="${idx}">✕</span>
    `;
    chip.addEventListener('click', e => {
      if (e.target.classList.contains('keyframe-del')) {
        const i = parseInt(e.target.dataset.idx);
        seq.frames.splice(i, 1);
        if (seq.currentFrame >= seq.frames.length) seq.currentFrame = Math.max(0, seq.frames.length - 1);
        updateSeqUI();
        return;
      }
      seq.currentFrame = idx;
      applyFrame(idx);
      updateSeqUI();
    });
    timeline.appendChild(chip);
  });
}

function applyFrame(idx) {
  if (idx < 0 || idx >= seq.frames.length) return;
  const cfg = robots[state.activeRobot];
  const frame = seq.frames[idx];
  cfg.joints = [...frame.joints];
  if (frame.bodyPose) cfg.params._bodyPose = { ...frame.bodyPose };
  else delete cfg.params._bodyPose;
  renderJointControls();
  rebuildCurrentRobot();
}

const BODY_POSE_KEYS = ['x', 'y', 'z', 'rx', 'ry', 'rz'];

// lerp two body poses; null counts as the neutral (all-zero) pose
function lerpBodyPose(a, b, t) {
  if (!a && !b) return null;
  const out = {};
  for (const k of BODY_POSE_KEYS) {
    const av = a?.[k] || 0, bv = b?.[k] || 0;
    out[k] = av + (bv - av) * t;
  }
  return out;
}

document.getElementById('seq-record').addEventListener('click', () => {
  const cfg = robots[state.activeRobot];
  seq.frames.push({
    joints: [...cfg.joints],
    bodyPose: cfg.params._bodyPose ? { ...cfg.params._bodyPose } : null,
  });
  seq.currentFrame = seq.frames.length - 1;
  updateSeqUI();
});

document.getElementById('seq-clear').addEventListener('click', () => {
  seq.frames = [];
  seq.playing = false;
  seq.currentFrame = 0;
  document.getElementById('seq-play-icon').className = 'fa-solid fa-play';
  updateSeqUI();
});

document.getElementById('seq-prev').addEventListener('click', () => {
  if (seq.frames.length < 2) return;
  seq.playing = false;
  document.getElementById('seq-play-icon').className = 'fa-solid fa-play';
  seq.currentFrame = (seq.currentFrame - 1 + seq.frames.length) % seq.frames.length;
  applyFrame(seq.currentFrame);
  updateSeqUI();
});

document.getElementById('seq-next').addEventListener('click', () => {
  if (seq.frames.length < 2) return;
  seq.playing = false;
  document.getElementById('seq-play-icon').className = 'fa-solid fa-play';
  seq.currentFrame = (seq.currentFrame + 1) % seq.frames.length;
  applyFrame(seq.currentFrame);
  updateSeqUI();
});

document.getElementById('seq-play').addEventListener('click', () => {
  if (seq.frames.length < 2) return;
  seq.playing = !seq.playing;
  if (seq.playing) {
    seq.lastTime = performance.now();
    seq.progress = 0;
  }
  document.getElementById('seq-play-icon').className = seq.playing ? 'fa-solid fa-pause' : 'fa-solid fa-play';
});

document.getElementById('seq-speed').addEventListener('input', e => {
  seq.speed = parseFloat(e.target.value);
  document.getElementById('seq-speed-val').textContent = `${seq.speed.toFixed(1)}×`;
});

// Called every frame by viewport
function tickSequencer(time) {
  if (!seq.playing || seq.frames.length < 2) return;

  const now   = performance.now();
  const delta = (now - seq.lastTime) / 1000;
  seq.lastTime = now;
  seq.progress += delta * seq.speed;

  if (seq.progress >= 1) {
    seq.progress = 0;
    seq.currentFrame = (seq.currentFrame + 1) % seq.frames.length;
    updateSeqUI();
  }

  // Interpolate
  const next = (seq.currentFrame + 1) % seq.frames.length;
  const cfg  = robots[state.activeRobot];
  const a    = seq.frames[seq.currentFrame];
  const b    = seq.frames[next];

  const t = seq.progress;
  cfg.joints = a.joints.map((av, i) => av + (b.joints[i] - av) * t);
  const bp = lerpBodyPose(a.bodyPose, b.bodyPose, t);
  if (bp) cfg.params._bodyPose = bp;
  else delete cfg.params._bodyPose;
  rebuildCurrentRobot();
}

viewport.registerCallback(tickSequencer);

// ─────────────────────────────────────────────────────────────
// INIT
// ─────────────────────────────────────────────────────────────
buildAndShowRobot(state.activeRobot);
