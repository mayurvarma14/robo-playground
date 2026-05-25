/**
 * main.js — App entry point and controller
 * Manages UI state, robot switching, joint controls, sequencer, and exports.
 */
import { Viewport } from './viewport.js';
import { ROBOTS } from './robots.js';
import { STLExporter } from 'three/addons/exporters/STLExporter.js';

// ─────────────────────────────────────────────────────────────
// STATE
// ─────────────────────────────────────────────────────────────
const state = {
  activeRobot: 'arm',
  theme: localStorage.getItem('robosim-theme') || 'dark',
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
  renderParamControls();
  renderExportList();
  updateTelemetry();
}

// ─────────────────────────────────────────────────────────────
// THEME
// ─────────────────────────────────────────────────────────────
function applyTheme(theme) {
  state.theme = theme;
  document.documentElement.setAttribute('data-theme', theme);
  localStorage.setItem('robosim-theme', theme);
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
}

// Reset pose
document.getElementById('btn-reset-pose').addEventListener('click', () => {
  const cfg = robots[state.activeRobot];
  const orig = ROBOTS[state.activeRobot].joints;
  cfg.joints = [...orig];
  renderJointControls();
  rebuildCurrentRobot();
});

// ─────────────────────────────────────────────────────────────
// IK CONTROLS
// ─────────────────────────────────────────────────────────────
document.getElementById('btn-ik-solve').addEventListener('click', () => {
  const cfg = robots[state.activeRobot];
  const warning = document.getElementById('ik-warning');

  if (!cfg.ikSupported || !cfg.fk) {
    warning.style.display = 'flex';
    warning.querySelector('span').textContent = ' IK not supported for this robot.';
    return;
  }

  const x = parseFloat(document.getElementById('ik-x').value);
  const y = parseFloat(document.getElementById('ik-y').value);
  const z = parseFloat(document.getElementById('ik-z').value);

  // Simple FABRIK-style IK for the 6-DOF arm (2-link planar)
  const { l2, l3 } = cfg.params;
  const r = Math.sqrt(x * x + z * z);
  const h = y - (cfg.params.l1 + 20);
  const d = Math.sqrt(r * r + h * h);

  if (d > l2 + l3) {
    warning.style.display = 'flex';
    warning.querySelector('span:last-child').textContent = ' Target out of workspace';
    return;
  }

  warning.style.display = 'none';

  // Shoulder/elbow angles (2-link)
  const cos_e = (d * d - l2 * l2 - l3 * l3) / (2 * l2 * l3);
  const elbow = -Math.acos(Math.max(-1, Math.min(1, cos_e)));
  const shoulder = Math.atan2(h, r) - Math.atan2(l3 * Math.sin(elbow), l2 + l3 * Math.cos(elbow));
  const base = Math.atan2(z, x);

  cfg.joints[0] = base;
  cfg.joints[1] = shoulder;
  cfg.joints[2] = elbow;
  cfg.joints[3] = -(shoulder + elbow);

  renderJointControls();
  rebuildCurrentRobot();

  document.getElementById('ik-status-text').textContent = 'IK Solved';
});

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

  if (seq.frames.length === 0) {
    empty.style.display = '';
    return;
  }
  empty.style.display = 'none';

  // Rebuild chips
  timeline.querySelectorAll('.keyframe-chip').forEach(c => c.remove());
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
  cfg.joints = [...seq.frames[idx]];
  renderJointControls();
  rebuildCurrentRobot();
}

document.getElementById('seq-record').addEventListener('click', () => {
  const cfg = robots[state.activeRobot];
  seq.frames.push([...cfg.joints]);
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
  cfg.joints = a.map((av, i) => av + (b[i] - av) * t);
  rebuildCurrentRobot();
}

viewport.registerCallback(tickSequencer);

// ─────────────────────────────────────────────────────────────
// INIT
// ─────────────────────────────────────────────────────────────
buildAndShowRobot(state.activeRobot);
