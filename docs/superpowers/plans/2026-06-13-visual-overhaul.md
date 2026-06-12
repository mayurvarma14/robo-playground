# Visual Overhaul Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Professional-grade visuals for all 7 robots via a studio rendering pipeline (environment reflections, rebalanced lighting, reflective floor) and a geometry polish pass — zero external assets, kinematics/IK/tests untouched.

**Architecture:** Part A upgrades `viewport.js` (PMREM RoomEnvironment, light rig rebalance, floor) and `materials.js` (clearcoat physical materials, retuned PBR). Part B adds shared detail-geometry helpers to `robots.js` and reshells each builder visually while preserving every joint group, mesh name, param, and transform.

**Tech Stack:** three.js 0.160 built-in addons only — `RoomEnvironment`, `RoundedBoxGeometry`, `PMREMGenerator`, `CapsuleGeometry`, `LatheGeometry`.

**Verification ground rules (every task):** reload `http://localhost:8080/?test=1` → `TESTS: 39 passed, 0 failed`; reload without `?test` → zero console errors; screenshot the affected robot(s) and judge against the goal "does this read as a product photo, not a toy?". The geometry tasks are expected to need 1–3 visual iterations — tune dimensions/colors after looking, then re-screenshot.

---

### Task 1: Environment reflections + light rig rebalance

**Files:**
- Modify: `js/viewport.js`

- [ ] **Step 1: Add the environment map.** Import at top of `js/viewport.js`:

```js
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
```

In `init()`, right after the renderer is appended to the container (after the `this.container.appendChild(...)` line):

```js
    // Studio environment — gives PBR metals real reflections (no asset files)
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.envTexture = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    pmrem.dispose();
    this.scene.environment = this.envTexture;
```

- [ ] **Step 2: Rebalance the light rig.** Replace the whole `_buildLighting()` body with:

```js
  _buildLighting() {
    // The environment map supplies ambient + fill; physical lights only shape
    // form (key) and separate the silhouette (rim).
    this.hemiLight = new THREE.HemisphereLight(0xc8ddf0, 0x6a7686, 0.35);
    this.hemiLight.position.set(0, 500, 0);
    this.scene.add(this.hemiLight);

    this.sunLight = new THREE.DirectionalLight(0xfff4e8, 2.4);
    this.sunLight.position.set(350, 600, 300);
    this.sunLight.castShadow = true;
    this.sunLight.shadow.mapSize.set(2048, 2048);
    this.sunLight.shadow.camera.near   = 10;
    this.sunLight.shadow.camera.far    = 2000;
    this.sunLight.shadow.camera.left   = -400;
    this.sunLight.shadow.camera.right  = 400;
    this.sunLight.shadow.camera.top    = 700;
    this.sunLight.shadow.camera.bottom = -200;
    this.sunLight.shadow.bias = -0.0002;
    this.sunLight.shadow.radius = 4;
    this.scene.add(this.sunLight);

    this.rimLight = new THREE.DirectionalLight(0xbcd8ff, 0.9);
    this.rimLight.position.set(-250, 300, -450);
    this.scene.add(this.rimLight);
  }
```

Note: `ambientLight`, `fillLight`, `backLight` properties die. Remove their constructor
initialisations and every reference in `_applySceneTheme` and `setBrightness`.

- [ ] **Step 3: Update `_applySceneTheme` light blocks** — replace the per-theme intensity assignments with:

```js
    if (theme === 'light') {
      ...existing background/fog/platform/grid lines stay...
      this.renderer.toneMappingExposure = 1.15;
      if (this.hemiLight) this.hemiLight.intensity = 0.5;
      if (this.sunLight)  this.sunLight.intensity  = 2.8;
      if (this.rimLight)  this.rimLight.intensity  = 0.7;
    } else {
      ...existing background/fog/platform/grid lines stay...
      this.renderer.toneMappingExposure = 1.0;
      if (this.hemiLight) this.hemiLight.intensity = 0.35;
      if (this.sunLight)  this.sunLight.intensity  = 2.4;
      if (this.rimLight)  this.rimLight.intensity  = 0.9;
    }
```

(Per-theme brightness rides `toneMappingExposure`, NOT `Scene.environmentIntensity` —
that property only landed in three r163 and this project pins r160.)

- [ ] **Step 4: Update `setBrightness`** to only touch the three surviving lights:

```js
  setBrightness(multiplier) {
    const isLight = this.theme === 'light';
    if (this.sunLight)  this.sunLight.intensity  = multiplier * (isLight ? 2.8 : 2.4);
    if (this.hemiLight) this.hemiLight.intensity = multiplier * (isLight ? 0.5 : 0.35);
    if (this.rimLight)  this.rimLight.intensity  = multiplier * (isLight ? 0.7 : 0.9);
  }
```

- [ ] **Step 5: Verify** — tests 39 green; app loads; metals (chrome joints, aluminium links) now show environment reflections; nothing blown out in either theme. Screenshot arm in both themes.

- [ ] **Step 6: Commit** — `git add js/viewport.js && git commit -m "feat: add studio environment reflections and rebalance light rig"`

---

### Task 2: Floor + contact shadow

**Files:**
- Modify: `js/viewport.js` (`_buildFloor`, `_applySceneTheme`)

- [ ] **Step 1: Replace `_buildFloor()` body:**

```js
  _buildFloor() {
    // Soft shadow catcher
    this.floorMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(6000, 6000),
      new THREE.ShadowMaterial({ opacity: 0.25 })
    );
    this.floorMesh.rotation.x = -Math.PI / 2;
    this.floorMesh.receiveShadow = true;
    this.scene.add(this.floorMesh);

    // Studio floor — dark, slightly glossy so it picks up the environment
    this.studioFloor = new THREE.Mesh(
      new THREE.CircleGeometry(2800, 64),
      new THREE.MeshStandardMaterial({ color: 0x141c28, roughness: 0.35, metalness: 0.55 })
    );
    this.studioFloor.rotation.x = -Math.PI / 2;
    this.studioFloor.position.y = -8.5;
    this.studioFloor.receiveShadow = true;
    this.scene.add(this.studioFloor);

    // Contact-shadow vignette under the robot (radial gradient canvas)
    const cv = document.createElement('canvas');
    cv.width = cv.height = 256;
    const ctx = cv.getContext('2d');
    const grad = ctx.createRadialGradient(128, 128, 20, 128, 128, 128);
    grad.addColorStop(0, 'rgba(0,0,0,0.45)');
    grad.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 256, 256);
    this.contactShadow = new THREE.Mesh(
      new THREE.PlaneGeometry(640, 640),
      new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(cv), transparent: true, depthWrite: false })
    );
    this.contactShadow.rotation.x = -Math.PI / 2;
    this.contactShadow.position.y = 0.2;
    this.scene.add(this.contactShadow);

    // Platform disc — brushed bezel ring + face
    this.platformMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(220, 230, 8, 96),
      new THREE.MeshStandardMaterial({ color: 0x2c3a50, roughness: 0.45, metalness: 0.6 })
    );
    this.platformMesh.position.y = -4;
    this.platformMesh.receiveShadow = true;
    this.scene.add(this.platformMesh);

    this.platformRing = new THREE.Mesh(
      new THREE.TorusGeometry(222, 2.5, 12, 96),
      new THREE.MeshStandardMaterial({ color: 0x9fb4cc, roughness: 0.25, metalness: 0.9 })
    );
    this.platformRing.rotation.x = -Math.PI / 2;
    this.platformRing.position.y = 0.5;
    this.scene.add(this.platformRing);

    // Grid
    this.gridHelper = new THREE.GridHelper(1200, 30, 0x4a6080, 0x2a3a50);
    this.gridHelper.position.y = 0.5;
    this.scene.add(this.gridHelper);

    // Axes helper
    this.axesHelper = new THREE.AxesHelper(160);
    this.axesHelper.position.set(-460, 1, -460);
    this.scene.add(this.axesHelper);
  }
```

- [ ] **Step 2: Theme the studio floor** in `_applySceneTheme`: light theme `this.studioFloor.material.color.setHex(0x9aa8ba); roughness 0.5`; dark theme `0x141c28; roughness 0.35`. Add `studioFloor`, `contactShadow`, `platformRing` to `_isHelper`? — NO: they should wireframe-toggle like scenery? They currently would. Exclude them the same way the platform behaves today (wireframe toggles scenery too — pre-existing behavior, leave as is).

- [ ] **Step 3: Verify** — floor shows a soft reflection-like sheen, vignette under robot, both themes look intentional. Tests green. Screenshot.

- [ ] **Step 4: Commit** — `git commit -am "feat: studio floor with contact shadow vignette"`

---

### Task 3: Material library retune

**Files:**
- Modify: `js/materials.js` (full rewrite of the `MAT` table)

- [ ] **Step 1: Rewrite `materials.js`:**

```js
/**
 * materials.js
 * Shared PBR material library for all robot parts.
 * Tuned against the RoomEnvironment studio reflections.
 */
import * as THREE from 'three';

function mat(color, roughness, metalness, extra = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness, ...extra });
}

// Clearcoated panel — automotive/consumer-product shell look
function panel(color, roughness = 0.4, extra = {}) {
  return new THREE.MeshPhysicalMaterial({
    color, roughness, metalness: 0.0,
    clearcoat: 0.8, clearcoatRoughness: 0.25, ...extra,
  });
}

// Subtle anisotropic-ish roughness variation for brushed metals
function brushedTexture() {
  const cv = document.createElement('canvas');
  cv.width = 256; cv.height = 256;
  const ctx = cv.getContext('2d');
  ctx.fillStyle = '#808080';
  ctx.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 1400; i++) {
    const y = Math.random() * 256;
    ctx.strokeStyle = `rgba(${Math.random() > 0.5 ? 255 : 0},0,0,0.04)`;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(256, y + (Math.random() - 0.5) * 4); ctx.stroke();
  }
  const tex = new THREE.CanvasTexture(cv);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  return tex;
}
const brushed = brushedTexture();

export const MAT = {
  // ── Structural frames
  darkSteel:     mat(0x2a3444, 0.45, 0.85),
  aluminium:     mat(0xc4ccd6, 0.28, 0.9, { roughnessMap: brushed }),
  blackAnodised: mat(0x161e2a, 0.32, 0.8),
  titanium:      mat(0x8b97a6, 0.24, 0.92, { roughnessMap: brushed }),

  // ── Body panels / covers (clearcoat shells)
  whitePolycarbonate: panel(0xeef1f5, 0.35),
  darkPolycarbonate:  panel(0x222c3e, 0.4),
  carbonFiber:        mat(0x16161e, 0.42, 0.55, { roughnessMap: brushed }),

  // ── Actuators / joints
  chrome:    mat(0xe8eef4, 0.06, 1.0),
  brass:     mat(0xc8952f, 0.22, 0.95),
  copper:    mat(0xbf5a28, 0.25, 0.95),
  steelDark: mat(0x3c4658, 0.35, 0.85),

  // ── Wheels / tyres
  rubber: mat(0x16191f, 0.92, 0.0),

  // ── Accent / glow
  cyan:   mat(0x18b8d8, 0.3, 0.2, { emissive: 0x0a7a96, emissiveIntensity: 0.9 }),
  orange: mat(0xf2602a, 0.35, 0.1, { emissive: 0x7a2406, emissiveIntensity: 0.5 }),
  green:  mat(0x26c862, 0.35, 0.1, { emissive: 0x0c5e2e, emissiveIntensity: 0.6 }),

  // ── Environment
  floor:    mat(0x1e2a3a, 0.8, 0.1),
  platform: mat(0x263040, 0.75, 0.15),
};

for (const m of Object.values(MAT)) {
  m.side = THREE.FrontSide;
}
```

- [ ] **Step 2: Verify** — every robot, both themes: whites are clean (not grey mud), chrome pops, anodised reads dark-metal not black-plastic. Tests green (materials not under test, but module must load). Screenshot arm + humanoid.

- [ ] **Step 3: Commit** — `git commit -am "feat: retune material library for studio environment, clearcoat panels"`

---

### Task 4: Shared detail-geometry helpers

**Files:**
- Modify: `js/robots.js` (helper section near the existing `box`/`cyl`/`sphere` helpers)

- [ ] **Step 1:** Import at top of robots.js:

```js
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
```

- [ ] **Step 2:** Add next to the existing helpers (read them first and match their exact mesh-creation pattern — name, castShadow/receiveShadow flags):

```js
// Rounded box — same signature as box(), with bevel radius
function rbox(w, h, d, r, material, name) {
  const m = new THREE.Mesh(new RoundedBoxGeometry(w, h, d, 3, Math.min(r, w / 2, h / 2, d / 2)), material);
  if (name) m.name = name;
  m.castShadow = m.receiveShadow = true;
  return m;
}

// Capsule limb segment, axis along Y
function capsule(radius, length, material, name) {
  const m = new THREE.Mesh(new THREE.CapsuleGeometry(radius, length, 6, 20), material);
  if (name) m.name = name;
  m.castShadow = m.receiveShadow = true;
  return m;
}

// Cylinder with a chamfered (truncated-cone) edge top and bottom
function chamferCyl(r, h, chamfer, segments, material, name) {
  const g = new THREE.Group();
  if (name) g.name = name;
  const body = cyl(r, r, h - 2 * chamfer, segments, material);
  g.add(body);
  const top = cyl(r - chamfer, r, chamfer, segments, material);
  top.position.y = h / 2 - chamfer / 2;
  g.add(top);
  const bot = cyl(r, r - chamfer, chamfer, segments, material);
  bot.position.y = -h / 2 + chamfer / 2;
  g.add(bot);
  return g;
}

// Ring of bolt heads on a joint face (lies in XZ plane, +Y up)
function boltCircle(radius, n, material) {
  const g = new THREE.Group();
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2;
    const bolt = cyl(1.6, 1.6, 2, 6, material);
    bolt.position.set(Math.cos(a) * radius, 0, Math.sin(a) * radius);
    g.add(bolt);
  }
  return g;
}

// Strip of recessed vent slots across width w (slots along X)
function vents(w, n, material) {
  const g = new THREE.Group();
  const slotW = (w * 0.7) / n;
  for (let i = 0; i < n; i++) {
    const slot = box(slotW * 0.55, 1.2, 6, material);
    slot.position.x = -w * 0.35 + (i + 0.5) * slotW;
    g.add(slot);
  }
  return g;
}
```

- [ ] **Step 3: Verify** — module loads, 39 tests green (helpers unused yet).

- [ ] **Step 4: Commit** — `git commit -am "feat: add rounded/capsule/chamfer/bolt/vent geometry helpers"`

---

### Tasks 5–11: Per-robot polish (one task + commit per robot)

Order: **5 arm, 6 humanoid, 7 quadruped, 8 rover, 9 SCARA, 10 drone, 11 bimanual.**

**Iron rules for every robot task:**
1. Do NOT touch: group hierarchy/transform structure, joint index usage, anything read by
   kinematics configs, exported mesh names that tests or docs reference (`grep` the name
   in js/ and docs/ before renaming anything).
2. New decorative meshes get either no name (skipped by export list) or a descriptive
   name if they're a meaningful printable part.
3. After editing: 39 tests green, screenshot the robot from iso + front, iterate up to 3
   times on proportions/colors, then commit.
4. Keep segment counts modest (RoundedBox segments 3, cylinders ≤ 24) — drone/rover
   rebuild per frame.

**Specific work per robot (replace primitive shells with detailed ones):**

- **Task 5 — Arm:** base gets a chamferCyl pedestal + boltCircle; turret/shoulder/elbow
  joints become chamfered cylinders with accent ring (thin cyan or UR-blue `panel`
  torus); upper-arm/forearm links become `rbox` housings (radius ~6) with a thin
  raised spine; add a cable conduit: `TubeGeometry` along a `CatmullRomCurve3` from
  base side to wrist (decorative, parent = root, recompute points from params); gripper
  fingers become rbox with rubber pads. Commit: `feat: polish arm visuals to cobot-grade detail`.
- **Task 6 — Humanoid:** upper arm/forearm/thigh/shin boxes → `capsule` segments
  (keep same lengths/centres); torso → rbox shell + chest `vents`; head → rbox(r=8)
  with clearcoat visor; shoulder/hip spheres stay chrome. Commit:
  `feat: polish humanoid visuals with capsule limbs and shell torso`.
- **Task 7 — Quadruped:** body box → rbox shell (r=10) + side fairing rboxes; femur/
  tibia boxes → capsules; foot spheres → rubber; sensor head rbox + lens cylinder
  (clearcoat); lidar chamferCyl + vents. Commit: `feat: polish quadruped visuals to Spot-grade shell`.
- **Task 8 — Rover:** chassis/deck → rbox; add RTG (finned block: stack of thin boxes)
  at rear, 2 antenna masts (thin cyl + sphere tip), mast camera lens; wheels get cleat
  ridges (small boxes around tyre circumference, inside spinGroup). Commit:
  `feat: polish rover visuals with RTG, antennas, wheel cleats`.
- **Task 9 — SCARA:** pedestal chamferCyl + boltCircle; arm links → rbox casings with
  seam line (thin dark box inset); status LED strip (small emissive green box); cable
  loop tube from pedestal top to elbow joint. Commit: `feat: polish SCARA visuals with cast-arm casings`.
- **Task 10 — Drone:** body → rbox shell + camera gimbal (chrome sphere + lens);
  motor mounts → chamferCyl bells with fin rings; props → flat extruded aerofoil shape
  (`ExtrudeGeometry` from a 2-point airfoil `Shape`, 2 blades), keep prop spin transform;
  landing struts (thin cylinders). Commit: `feat: polish drone visuals with motor bells and aerofoil props`.
- **Task 11 — Bimanual:** torso → rbox column + vents; upper/forearms → capsules; palm →
  rbox; finger segments → small capsules (keep finger joint groups); head sensor strip.
  Commit: `feat: polish bimanual arm visuals with capsule limbs`.

---

### Task 12: Final regression + ship

- [ ] 39 tests green; golden path all 7 robots (joints, IK, export list, sequencer);
  both themes; wireframe toggle; gizmo/triads unaffected; drone/rover drive smooth.
- [ ] Screenshot every robot (iso view) — final visual sign-off gallery for the user.
- [ ] Commit any final tuning, push to origin.
