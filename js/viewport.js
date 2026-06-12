import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

export class Viewport {
  constructor() {
    this.scene      = null;
    this.camera     = null;
    this.renderer   = null;
    this.controls   = null;

    this.gridHelper   = null;
    this.axesHelper   = null;
    this.platformMesh = null;
    this.floorMesh    = null;

    this.hemiLight    = null;
    this.sunLight     = null;
    this.rimLight     = null;

    this.showGrid  = true;
    this.showAxes  = true;
    this.wireframe = false;
    this.callbacks = [];
    this.theme     = 'dark';
  }

  init(containerId) {
    this.container = document.getElementById(containerId);
    if (!this.container) { console.error('Canvas container not found'); return; }

    this.scene = new THREE.Scene();

    const { clientWidth: w, clientHeight: h } = this.container;
    this.camera = new THREE.PerspectiveCamera(45, w / h, 1, 10000);
    this.camera.position.set(380, 320, 420);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setSize(w, h);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type    = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace  = THREE.SRGBColorSpace;
    this.renderer.toneMapping       = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.1;
    this.container.appendChild(this.renderer.domElement);

    // Studio environment — gives PBR metals real reflections (no asset files)
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.envTexture = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    pmrem.dispose();
    this.scene.environment = this.envTexture;

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.06;
    this.controls.minDistance   = 60;
    this.controls.maxDistance   = 4000;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.01;
    this.controls.target.set(0, 90, 0);
    this.controls.update();

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

    this._buildLighting();
    this._buildFloor();
    this._applySceneTheme('dark');

    window.addEventListener('resize', () => this._onResize());
    this.renderer.setAnimationLoop((t) => this._tick(t));
  }

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

  _buildFloor() {
    // Soft shadow catcher
    this.floorMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(6000, 6000),
      new THREE.ShadowMaterial({ opacity: 0.12 })
    );
    this.floorMesh.rotation.x = -Math.PI / 2;
    this.floorMesh.receiveShadow = true;
    this.scene.add(this.floorMesh);

    // Platform disc
    this.platformMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(220, 230, 8, 64),
      new THREE.MeshStandardMaterial({ color: 0x3a4a60, roughness: 0.7, metalness: 0.15 })
    );
    this.platformMesh.position.y = -4;
    this.platformMesh.receiveShadow = true;
    this.scene.add(this.platformMesh);

    // Grid
    this.gridHelper = new THREE.GridHelper(1200, 30, 0x4a6080, 0x2a3a50);
    this.gridHelper.position.y = 0.5;
    this.scene.add(this.gridHelper);

    // Axes helper
    this.axesHelper = new THREE.AxesHelper(160);
    this.axesHelper.position.set(-460, 1, -460);
    this.scene.add(this.axesHelper);
  }

  _applySceneTheme(theme) {
    this.theme = theme;

    if (theme === 'light') {
      // Bright, airy laboratory feel
      this.scene.background = new THREE.Color(0xdce6f0);
      this.scene.fog = new THREE.Fog(0xdce6f0, 2000, 7000);
      if (this.platformMesh) this.platformMesh.material.color.setHex(0xa8bdd0);
      this._rebuildGrid(0x7090b0, 0xb8ccd8);
      this.renderer.toneMappingExposure = 1.15;
      if (this.hemiLight) this.hemiLight.intensity = 0.5;
      if (this.sunLight)  this.sunLight.intensity  = 2.8;
      if (this.rimLight)  this.rimLight.intensity  = 0.7;
    } else {
      // Slate-blue dark — comfortable engineering software tone, NOT a black void
      this.scene.background = new THREE.Color(0x1c2a3a);
      this.scene.fog = new THREE.Fog(0x1c2a3a, 2500, 8000);
      if (this.platformMesh) this.platformMesh.material.color.setHex(0x3a4a60);
      this._rebuildGrid(0x4a6080, 0x2a3a50);
      this.renderer.toneMappingExposure = 1.0;
      if (this.hemiLight) this.hemiLight.intensity = 0.35;
      if (this.sunLight)  this.sunLight.intensity  = 2.4;
      if (this.rimLight)  this.rimLight.intensity  = 0.9;
    }
  }

  _rebuildGrid(c1, c2) {
    if (this.gridHelper) this.scene.remove(this.gridHelper);
    this.gridHelper = new THREE.GridHelper(1200, 30, c1, c2);
    this.gridHelper.position.y = 0.5;
    this.gridHelper.visible = this.showGrid;
    this.scene.add(this.gridHelper);
  }

  _onResize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  _tick(time) {
    this.controls.update();
    for (const cb of this.callbacks) cb(time);
    this.renderer.render(this.scene, this.camera);
  }

  registerCallback(fn)   { this.callbacks.push(fn); }
  unregisterCallback(fn) { this.callbacks = this.callbacks.filter(c => c !== fn); }

  addObject(obj)    { this.scene.add(obj); }
  removeObject(obj) { if (obj) this.scene.remove(obj); }

  setTheme(theme)   { this._applySceneTheme(theme); }

  setView(name) {
    const views = {
      iso:   { pos: [380, 320, 420], tgt: [0, 90, 0] },
      front: { pos: [0, 180, 650],   tgt: [0, 120, 0] },
      side:  { pos: [650, 180, 0],   tgt: [0, 120, 0] },
      top:   { pos: [0, 800, 0.1],   tgt: [0, 0, 0] },
    };
    const v = views[name] || views.iso;
    this.camera.position.set(...v.pos);
    this.controls.target.set(...v.tgt);
    this.controls.update();
  }

  setGridVisible(v) {
    this.showGrid = v;
    if (this.gridHelper) this.gridHelper.visible = v;
  }

  setAxesVisible(v) {
    this.showAxes = v;
    if (this.axesHelper) this.axesHelper.visible = v;
  }

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

  _isHelper(obj) {
    let o = obj;
    while (o) {
      if (o === this.gizmo || o === this.kinHelpers || o === this.traceLine) return true;
      o = o.parent;
    }
    return false;
  }

  setWireframe(v) {
    this.wireframe = v;
    this.scene.traverse(obj => {
      if (obj.isMesh && obj.material && !this._isHelper(obj)) obj.material.wireframe = v;
    });
  }

  setBrightness(multiplier) {
    const isLight = this.theme === 'light';
    if (this.sunLight)  this.sunLight.intensity  = multiplier * (isLight ? 2.8 : 2.4);
    if (this.hemiLight) this.hemiLight.intensity = multiplier * (isLight ? 0.5 : 0.35);
    if (this.rimLight)  this.rimLight.intensity  = multiplier * (isLight ? 0.7 : 0.9);
  }
}
