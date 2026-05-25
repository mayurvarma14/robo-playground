import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

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

    this.ambientLight = null;
    this.hemiLight    = null;
    this.sunLight     = null;
    this.fillLight    = null;
    this.rimLight     = null;
    this.backLight    = null;

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

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.06;
    this.controls.minDistance   = 60;
    this.controls.maxDistance   = 4000;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.01;
    this.controls.target.set(0, 90, 0);
    this.controls.update();

    this._buildLighting();
    this._buildFloor();
    this._applySceneTheme('dark');

    window.addEventListener('resize', () => this._onResize());
    this.renderer.setAnimationLoop((t) => this._tick(t));
  }

  _buildLighting() {
    // Strong ambient so nothing is ever pitch-black
    this.ambientLight = new THREE.AmbientLight(0xffffff, 1.2);
    this.scene.add(this.ambientLight);

    // Hemisphere: warm sky, cooler ground bounce
    this.hemiLight = new THREE.HemisphereLight(0xc8ddf0, 0x9aabb8, 1.0);
    this.hemiLight.position.set(0, 500, 0);
    this.scene.add(this.hemiLight);

    // Key sun — main shadow caster
    this.sunLight = new THREE.DirectionalLight(0xfff8f0, 1.8);
    this.sunLight.position.set(350, 600, 300);
    this.sunLight.castShadow = true;
    this.sunLight.shadow.mapSize.set(2048, 2048);
    this.sunLight.shadow.camera.near   = 10;
    this.sunLight.shadow.camera.far    = 2000;
    this.sunLight.shadow.camera.left   = -500;
    this.sunLight.shadow.camera.right  = 500;
    this.sunLight.shadow.camera.top    = 500;
    this.sunLight.shadow.camera.bottom = -500;
    this.sunLight.shadow.bias = -0.0003;
    this.scene.add(this.sunLight);

    // Cool fill from opposite side
    this.fillLight = new THREE.DirectionalLight(0xb0d0ff, 1.0);
    this.fillLight.position.set(-300, 400, -250);
    this.scene.add(this.fillLight);

    // Warm back rim
    this.rimLight = new THREE.DirectionalLight(0xffd6a0, 0.6);
    this.rimLight.position.set(0, 200, -500);
    this.scene.add(this.rimLight);

    // Low under-fill (lights underside)
    this.backLight = new THREE.DirectionalLight(0xffffff, 0.5);
    this.backLight.position.set(0, -200, 0);
    this.scene.add(this.backLight);
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
      if (this.ambientLight) this.ambientLight.intensity = 1.8;
      if (this.hemiLight)    this.hemiLight.intensity    = 1.4;
      if (this.sunLight)     this.sunLight.intensity     = 2.2;
      if (this.fillLight)    this.fillLight.intensity    = 1.2;
      if (this.rimLight)     this.rimLight.intensity     = 0.6;
      if (this.backLight)    this.backLight.intensity    = 0.7;
    } else {
      // Slate-blue dark — comfortable engineering software tone, NOT a black void
      this.scene.background = new THREE.Color(0x1c2a3a);
      this.scene.fog = new THREE.Fog(0x1c2a3a, 2500, 8000);
      if (this.platformMesh) this.platformMesh.material.color.setHex(0x3a4a60);
      this._rebuildGrid(0x4a6080, 0x2a3a50);
      if (this.ambientLight) this.ambientLight.intensity = 1.2;
      if (this.hemiLight)    this.hemiLight.intensity    = 1.0;
      if (this.sunLight)     this.sunLight.intensity     = 1.8;
      if (this.fillLight)    this.fillLight.intensity    = 1.0;
      if (this.rimLight)     this.rimLight.intensity     = 0.6;
      if (this.backLight)    this.backLight.intensity    = 0.5;
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

  setWireframe(v) {
    this.wireframe = v;
    this.scene.traverse(obj => {
      if (obj.isMesh && obj.material) obj.material.wireframe = v;
    });
  }

  setBrightness(multiplier) {
    const isLight = this.theme === 'light';
    if (this.ambientLight) this.ambientLight.intensity = multiplier * (isLight ? 1.8 : 1.2);
    if (this.sunLight)     this.sunLight.intensity     = multiplier * (isLight ? 2.2 : 1.8);
    if (this.fillLight)    this.fillLight.intensity    = multiplier * (isLight ? 1.2 : 1.0);
    if (this.hemiLight)    this.hemiLight.intensity    = multiplier * (isLight ? 1.4 : 1.0);
  }
}
