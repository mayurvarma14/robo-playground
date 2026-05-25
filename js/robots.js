/**
 * robots.js
 * Builds detailed procedural 3D robot meshes using THREE.js geometry.
 * Every robot is a THREE.Group with named sub-meshes for STL export.
 */
import * as THREE from 'three';
import { MAT } from './materials.js';

// ─────────────────────────────────────────────────────────────
// SHARED GEOMETRY HELPERS
// ─────────────────────────────────────────────────────────────

function mesh(geo, mat, name = '') {
  const m = new THREE.Mesh(geo, mat);
  m.castShadow = true;
  m.receiveShadow = true;
  if (name) m.name = name;
  return m;
}

function box(w, h, d, mat, name) {
  return mesh(new THREE.BoxGeometry(w, h, d, 1, 1, 1), mat, name);
}

function cyl(rTop, rBot, h, segs = 24, mat, name) {
  return mesh(new THREE.CylinderGeometry(rTop, rBot, h, segs), mat, name);
}

function sphere(r, segs = 18, mat, name) {
  return mesh(new THREE.SphereGeometry(r, segs, segs), mat, name);
}

function ring(inner, outer, h, segs = 32, mat, name) {
  // Hollow cylinder (torus-like, but a proper ring)
  const shape = new THREE.Shape();
  shape.absarc(0, 0, outer, 0, Math.PI * 2, false);
  const hole = new THREE.Path();
  hole.absarc(0, 0, inner, 0, Math.PI * 2, true);
  shape.holes.push(hole);
  const geo = new THREE.ExtrudeGeometry(shape, { depth: h, bevelEnabled: false });
  geo.center();
  const m = mesh(geo, mat, name);
  m.rotation.x = Math.PI / 2;
  return m;
}

// Generic servo motor visual block
function servoBlock(w, h, d, mat) {
  const g = new THREE.Group();
  const body = box(w, h, d, mat, 'Servo Body');
  g.add(body);
  // output shaft
  const shaft = cyl(2.5, 2.5, 8, 12, MAT.chrome);
  shaft.position.set(0, h / 2 + 4, 0);
  g.add(shaft);
  // mounting flanges
  const fl = box(w + 10, 4, 6, MAT.darkSteel);
  fl.position.set(0, -h / 2 + 2, 0);
  g.add(fl);
  return g;
}

// Ball bearing ring detail
function bearing(outerR, width) {
  return ring(outerR - 5, outerR, width, 32, MAT.chrome);
}

// Rounded-end link (structural arm segment)
function roundedLink(length, width, depth, mat, name) {
  const shape = new THREE.Shape();
  const hw = width / 2;
  const hd = depth / 2;
  shape.absarc(0, 0, hw, Math.PI / 2, (3 * Math.PI) / 2, false);
  shape.lineTo(length, -hw);
  shape.absarc(length, 0, hw, -Math.PI / 2, Math.PI / 2, false);
  shape.lineTo(0, hw);
  const geo = new THREE.ExtrudeGeometry(shape, { depth: hd * 2, bevelEnabled: true, bevelSize: 1, bevelThickness: 0.5, bevelSegments: 2 });
  geo.center();
  const m = mesh(geo, mat, name);
  m.rotation.y = Math.PI / 2;
  return m;
}

// Lightweight truss bar with triangular cutouts (extruded 2D profile)
function trussBar(length, height, depth, mat, name) {
  const shape = new THREE.Shape();
  const hh = height / 2;
  shape.moveTo(0, -hh);
  shape.lineTo(length, -hh);
  shape.lineTo(length, hh);
  shape.lineTo(0, hh);
  shape.closePath();
  // Add triangular weight-reduction cutouts
  const numCuts = Math.floor(length / 30);
  for (let i = 0; i < numCuts; i++) {
    const x0 = 12 + i * ((length - 24) / numCuts);
    const x1 = x0 + (length - 24) / numCuts - 8;
    const midX = (x0 + x1) / 2;
    const path = new THREE.Path();
    if (i % 2 === 0) {
      path.moveTo(x0, -hh + 5); path.lineTo(x1, -hh + 5); path.lineTo(midX, hh - 5);
    } else {
      path.moveTo(x0, hh - 5); path.lineTo(x1, hh - 5); path.lineTo(midX, -hh + 5);
    }
    path.closePath();
    shape.holes.push(path);
  }
  const geo = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false });
  geo.center();
  const m = mesh(geo, mat, name);
  return m;
}

// ─────────────────────────────────────────────────────────────
// ROBOT 1 — 6-DOF INDUSTRIAL ARM
// ─────────────────────────────────────────────────────────────
export function buildArm(joints, params) {
  const { l1 = 80, l2 = 130, l3 = 110, l4 = 80 } = params;
  const root = new THREE.Group();

  // ── BASE
  const baseGroup = new THREE.Group();
  root.add(baseGroup);

  // base plate disc
  const baseDisk = cyl(90, 100, 18, 48, MAT.darkSteel, 'Base Mount Plate');
  baseDisk.position.y = 9;
  baseGroup.add(baseDisk);

  // base top ring bearing
  const b0 = bearing(60, 10);
  b0.position.y = 20;
  baseGroup.add(b0);

  // rotating turret
  const turret = new THREE.Group();
  turret.rotation.y = joints[0];
  baseGroup.add(turret);

  const turretBody = cyl(42, 48, 50, 32, MAT.blackAnodised, 'Base Turret');
  turretBody.position.y = 45;
  turret.add(turretBody);

  // shoulder bearing
  const b1 = bearing(36, 12);
  b1.position.y = l1 + 20;
  turret.add(b1);

  // ── UPPER ARM
  const upperGroup = new THREE.Group();
  upperGroup.position.y = l1 + 20;
  upperGroup.rotation.z = joints[1];
  turret.add(upperGroup);

  const upperLink = trussBar(l2, 32, 24, MAT.carbonFiber, 'Upper Arm Truss');
  upperLink.rotation.z = Math.PI / 2;
  upperLink.position.y = l2 / 2;
  upperGroup.add(upperLink);

  // shoulder actuator
  const shoulderActuator = cyl(18, 18, 30, 20, MAT.darkSteel, 'Shoulder Actuator');
  shoulderActuator.rotation.z = Math.PI / 2;
  shoulderActuator.position.y = 2;
  upperGroup.add(shoulderActuator);

  // elbow bearing
  const b2 = bearing(28, 10);
  b2.position.y = l2;
  upperGroup.add(b2);

  // ── FOREARM
  const forearmGroup = new THREE.Group();
  forearmGroup.position.y = l2;
  forearmGroup.rotation.z = joints[2];
  upperGroup.add(forearmGroup);

  const foreLink = trussBar(l3, 26, 20, MAT.carbonFiber, 'Forearm Truss');
  foreLink.rotation.z = Math.PI / 2;
  foreLink.position.y = l3 / 2;
  forearmGroup.add(foreLink);

  const elbowActuator = cyl(15, 15, 26, 20, MAT.darkSteel, 'Elbow Actuator');
  elbowActuator.rotation.z = Math.PI / 2;
  elbowActuator.position.y = 2;
  forearmGroup.add(elbowActuator);

  // wrist bearing
  const b3 = bearing(22, 8);
  b3.position.y = l3;
  forearmGroup.add(b3);

  // ── WRIST
  const wristGroup = new THREE.Group();
  wristGroup.position.y = l3;
  wristGroup.rotation.z = joints[3];
  forearmGroup.add(wristGroup);

  const wristLink = roundedLink(l4, 20, 16, MAT.aluminium, 'Wrist Link');
  wristLink.position.y = l4 / 2;
  wristGroup.add(wristLink);

  const wristActuator = cyl(12, 12, 20, 20, MAT.chrome, 'Wrist Actuator');
  wristActuator.rotation.z = Math.PI / 2;
  wristActuator.position.y = 2;
  wristGroup.add(wristActuator);

  // ── END EFFECTOR / GRIPPER
  const gripGroup = new THREE.Group();
  gripGroup.position.y = l4;
  wristGroup.add(gripGroup);

  const gripBase = cyl(14, 14, 18, 20, MAT.blackAnodised, 'Gripper Base');
  gripBase.position.y = 9;
  gripGroup.add(gripBase);

  const clawGap = joints[4] !== undefined ? joints[4] : 18;
  for (let side of [-1, 1]) {
    const claw = new THREE.Group();
    claw.position.set(side * clawGap / 2, 16, 0);

    const finger = box(6, 30, 8, MAT.darkSteel, side > 0 ? 'Gripper Finger Right' : 'Gripper Finger Left');
    finger.position.y = 15;
    claw.add(finger);

    const tip = box(8, 10, 10, MAT.rubber, 'Gripper Tip');
    tip.position.y = 34;
    claw.add(tip);
    gripGroup.add(claw);
  }

  return root;
}

// FK helper for arm
export function armFK(joints, params) {
  const { l1 = 80, l2 = 130, l3 = 110, l4 = 80 } = params;
  const j = joints;
  const cos = Math.cos, sin = Math.sin;
  const sa = j[1], ea = j[1] + j[2], wa = j[1] + j[2] + j[3];
  const x = (l2 * cos(sa) + l3 * cos(ea) + l4 * cos(wa)) * cos(j[0]);
  const y = l1 + l2 * sin(sa) + l3 * sin(ea) + l4 * sin(wa) + 20;
  const z = (l2 * cos(sa) + l3 * cos(ea) + l4 * cos(wa)) * sin(j[0]);
  return { x: Math.round(x), y: Math.round(y), z: Math.round(z) };
}

// ─────────────────────────────────────────────────────────────
// ROBOT 2 — HUMANOID (OPTIMUS INSPIRED)
// ─────────────────────────────────────────────────────────────
export function buildHumanoid(joints, params) {
  const { hipSpacing = 90, thigh = 100, shin = 110, footH = 18 } = params;
  const root = new THREE.Group();

  // ── TORSO
  const torso = new THREE.Group();
  torso.rotation.y = joints[0]; // torso twist
  root.add(torso);

  // Pelvis
  const pelvis = box(hipSpacing + 30, 30, 40, MAT.darkSteel, 'Pelvis');
  pelvis.position.y = 0;
  torso.add(pelvis);

  // Abdomen
  const abdomen = box(60, 50, 35, MAT.whitePolycarbonate, 'Abdomen Panel');
  abdomen.position.y = 45;
  torso.add(abdomen);

  // Chest plate
  const chest = box(80, 70, 38, MAT.whitePolycarbonate, 'Chest Panel');
  chest.position.y = 110;
  torso.add(chest);

  // Chest internal frame
  const chestFrame = box(72, 62, 30, MAT.darkSteel, 'Chest Frame');
  chestFrame.position.y = 110;
  torso.add(chestFrame);

  // Neck
  const neck = cyl(12, 14, 25, 16, MAT.chrome, 'Neck');
  neck.position.y = 160;
  torso.add(neck);

  // Head
  const headGroup = new THREE.Group();
  headGroup.position.y = 190;
  torso.add(headGroup);

  const headBody = box(48, 50, 40, MAT.whitePolycarbonate, 'Head Shell');
  headGroup.add(headBody);

  // Visor
  const visorGeo = new THREE.BoxGeometry(40, 12, 5);
  const visor = mesh(visorGeo, MAT.cyan, 'Vision Visor');
  visor.position.set(0, 5, 22);
  headGroup.add(visor);

  // Shoulders
  for (let side of [-1, 1]) {
    const shoulderGroup = new THREE.Group();
    shoulderGroup.position.set(side * (hipSpacing / 2 + 30), 120, 0);
    torso.add(shoulderGroup);

    const shoulderJoint = sphere(16, 16, MAT.chrome, 'Shoulder Joint');
    shoulderGroup.add(shoulderJoint);

    // Upper arm
    const armG = new THREE.Group();
    armG.rotation.z = side > 0 ? joints[4] : joints[1];
    shoulderGroup.add(armG);

    const upperArm = box(22, 80, 22, MAT.blackAnodised, side > 0 ? 'R Upper Arm' : 'L Upper Arm');
    upperArm.position.y = -40;
    armG.add(upperArm);

    // Elbow
    const elbowJoint = cyl(12, 12, 24, 16, MAT.chrome, 'Elbow Joint');
    elbowJoint.rotation.z = Math.PI / 2;
    elbowJoint.position.y = -82;
    armG.add(elbowJoint);

    // Forearm
    const forearmG = new THREE.Group();
    forearmG.position.y = -82;
    forearmG.rotation.z = side > 0 ? joints[6] : joints[3];
    armG.add(forearmG);

    const forearm = box(18, 75, 18, MAT.whitePolycarbonate, side > 0 ? 'R Forearm' : 'L Forearm');
    forearm.position.y = -38;
    forearmG.add(forearm);

    // Hand
    const hand = box(22, 20, 16, MAT.darkSteel, side > 0 ? 'R Hand' : 'L Hand');
    hand.position.y = -82;
    forearmG.add(hand);

    // Fingers (simplified 3 fingers)
    for (let fi = -1; fi <= 1; fi++) {
      const finger = box(5, 20, 5, MAT.darkSteel);
      finger.position.set(fi * 6, -96, 0);
      forearmG.add(finger);
    }
  }

  // ── LEGS
  for (let side of [-1, 1]) {
    const hipJoint = new THREE.Group();
    hipJoint.position.set(side * hipSpacing / 2, -15, 0);
    torso.add(hipJoint);

    const hipBall = sphere(18, 16, MAT.chrome, 'Hip Joint');
    hipJoint.add(hipBall);

    // Thigh
    const thighG = new THREE.Group();
    thighG.rotation.x = side > 0 ? joints[10] : joints[7];
    hipJoint.add(thighG);

    const thighMesh = box(28, thigh, 28, MAT.whitePolycarbonate, side > 0 ? 'R Thigh' : 'L Thigh');
    thighMesh.position.y = -thigh / 2;
    thighG.add(thighMesh);

    // Thigh actuator cover
    const thighActuator = cyl(16, 16, thigh - 10, 20, MAT.darkSteel, side > 0 ? 'R Thigh Actuator' : 'L Thigh Actuator');
    thighActuator.position.y = -thigh / 2;
    thighG.add(thighActuator);

    // Knee
    const kneeG = new THREE.Group();
    kneeG.position.y = -thigh;
    kneeG.rotation.x = side > 0 ? joints[11] : joints[8];
    thighG.add(kneeG);

    const kneeBall = sphere(16, 16, MAT.chrome, 'Knee Joint');
    kneeG.add(kneeBall);

    // Shin
    const shinMesh = box(24, shin, 24, MAT.whitePolycarbonate, side > 0 ? 'R Shin' : 'L Shin');
    shinMesh.position.y = -shin / 2;
    kneeG.add(shinMesh);

    // Ankle
    const ankleG = new THREE.Group();
    ankleG.position.y = -shin;
    ankleG.rotation.x = side > 0 ? joints[12] : joints[9];
    kneeG.add(ankleG);

    const ankleBall = sphere(13, 12, MAT.chrome, 'Ankle Joint');
    ankleG.add(ankleBall);

    // Foot
    const foot = box(40, footH, 90, MAT.darkSteel, side > 0 ? 'R Foot' : 'L Foot');
    foot.position.set(0, -footH / 2, 20);
    ankleG.add(foot);
  }

  // Position robot so feet rest on ground
  root.position.y = thigh + shin + footH + 15;
  return root;
}

// ─────────────────────────────────────────────────────────────
// ROBOT 3 — QUADRUPED (SPOT-INSPIRED)
// ─────────────────────────────────────────────────────────────
export function buildQuadruped(joints, params) {
  const { coxa = 40, femur = 80, tibia = 85, bodyLen = 160, bodyW = 100 } = params;
  const root = new THREE.Group();

  // Body
  const body = box(bodyW, 30, bodyLen, MAT.blackAnodised, 'Main Body');
  body.position.y = femur + tibia + 10;
  root.add(body);

  // Body top cover
  const cover = box(bodyW - 10, 20, bodyLen - 10, MAT.darkPolycarbonate, 'Body Cover');
  cover.position.y = femur + tibia + 28;
  root.add(cover);

  // Sensor head
  const sensorHead = box(bodyW - 20, 24, 30, MAT.titanium, 'Sensor Head');
  sensorHead.position.set(0, femur + tibia + 30, bodyLen / 2 - 5);
  root.add(sensorHead);

  // LiDAR disk
  const lidar = cyl(12, 12, 8, 20, MAT.cyan, 'LiDAR');
  lidar.position.set(0, femur + tibia + 46, bodyLen / 2 - 5);
  root.add(lidar);

  const legPositions = [
    [  bodyW / 2,  bodyLen / 2 - 20 ],  // FR
    [ -bodyW / 2,  bodyLen / 2 - 20 ],  // FL
    [ -bodyW / 2, -bodyLen / 2 + 20 ],  // BL
    [  bodyW / 2, -bodyLen / 2 + 20 ],  // BR
  ];

  const legNames = ['FR', 'FL', 'BL', 'BR'];

  for (let i = 0; i < 4; i++) {
    const [lx, lz] = legPositions[i];
    const prefix = legNames[i];
    const ji = i * 3; // joint index offset

    const legGroup = new THREE.Group();
    legGroup.position.set(lx, femur + tibia + 10, lz);
    root.add(legGroup);

    // Hip yaw joint
    const hipYaw = new THREE.Group();
    hipYaw.rotation.y = joints[ji];
    legGroup.add(hipYaw);

    const hipDisk = cyl(14, 14, 20, 16, MAT.chrome, `${prefix} Hip`);
    hipDisk.rotation.z = Math.PI / 2;
    hipYaw.add(hipDisk);

    // Coxa (hip abductor)
    const coxaLink = roundedLink(coxa, 18, 14, MAT.darkSteel, `${prefix} Coxa`);
    coxaLink.position.set(lx > 0 ? coxa / 2 : -coxa / 2, 0, 0);
    hipYaw.add(coxaLink);

    const coxaEnd = new THREE.Group();
    coxaEnd.position.set(lx > 0 ? coxa : -coxa, 0, 0);
    hipYaw.add(coxaEnd);

    // Hip pitch joint
    const hipPitch = new THREE.Group();
    hipPitch.rotation.x = joints[ji + 1];
    coxaEnd.add(hipPitch);

    const femurLink = box(18, femur, 16, MAT.whitePolycarbonate, `${prefix} Femur`);
    femurLink.position.y = -femur / 2;
    hipPitch.add(femurLink);

    // Knee joint
    const kneeGroup = new THREE.Group();
    kneeGroup.position.y = -femur;
    kneeGroup.rotation.x = joints[ji + 2];
    hipPitch.add(kneeGroup);

    const kneeDisk = cyl(12, 12, 18, 16, MAT.chrome, `${prefix} Knee`);
    kneeDisk.rotation.z = Math.PI / 2;
    kneeGroup.add(kneeDisk);

    const tibiaLink = box(14, tibia, 12, MAT.carbonFiber, `${prefix} Tibia`);
    tibiaLink.position.y = -tibia / 2;
    kneeGroup.add(tibiaLink);

    // Foot
    const foot = sphere(9, 12, MAT.rubber, `${prefix} Foot`);
    foot.position.y = -tibia - 2;
    kneeGroup.add(foot);
  }

  return root;
}

// ─────────────────────────────────────────────────────────────
// ROBOT 4 — MARS ROVER (ROCKER-BOGIE)
// ─────────────────────────────────────────────────────────────
export function buildRover(joints, params) {
  const { chassisL = 200, chassisW = 140, wheelR = 40, wheelW = 20 } = params;
  const root = new THREE.Group();
  const bodyY = wheelR + 30;

  // Main chassis frame
  const chassis = box(chassisW, 20, chassisL, MAT.darkSteel, 'Chassis Frame');
  chassis.position.y = bodyY;
  root.add(chassis);

  // Science deck / body
  const deck = box(chassisW - 10, 14, chassisL - 30, MAT.titanium, 'Science Deck');
  deck.position.y = bodyY + 18;
  root.add(deck);

  // Solar panels
  const solarL = box(chassisW + 50, 4, chassisL - 60, MAT.cyan, 'Solar Panel L');
  solarL.position.set(-(chassisW + 50) / 2, bodyY + 30, 0);
  root.add(solarL);

  const solarR = box(chassisW + 50, 4, chassisL - 60, MAT.cyan, 'Solar Panel R');
  solarR.position.set((chassisW + 50) / 2, bodyY + 30, 0);
  root.add(solarR);

  // Camera mast
  const mast = cyl(5, 5, 80, 8, MAT.aluminium, 'Camera Mast');
  mast.position.set(0, bodyY + 40 + 40, -chassisL / 2 + 20);
  root.add(mast);

  const camHead = box(30, 20, 20, MAT.blackAnodised, 'Camera Head');
  camHead.position.set(0, bodyY + 90, -chassisL / 2 + 20);
  root.add(camHead);

  // Robotic arm (front right)
  const armBase = box(16, 60, 16, MAT.aluminium, 'Sample Arm');
  armBase.position.set(chassisW / 2 - 10, bodyY + 20, -chassisL / 2 + 30);
  armBase.rotation.z = -Math.PI / 6;
  root.add(armBase);

  // 6 wheels — rocker-bogie style
  const wheelPositions = [
    [-1, -1, -chassisL / 2 + 25],  // FL
    [ 1, -1, -chassisL / 2 + 25],  // FR
    [-1, -1,  0],                   // ML
    [ 1, -1,  0],                   // MR
    [-1, -1,  chassisL / 2 - 25],  // BL
    [ 1, -1,  chassisL / 2 - 25],  // BR
  ];

  const wheelNames = ['Front Left', 'Front Right', 'Mid Left', 'Mid Right', 'Rear Left', 'Rear Right'];

  for (let i = 0; i < 6; i++) {
    const [sx, , wz] = wheelPositions[i];
    const wg = new THREE.Group();
    wg.position.set(sx * (chassisW / 2 + wheelW / 2), wheelR, wz);

    // Tyre
    const tyre = cyl(wheelR, wheelR, wheelW, 24, MAT.rubber, `${wheelNames[i]} Tyre`);
    tyre.rotation.z = Math.PI / 2;
    tyre.rotation.y = joints[i < 4 ? (i < 2 ? 0 : 1) : 2];
    wg.add(tyre);

    // Hub
    const hub = cyl(wheelR * 0.45, wheelR * 0.45, wheelW + 4, 16, MAT.aluminium, `${wheelNames[i]} Hub`);
    hub.rotation.z = Math.PI / 2;
    wg.add(hub);

    // Spokes
    for (let s = 0; s < 5; s++) {
      const spoke = box(wheelR * 0.75, 3, 3, MAT.darkSteel);
      spoke.rotation.z = (s / 5) * Math.PI;
      wg.add(spoke);
    }

    root.add(wg);
  }

  return root;
}

// ─────────────────────────────────────────────────────────────
// ROBOT 5 — SCARA ARM
// ─────────────────────────────────────────────────────────────
export function buildSCARA(joints, params) {
  const { l1 = 150, l2 = 130, pedestalH = 190 } = params;
  const root = new THREE.Group();

  // Pedestal
  const pedestal = cyl(30, 36, pedestalH, 24, MAT.darkSteel, 'Pedestal Column');
  pedestal.position.y = pedestalH / 2;
  root.add(pedestal);

  // Pedestal cap
  const cap = cyl(42, 42, 20, 24, MAT.blackAnodised, 'Pedestal Cap');
  cap.position.y = pedestalH + 10;
  root.add(cap);

  // ── INNER ARM
  const inner = new THREE.Group();
  inner.position.y = pedestalH + 20;
  inner.rotation.y = joints[0];
  root.add(inner);

  const innerLink = roundedLink(l1, 40, 28, MAT.aluminium, 'Inner Arm Link');
  innerLink.rotation.z = Math.PI / 2;
  innerLink.position.x = l1 / 2;
  inner.add(innerLink);

  // Shoulder motor
  const shoulderMot = cyl(26, 26, 40, 24, MAT.blackAnodised, 'Shoulder Motor');
  inner.add(shoulderMot);

  // Elbow motor
  const elbowMot = cyl(20, 20, 36, 20, MAT.darkSteel, 'Elbow Motor');
  elbowMot.position.x = l1;
  inner.add(elbowMot);

  // ── OUTER ARM
  const outer = new THREE.Group();
  outer.position.x = l1;
  outer.rotation.y = joints[1];
  inner.add(outer);

  const outerLink = roundedLink(l2, 32, 24, MAT.aluminium, 'Outer Arm Link');
  outerLink.rotation.z = Math.PI / 2;
  outerLink.position.x = l2 / 2;
  outer.add(outerLink);

  // ── Z SLIDE ASSEMBLY
  const slideGroup = new THREE.Group();
  slideGroup.position.set(l2, 0, 0);
  outer.add(slideGroup);

  // Rod guides
  const guideBlock = box(36, 40, 28, MAT.blackAnodised, 'Linear Guide Block');
  slideGroup.add(guideBlock);

  // Z rods
  for (let s of [-1, 1]) {
    const rod = cyl(5, 5, 150, 12, MAT.chrome, s > 0 ? 'Slide Rod R' : 'Slide Rod L');
    rod.position.set(s * 12, 0, 0);
    slideGroup.add(rod);
  }

  // Spindle
  const spindle = cyl(4, 4, 150, 12, MAT.brass, 'Lead Screw');
  slideGroup.add(spindle);

  // End effector head
  const eeGroup = new THREE.Group();
  eeGroup.position.y = -(joints[2] || 30) - 20;
  slideGroup.add(eeGroup);

  const eeHead = box(30, 20, 22, MAT.darkSteel, 'End Effector Head');
  eeGroup.add(eeHead);

  // Vacuum nozzle
  const nozzle = cyl(4, 8, 28, 12, MAT.chrome, 'Vacuum Nozzle');
  nozzle.position.y = -20;
  eeGroup.add(nozzle);

  return root;
}

// ─────────────────────────────────────────────────────────────
// ROBOT 6 — DEXTEROUS BIMANUAL ARM (Torso + two 5-finger hands)
// ─────────────────────────────────────────────────────────────

// Helper: build one full dexterous arm attached to parent.
// side: -1=left, +1=right. Both arms share the same 20 joints (mirrored).
function buildSingleArm(parent, side, joints, upperArmLen, forearmLen, palmLen, prefix) {
  // ── SHOULDER BEARING (decorative, on the mount face)
  const shoulderBearing = ring(22, 36, 10, 32, MAT.chrome, `${prefix} Shoulder Bearing`);
  shoulderBearing.rotation.z = Math.PI / 2;
  parent.add(shoulderBearing);

  const shoulderBall = sphere(18, 20, MAT.chrome, `${prefix} Shoulder Ball`);
  parent.add(shoulderBall);

  // ── SHOULDER GROUP — pitch + yaw, mirrored per side
  const shoulderGroup = new THREE.Group();
  shoulderGroup.rotation.z = joints[0] * side; // raise/lower arm
  shoulderGroup.rotation.y = joints[1] * side; // swing forward/back
  parent.add(shoulderGroup);

  // ── T-POSE BASE: arm extends OUTWARD (\u00b1X) not downward
  // right arm side=+1 → rotation.z=+π/2 → local -Y becomes world +X
  // left  arm side=-1 → rotation.z=-π/2 → local -Y becomes world -X
  const tPose = new THREE.Group();
  tPose.rotation.z = side * Math.PI / 2;
  shoulderGroup.add(tPose);

  // ── UPPER ARM
  const upperArmGroup = new THREE.Group();
  tPose.add(upperArmGroup);

  const upperOuter = box(28, upperArmLen, 24, MAT.whitePolycarbonate, `${prefix} Upper Arm Shell`);
  upperOuter.position.y = -upperArmLen / 2;
  upperArmGroup.add(upperOuter);

  const upperInner = box(18, upperArmLen - 10, 16, MAT.darkSteel, `${prefix} Upper Arm Frame`);
  upperInner.position.y = -upperArmLen / 2;
  upperArmGroup.add(upperInner);

  for (let s of [-1, 1]) {
    const act = cyl(5, 5, upperArmLen * 0.55, 12, MAT.aluminium);
    act.position.set(s * 10, -upperArmLen / 2, 0);
    upperArmGroup.add(act);
    const groove = box(3, upperArmLen * 0.45, 4, MAT.carbonFiber);
    groove.position.set(s * 16, -upperArmLen / 2, 0);
    upperArmGroup.add(groove);
  }

  // ── ELBOW — rotation negated \u00d7 side so positive joint[2] = bends DOWN for both arms
  // (because the tPose rotation flips the effective bend direction per side)
  const elbowGroup = new THREE.Group();
  elbowGroup.position.y = -upperArmLen;
  elbowGroup.rotation.z = -joints[2] * side;
  upperArmGroup.add(elbowGroup);

  const elbowHousing = cyl(14, 14, 32, 20, MAT.blackAnodised, `${prefix} Elbow Housing`);
  elbowHousing.rotation.z = Math.PI / 2;
  elbowGroup.add(elbowHousing);

  for (let s of [-1, 1]) {
    const eb = cyl(16, 18, 8, 24, MAT.chrome, `${prefix} Elbow Bearing`);
    eb.rotation.z = Math.PI / 2;
    eb.position.x = s * 16;
    elbowGroup.add(eb);
  }

  const elbowMotor = box(18, 24, 18, MAT.darkSteel, `${prefix} Elbow Motor`);
  elbowMotor.position.set(0, 10, 0);
  elbowGroup.add(elbowMotor);

  // ── FOREARM
  const forearmGroup = new THREE.Group();
  forearmGroup.position.y = -8;
  elbowGroup.add(forearmGroup);

  const foreOuter = box(22, forearmLen, 20, MAT.whitePolycarbonate, `${prefix} Forearm Shell`);
  foreOuter.position.y = -forearmLen / 2;
  forearmGroup.add(foreOuter);

  const foreInner = box(14, forearmLen - 8, 12, MAT.aluminium, `${prefix} Forearm Frame`);
  foreInner.position.y = -forearmLen / 2;
  forearmGroup.add(foreInner);

  const twistRing = ring(9, 14, 12, 24, MAT.chrome, `${prefix} Twist Ring`);
  twistRing.position.y = -forearmLen * 0.35;
  forearmGroup.add(twistRing);

  for (let i = 0; i < 3; i++) {
    const tendon = cyl(2.5, 2.5, 5, 8, MAT.brass);
    tendon.rotation.x = Math.PI / 2;
    tendon.position.set(0, -forearmLen * (0.3 + i * 0.2), 11);
    forearmGroup.add(tendon);
  }

  // ── WRIST
  const wristGroup = new THREE.Group();
  wristGroup.position.y = -forearmLen;
  wristGroup.rotation.x = joints[3];          // flex forward/back
  wristGroup.rotation.z = -joints[4] * side;  // rotation, mirrored
  forearmGroup.add(wristGroup);

  const wristBall = sphere(12, 16, MAT.chrome, `${prefix} Wrist Ball`);
  wristGroup.add(wristBall);

  const wristHousing = cyl(14, 18, 18, 24, MAT.blackAnodised, `${prefix} Wrist Housing`);
  wristHousing.position.y = -9;
  wristGroup.add(wristHousing);

  // ── PALM
  const palmGroup = new THREE.Group();
  palmGroup.position.y = -20;
  wristGroup.add(palmGroup);

  const palmBody = box(54, palmLen, 18, MAT.darkSteel, `${prefix} Palm`);
  palmBody.position.y = -palmLen / 2;
  palmGroup.add(palmBody);

  const knucklePlate = box(58, 8, 16, MAT.blackAnodised, `${prefix} Knuckle Plate`);
  knucklePlate.position.y = -palmLen - 2;
  palmGroup.add(knucklePlate);

  for (let i = 0; i < 5; i++) {
    const sv = box(7, 8, 10, MAT.aluminium);
    sv.position.set(-20 + i * 10, -palmLen * 0.5, 0);
    palmGroup.add(sv);
  }

  // ── FINGERS — 5 fingers, 3 phalanges each, curl on X-axis
  const thumbX = side * 27;
  const fingerDefs = [
    [`${prefix} Thumb`,  thumbX,  9, joints[5],  joints[6],  joints[7],  [26,20,16], side * 0.45 ],
    [`${prefix} Index`,     -18, -2, joints[8],  joints[9],  joints[10], [32,24,16], 0           ],
    [`${prefix} Middle`,     -6, -2, joints[11], joints[12], joints[13], [34,26,18], 0           ],
    [`${prefix} Ring`,        6, -2, joints[14], joints[15], joints[16], [30,24,16], 0           ],
    [`${prefix} Pinky`,      18, -2, joints[17], joints[18], joints[19], [24,18,14], 0           ],
  ];

  for (const [name, fx, fz, jB, jM, jT, len, spread] of fingerDefs) {
    const fingerRoot = new THREE.Group();
    fingerRoot.position.set(fx, -palmLen - 5, fz);
    fingerRoot.rotation.z = spread;
    palmGroup.add(fingerRoot);

    const knuckleBall = sphere(4.5, 10, MAT.chrome, `${name} Knuckle`);
    fingerRoot.add(knuckleBall);

    const p1w = name.includes('Thumb') ? 8 : 6;

    // Proximal phalange — rotation.x = curl forward/back
    const p1Group = new THREE.Group();
    p1Group.rotation.x = jB;
    fingerRoot.add(p1Group);

    const p1 = box(p1w, len[0], 9, MAT.whitePolycarbonate, `${name} Proximal`);
    p1.position.y = -len[0] / 2;
    p1Group.add(p1);

    const j1 = cyl(4.5, 4.5, p1w + 4, 10, MAT.chrome, `${name} IP1`);
    j1.rotation.z = Math.PI / 2;
    j1.position.y = -len[0];
    p1Group.add(j1);

    // Medial phalange
    const p2Group = new THREE.Group();
    p2Group.position.y = -len[0];
    p2Group.rotation.x = jM;
    p1Group.add(p2Group);

    const p2 = box(p1w - 1, len[1], 8, MAT.whitePolycarbonate, `${name} Medial`);
    p2.position.y = -len[1] / 2;
    p2Group.add(p2);

    const j2 = cyl(3.5, 3.5, p1w + 2, 10, MAT.chrome, `${name} IP2`);
    j2.rotation.z = Math.PI / 2;
    j2.position.y = -len[1];
    p2Group.add(j2);

    // Distal phalange (fingertip)
    const p3Group = new THREE.Group();
    p3Group.position.y = -len[1];
    p3Group.rotation.x = jT;
    p2Group.add(p3Group);

    const p3 = mesh(
      new THREE.CylinderGeometry(1.8, p1w / 2 - 0.5, len[2], 10),
      MAT.rubber,
      `${name} Tip`
    );
    p3.position.y = -len[2] / 2;
    p3Group.add(p3);

    const nail = box(p1w - 3, len[2] * 0.6, 2.5, MAT.aluminium, `${name} Nail`);
    nail.position.set(0, -len[2] * 0.38, 4.5);
    p3Group.add(nail);
  }
}

export function buildDexArm(joints, params) {
  const {
    upperArmLen = 120,
    forearmLen  = 110,
    palmLen     = 50,
  } = params;

  const root = new THREE.Group();

  // ── TORSO COLUMN
  const torsoH = 180;
  const torsoW = 80;

  // Main torso body
  const torsoBody = box(torsoW, torsoH, 50, MAT.whitePolycarbonate, 'Torso Body');
  torsoBody.position.y = torsoH / 2;
  root.add(torsoBody);

  // Torso inner structural spine
  const torsoSpine = box(torsoW - 20, torsoH - 10, 36, MAT.darkSteel, 'Torso Spine');
  torsoSpine.position.y = torsoH / 2;
  root.add(torsoSpine);

  // Torso accent panel (cyan strip down the front)
  const torsoAccent = box(20, torsoH - 30, 8, MAT.cyan, 'Torso Accent');
  torsoAccent.position.set(0, torsoH / 2, 27);
  root.add(torsoAccent);

  // Torso top collar
  const collar = cyl(30, 36, 18, 24, MAT.blackAnodised, 'Torso Collar');
  collar.position.y = torsoH + 9;
  root.add(collar);

  // Torso bottom base plate
  const basePlate = box(torsoW + 20, 16, 60, MAT.darkSteel, 'Torso Base Plate');
  basePlate.position.y = 8;
  root.add(basePlate);

  // Mounting rail on each side (where arms attach)
  for (let s of [-1, 1]) {
    const rail = box(12, torsoH * 0.7, 14, MAT.aluminium, s > 0 ? 'R Shoulder Rail' : 'L Shoulder Rail');
    rail.position.set(s * (torsoW / 2 + 6), torsoH * 0.65, 0);
    root.add(rail);
  }

  // ── LEFT ARM  (joints 0–19, side = -1)
  const leftMount = new THREE.Group();
  leftMount.position.set(-(torsoW / 2 + 18), torsoH * 0.82, 0);
  root.add(leftMount);
  buildSingleArm(leftMount,  -1, joints, upperArmLen, forearmLen, palmLen, 'L');

  // ── RIGHT ARM  (joints mirrored, side = +1)
  const rightMount = new THREE.Group();
  rightMount.position.set( (torsoW / 2 + 18), torsoH * 0.82, 0);
  root.add(rightMount);
  buildSingleArm(rightMount, +1, joints, upperArmLen, forearmLen, palmLen, 'R');

  // Lift torso so arms hang clear of the ground
  // Shoulder height = 150 + 180*0.82 = ~298; arm length = ~375 → hands near y=0
  root.position.y = 150;
  return root;
}

// FK helper — returns zeros (IK not active for this robot)
export function dexArmFK(joints, params) {
  return { x: 0, y: 0, z: 0 };
}

// ─────────────────────────────────────────────────────────────
// ROBOT 7 — QUADCOPTER DRONE
// ─────────────────────────────────────────────────────────────
export function buildDrone(joints, params) {
  const { armLen = 140, bodySize = 60 } = params;
  const root = new THREE.Group();
  root.position.y = 80;

  // Central body
  const bodyTop = box(bodySize, 20, bodySize, MAT.blackAnodised, 'Drone Body');
  root.add(bodyTop);

  // Battery plate
  const battery = box(bodySize - 10, 12, bodySize - 12, MAT.orange, 'Battery Pack');
  battery.position.y = -14;
  root.add(battery);

  // Flight controller PCB
  const fcb = box(30, 4, 30, MAT.darkSteel, 'Flight Controller');
  fcb.position.y = 14;
  root.add(fcb);

  // Camera
  const cam = box(20, 16, 22, MAT.titanium, 'Camera');
  cam.position.set(0, -2, bodySize / 2 + 4);
  root.add(cam);

  // 4 arms at 45° angles
  const armAngles = [45, 135, 225, 315];
  const motorNames = ['Motor FR', 'Motor FL', 'Motor BL', 'Motor BR'];

  for (let i = 0; i < 4; i++) {
    const angle = (armAngles[i] * Math.PI) / 180;
    const ax = Math.cos(angle) * armLen;
    const az = Math.sin(angle) * armLen;

    // Arm tube
    const arm = cyl(4, 4, armLen * 2, 8, MAT.carbonFiber, `Arm ${i + 1}`);
    arm.rotation.z = Math.PI / 2;
    arm.rotation.y = angle;
    arm.position.set(ax / 2, 2, az / 2);
    root.add(arm);

    // Motor pod
    const motorPod = cyl(16, 16, 20, 16, MAT.darkSteel, motorNames[i]);
    motorPod.position.set(ax, 8, az);
    root.add(motorPod);

    // Prop (spinning angle from joint)
    const propAngle = joints[i] || 0;
    const propGroup = new THREE.Group();
    propGroup.position.set(ax, 20, az);
    propGroup.rotation.y = propAngle;
    root.add(propGroup);

    // Two blades
    for (let b of [-1, 1]) {
      const blade = box(armLen * 0.55, 4, 16, MAT.carbonFiber, `Prop Blade`);
      blade.position.x = b * armLen * 0.28;
      blade.rotation.z = b * 0.12;
      propGroup.add(blade);
    }

    // Motor nut (chrome detail)
    const nut = cyl(5, 5, 6, 6, MAT.chrome);
    nut.position.set(ax, 22, az);
    root.add(nut);
  }

  // Landing gear
  for (let side of [-1, 1]) {
    const leg = cyl(3, 3, 50, 8, MAT.aluminium, side > 0 ? 'Landing Leg R' : 'Landing Leg L');
    leg.position.set(side * (bodySize / 2 - 4), -30, 0);
    leg.rotation.z = side * 0.3;
    root.add(leg);

    const foot = cyl(3, 3, bodySize, 8, MAT.rubber, side > 0 ? 'Landing Skid R' : 'Landing Skid L');
    foot.rotation.z = Math.PI / 2;
    foot.position.set(0, -52, 0);
    root.add(foot);
  }

  return root;
}

// ─────────────────────────────────────────────────────────────
// ROBOT CONFIG DEFINITIONS
// ─────────────────────────────────────────────────────────────
export const ROBOTS = {
  arm: {
    name: '6-DOF Robotic Arm',
    builder: buildArm,
    fk: armFK,
    joints: [0, Math.PI / 6, -Math.PI / 3, -Math.PI / 6, 18],
    jointNames: ['Base Yaw', 'Shoulder Pitch', 'Elbow Flex', 'Wrist Pitch', 'Gripper Gap'],
    jointLimits: [
      { min: -180, max: 180, step: 1, isAngle: true },
      { min: -90,  max: 90,  step: 1, isAngle: true },
      { min: -135, max: 135, step: 1, isAngle: true },
      { min: -90,  max: 90,  step: 1, isAngle: true },
      { min: 8,    max: 45,  step: 1, isAngle: false },
    ],
    params: { l1: 80, l2: 130, l3: 110, l4: 80 },
    paramDefs: [
      { label: 'Base Height',   key: 'l1', min: 50,  max: 150, step: 5,  unit: 'mm' },
      { label: 'Upper Arm',     key: 'l2', min: 80,  max: 220, step: 5,  unit: 'mm' },
      { label: 'Forearm',       key: 'l3', min: 60,  max: 180, step: 5,  unit: 'mm' },
      { label: 'Wrist Link',    key: 'l4', min: 40,  max: 120, step: 5,  unit: 'mm' },
    ],
    ikSupported: true,
  },

  humanoid: {
    name: 'Humanoid Robot',
    builder: buildHumanoid,
    fk: null,
    joints: [0, 0, 0, -Math.PI/4, 0, 0, -Math.PI/4, -Math.PI/12, Math.PI/5, -Math.PI/12, -Math.PI/12, Math.PI/5, -Math.PI/12],
    jointNames: [
      'Torso Yaw',
      'L-Shoulder', 'L-Shoulder Roll', 'L-Elbow',
      'R-Shoulder', 'R-Shoulder Roll', 'R-Elbow',
      'L-Hip', 'L-Knee', 'L-Ankle',
      'R-Hip', 'R-Knee', 'R-Ankle',
    ],
    jointLimits: [
      { min: -90,  max: 90,  step: 1, isAngle: true },
      { min: -180, max: 90,  step: 1, isAngle: true },
      { min: -10,  max: 110, step: 1, isAngle: true },
      { min: -130, max: 0,   step: 1, isAngle: true },
      { min: -180, max: 90,  step: 1, isAngle: true },
      { min: -10,  max: 110, step: 1, isAngle: true },
      { min: -130, max: 0,   step: 1, isAngle: true },
      { min: -60,  max: 60,  step: 1, isAngle: true },
      { min: 0,    max: 120, step: 1, isAngle: true },
      { min: -60,  max: 60,  step: 1, isAngle: true },
      { min: -60,  max: 60,  step: 1, isAngle: true },
      { min: 0,    max: 120, step: 1, isAngle: true },
      { min: -60,  max: 60,  step: 1, isAngle: true },
    ],
    params: { hipSpacing: 90, thigh: 100, shin: 110, footH: 18 },
    paramDefs: [
      { label: 'Hip Spacing', key: 'hipSpacing', min: 60,  max: 130, step: 5,  unit: 'mm' },
      { label: 'Thigh',       key: 'thigh',      min: 70,  max: 150, step: 5,  unit: 'mm' },
      { label: 'Shin',        key: 'shin',        min: 70,  max: 160, step: 5,  unit: 'mm' },
      { label: 'Foot Height', key: 'footH',       min: 10,  max: 28,  step: 2,  unit: 'mm' },
    ],
    ikSupported: false,
  },

  quadruped: {
    name: 'Quadruped Robot',
    builder: buildQuadruped,
    fk: null,
    joints: Array(12).fill(0).map((_, i) => [0, Math.PI/8, -Math.PI/4][i % 3]),
    jointNames: [
      'FR Hip Yaw', 'FR Hip Pitch', 'FR Knee',
      'FL Hip Yaw', 'FL Hip Pitch', 'FL Knee',
      'BL Hip Yaw', 'BL Hip Pitch', 'BL Knee',
      'BR Hip Yaw', 'BR Hip Pitch', 'BR Knee',
    ],
    jointLimits: Array(4).fill([
      { min: -45,  max: 45,  step: 1, isAngle: true },
      { min: -60,  max: 60,  step: 1, isAngle: true },
      { min: -110, max: 0,   step: 1, isAngle: true },
    ]).flat(),
    params: { coxa: 40, femur: 80, tibia: 85, bodyLen: 160, bodyW: 100 },
    paramDefs: [
      { label: 'Coxa (Hip)',  key: 'coxa',    min: 25,  max: 60,  step: 5,  unit: 'mm' },
      { label: 'Femur',      key: 'femur',   min: 50,  max: 120, step: 5,  unit: 'mm' },
      { label: 'Tibia',      key: 'tibia',   min: 50,  max: 130, step: 5,  unit: 'mm' },
      { label: 'Body Length',key: 'bodyLen', min: 100, max: 250, step: 10, unit: 'mm' },
      { label: 'Body Width', key: 'bodyW',   min: 70,  max: 160, step: 10, unit: 'mm' },
    ],
    ikSupported: false,
  },

  rover: {
    name: 'Mars Rover',
    builder: buildRover,
    fk: null,
    joints: [0, 0, 0, 0, 0, 0],
    jointNames: ['Front Wheels', 'Mid Wheels', 'Rear Wheels', 'Arm Joint 1', 'Arm Joint 2', 'Arm Joint 3'],
    jointLimits: Array(6).fill({ min: -360, max: 360, step: 5, isAngle: true }),
    params: { chassisL: 200, chassisW: 140, wheelR: 40, wheelW: 20 },
    paramDefs: [
      { label: 'Chassis Length', key: 'chassisL', min: 150, max: 300, step: 10, unit: 'mm' },
      { label: 'Chassis Width',  key: 'chassisW', min: 100, max: 200, step: 10, unit: 'mm' },
      { label: 'Wheel Radius',   key: 'wheelR',   min: 25,  max: 65,  step: 5,  unit: 'mm' },
      { label: 'Wheel Width',    key: 'wheelW',   min: 12,  max: 35,  step: 2,  unit: 'mm' },
    ],
    ikSupported: false,
  },

  scara: {
    name: 'SCARA Arm',
    builder: buildSCARA,
    fk: null,
    joints: [Math.PI / 5, Math.PI / 5, 30, 0],
    jointNames: ['Shoulder Axis', 'Elbow Axis', 'Z Slide', 'Tool Rotation'],
    jointLimits: [
      { min: -150, max: 150, step: 1, isAngle: true },
      { min: -150, max: 150, step: 1, isAngle: true },
      { min: 0,    max: 130, step: 1, isAngle: false },
      { min: -180, max: 180, step: 1, isAngle: true },
    ],
    params: { l1: 150, l2: 130, pedestalH: 190 },
    paramDefs: [
      { label: 'Inner Link',    key: 'l1',        min: 100, max: 240, step: 10, unit: 'mm' },
      { label: 'Outer Link',    key: 'l2',        min: 80,  max: 200, step: 10, unit: 'mm' },
      { label: 'Pedestal Ht.', key: 'pedestalH', min: 120, max: 280, step: 10, unit: 'mm' },
    ],
    ikSupported: false,
  },

  drone: {
    name: 'Quadcopter Drone',
    builder: buildDrone,
    fk: null,
    joints: [0, 0, 0, 0],
    jointNames: ['Rotor FR', 'Rotor FL', 'Rotor BL', 'Rotor BR'],
    jointLimits: Array(4).fill({ min: -720, max: 720, step: 10, isAngle: true }),
    params: { armLen: 140, bodySize: 60 },
    paramDefs: [
      { label: 'Arm Length',  key: 'armLen',   min: 80,  max: 220, step: 10, unit: 'mm' },
      { label: 'Body Size',   key: 'bodySize', min: 40,  max: 100, step: 5,  unit: 'mm' },
    ],
    ikSupported: false,
  },

  dexarm: {
    name: 'Dexterous Bimanual Arm',
    builder: buildDexArm,
    fk: dexArmFK,
    // 20 shared joints — applied symmetrically to both arms
    joints: [
      // [0] Shoulder Pitch  [1] Shoulder Yaw
      0, 0,
      // [2] Elbow Flex — positive = bends forearm DOWN from horizontal T-pose
      Math.PI * 5 / 12,   // ~75° → forearms hanging naturally
      // [3] Wrist Flex  [4] Wrist Rotation
      0, 0,
      // [5-7] Thumb: Base / Mid / Tip
      Math.PI / 12, Math.PI / 16, 0,
      // [8-10] Index: Base / Mid / Tip
      0, Math.PI / 20, 0,
      // [11-13] Middle: Base / Mid / Tip
      0, Math.PI / 20, 0,
      // [14-16] Ring: Base / Mid / Tip
      0, Math.PI / 20, 0,
      // [17-19] Pinky: Base / Mid / Tip
      0, Math.PI / 20, 0,
    ],
    jointNames: [
      'Shoulder Pitch', 'Shoulder Yaw',
      'Elbow Flex',
      'Wrist Flex', 'Wrist Rotation',
      'Thumb Base', 'Thumb Mid', 'Thumb Tip',
      'Index Base', 'Index Mid', 'Index Tip',
      'Middle Base', 'Middle Mid', 'Middle Tip',
      'Ring Base', 'Ring Mid', 'Ring Tip',
      'Pinky Base', 'Pinky Mid', 'Pinky Tip',
    ],
    jointLimits: [
      { min: -60,  max: 60,  step: 1, isAngle: true },  // Shoulder Pitch
      { min: -80,  max: 80,  step: 1, isAngle: true },  // Shoulder Yaw
      { min: 0,    max: 145, step: 1, isAngle: true },  // Elbow Flex
      { min: -60,  max: 60,  step: 1, isAngle: true },  // Wrist Flex
      { min: -90,  max: 90,  step: 1, isAngle: true },  // Wrist Rotation
      { min: -40,  max: 80,  step: 1, isAngle: true },  // Thumb Base
      { min: 0,    max: 90,  step: 1, isAngle: true },  // Thumb Mid
      { min: 0,    max: 80,  step: 1, isAngle: true },  // Thumb Tip
      { min: -10,  max: 90,  step: 1, isAngle: true },  // Index Base
      { min: 0,    max: 110, step: 1, isAngle: true },  // Index Mid
      { min: 0,    max: 90,  step: 1, isAngle: true },  // Index Tip
      { min: -10,  max: 90,  step: 1, isAngle: true },  // Middle Base
      { min: 0,    max: 110, step: 1, isAngle: true },  // Middle Mid
      { min: 0,    max: 90,  step: 1, isAngle: true },  // Middle Tip
      { min: -10,  max: 90,  step: 1, isAngle: true },  // Ring Base
      { min: 0,    max: 110, step: 1, isAngle: true },  // Ring Mid
      { min: 0,    max: 90,  step: 1, isAngle: true },  // Ring Tip
      { min: -10,  max: 90,  step: 1, isAngle: true },  // Pinky Base
      { min: 0,    max: 110, step: 1, isAngle: true },  // Pinky Mid
      { min: 0,    max: 90,  step: 1, isAngle: true },  // Pinky Tip
    ],
    params: { upperArmLen: 120, forearmLen: 110, palmLen: 50 },
    paramDefs: [
      { label: 'Upper Arm', key: 'upperArmLen', min: 80,  max: 180, step: 5, unit: 'mm' },
      { label: 'Forearm',   key: 'forearmLen',  min: 70,  max: 160, step: 5, unit: 'mm' },
      { label: 'Palm',      key: 'palmLen',     min: 35,  max: 80,  step: 5, unit: 'mm' },
    ],
    ikSupported: false,
  },
};
