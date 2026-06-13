/**
 * robots.js
 * Builds detailed procedural 3D robot meshes using THREE.js geometry.
 * Every robot is a THREE.Group with named sub-meshes for STL export.
 */
import * as THREE from 'three';
import { MAT } from './materials.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { DHChain, dhToWorld, scaraFK, matMul, matRotX, matRotY, matRotZ, matTrans, matPoint } from './kinematics.js';

export const ARM_FLANGE = 30;

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

// Rounded box — same signature as box(), with bevel radius
function rbox(w, h, d, r, mat, name) {
  return mesh(new RoundedBoxGeometry(w, h, d, 3, Math.min(r, w / 2, h / 2, d / 2)), mat, name);
}

// Capsule limb segment, axis along Y
function capsule(radius, length, mat, name) {
  return mesh(new THREE.CapsuleGeometry(radius, length, 6, 20), mat, name);
}

// Cylinder with a chamfered (truncated-cone) edge top and bottom
function chamferCyl(r, h, chamfer, segments, mat, name) {
  const g = new THREE.Group();
  if (name) g.name = name;
  const body = cyl(r, r, h - 2 * chamfer, segments, mat);
  g.add(body);
  const top = cyl(r - chamfer, r, chamfer, segments, mat);
  top.position.y = h / 2 - chamfer / 2;
  g.add(top);
  const bot = cyl(r, r - chamfer, chamfer, segments, mat);
  bot.position.y = -h / 2 + chamfer / 2;
  g.add(bot);
  return g;
}

// Ring of bolt heads on a joint face (lies in XZ plane, +Y up)
function boltCircle(radius, n, mat) {
  const g = new THREE.Group();
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2;
    const bolt = cyl(1.6, 1.6, 2, 6, mat);
    bolt.position.set(Math.cos(a) * radius, 0, Math.sin(a) * radius);
    g.add(bolt);
  }
  return g;
}

// Strip of recessed vent slots across width w (slots along X)
function vents(w, n, mat) {
  const g = new THREE.Group();
  const slotW = (w * 0.7) / n;
  for (let i = 0; i < n; i++) {
    const slot = box(slotW * 0.55, 1.2, 6, mat);
    slot.position.x = -w * 0.35 + (i + 0.5) * slotW;
    g.add(slot);
  }
  return g;
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

// Joint actuator drum lying across a joint axis (X): black anodised body,
// cyan accent ring + end cap on each face. Name goes on the body mesh so it
// stays in the STL export list.
function actuator(r, w, name) {
  const g = new THREE.Group();
  g.add(cyl(r, r, w, 24, MAT.blackAnodised, name));
  for (const s of [-1, 1]) {
    const accent = mesh(new THREE.TorusGeometry(r * 0.62, 1.1, 8, 24), MAT.cyan);
    accent.rotation.x = Math.PI / 2;
    accent.position.y = s * (w / 2 + 0.4);
    g.add(accent);
    const cap = cyl(r * 0.78, r * 0.78, 2, 24, MAT.blackAnodised);
    cap.position.y = s * (w / 2 + 1);
    g.add(cap);
  }
  g.rotation.z = Math.PI / 2;
  return g;
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

  // base pedestal — chamfered aluminium disc, bolt circle on the top face
  const basePedestal = chamferCyl(95, 18, 4, 48, MAT.aluminium);
  basePedestal.children[0].name = 'Base Mount Plate';
  basePedestal.position.y = 9;
  baseGroup.add(basePedestal);

  const baseBolts = boltCircle(75, 8, MAT.steelDark);
  baseBolts.position.y = 18;
  baseGroup.add(baseBolts);

  // base top ring bearing
  const b0 = bearing(60, 10);
  b0.position.y = 20;
  baseGroup.add(b0);

  // rotating turret
  const turret = new THREE.Group();
  turret.rotation.y = joints[0];
  baseGroup.add(turret);

  const turretBody = chamferCyl(46, 50, 5, 32, MAT.blackAnodised);
  turretBody.children[0].name = 'Base Turret';
  turretBody.position.y = 45;
  turret.add(turretBody);

  // turret accent ring
  const turretAccent = mesh(new THREE.TorusGeometry(43, 1.2, 8, 32), MAT.cyan);
  turretAccent.rotation.x = Math.PI / 2;
  turretAccent.position.y = 62;
  turret.add(turretAccent);

  // shoulder bearing
  const b1 = bearing(36, 12);
  b1.position.y = l1 + 20;
  turret.add(b1);

  // ── UPPER ARM
  const upperGroup = new THREE.Group();
  upperGroup.position.y = l1 + 20;
  upperGroup.rotation.z = joints[1];
  turret.add(upperGroup);

  const upperLink = rbox(32, l2, 24, 8, MAT.whitePolycarbonate, 'Upper Arm Housing');
  upperLink.position.y = l2 / 2;
  upperGroup.add(upperLink);

  // raised aluminium spine along the outer face
  const upperSpine = rbox(7, l2 * 0.78, 4, 2, MAT.aluminium);
  upperSpine.position.set(0, l2 / 2, 13.5);
  upperGroup.add(upperSpine);

  // shoulder actuator
  const shoulderActuator = actuator(20, 36, 'Shoulder Actuator');
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

  const foreLink = rbox(26, l3, 20, 8, MAT.whitePolycarbonate, 'Forearm Housing');
  foreLink.position.y = l3 / 2;
  forearmGroup.add(foreLink);

  const foreSpine = rbox(6, l3 * 0.74, 3.5, 1.7, MAT.aluminium);
  foreSpine.position.set(0, l3 / 2, 11.4);
  forearmGroup.add(foreSpine);

  const elbowActuator = actuator(16, 32, 'Elbow Actuator');
  elbowActuator.position.y = 2;
  forearmGroup.add(elbowActuator);

  // wrist bearing
  const b3 = bearing(22, 8);
  b3.position.y = l3;
  forearmGroup.add(b3);

  // ── WRIST 1 (pitch)
  const wristGroup = new THREE.Group();
  wristGroup.position.y = l3;
  wristGroup.rotation.z = joints[3];
  forearmGroup.add(wristGroup);

  const wristActuator = actuator(13, 24, 'Wrist 1 Actuator');
  wristActuator.position.y = 2;
  wristGroup.add(wristActuator);

  // ── WRIST 2 (roll about the link axis)
  const rollGroup = new THREE.Group();
  rollGroup.rotation.y = joints[4];
  wristGroup.add(rollGroup);

  const wristLink = capsule(10, Math.max(l4 - 20, 8), MAT.whitePolycarbonate, 'Wrist Link');
  wristLink.position.y = l4 / 2;
  rollGroup.add(wristLink);

  const rollRing = bearing(14, 6);
  rollRing.position.y = l4 * 0.45;
  rollGroup.add(rollRing);

  // ── WRIST 3 (pitch)
  const wrist2Group = new THREE.Group();
  wrist2Group.position.y = l4;
  wrist2Group.rotation.z = joints[5];
  rollGroup.add(wrist2Group);

  const wrist2Actuator = actuator(11, 20, 'Wrist 3 Actuator');
  wrist2Group.add(wrist2Actuator);

  // ── FLANGE + GRIPPER (FLANGE = 30 must match the DH table)
  const gripGroup = new THREE.Group();
  gripGroup.position.y = ARM_FLANGE;
  wrist2Group.add(gripGroup);

  const gripBase = chamferCyl(14, 18, 3, 20, MAT.blackAnodised);
  gripBase.children[0].name = 'Gripper Base';
  gripBase.position.y = -9;
  gripGroup.add(gripBase);

  const clawGap = joints[6] !== undefined ? joints[6] : 18;
  for (let side of [-1, 1]) {
    const claw = new THREE.Group();
    claw.position.set(side * clawGap / 2, 0, 0);

    const finger = rbox(6, 30, 8, 2, MAT.aluminium, side > 0 ? 'Gripper Finger Right' : 'Gripper Finger Left');
    finger.position.y = 15;
    claw.add(finger);

    // rubber grip pad on the inner face
    const pad = box(2, 20, 6, MAT.rubber);
    pad.position.set(-side * 4, 16, 0);
    claw.add(pad);

    const tip = box(8, 10, 10, MAT.rubber, 'Gripper Tip');
    tip.position.y = 34;
    claw.add(tip);
    gripGroup.add(claw);
  }

  // decorative cable conduit following the zero-pose arm (root-parented)
  const shoulderY = l1 + 20;
  const conduitCurve = new THREE.CatmullRomCurve3([
    new THREE.Vector3(0, 48, 34),
    new THREE.Vector3(0, shoulderY - 8, 27),
    new THREE.Vector3(0, shoulderY + l2 * 0.5, 19),
    new THREE.Vector3(0, shoulderY + l2, 23),
    new THREE.Vector3(0, shoulderY + l2 + l3 * 0.55, 16),
    new THREE.Vector3(0, shoulderY + l2 + l3 - 6, 12),
  ]);
  const conduit = mesh(new THREE.TubeGeometry(conduitCurve, 32, 3.5, 8, false), MAT.rubber);
  root.add(conduit);

  return root;
}

export function armDHRows(q, p) {
  return [
    [q[0],                p.l1 + 20, 0,    Math.PI / 2],
    [q[1] + Math.PI / 2,  0,         p.l2, 0],
    [q[2],                0,         p.l3, 0],
    [q[3] + Math.PI / 2,  0,         0,    Math.PI / 2],
    [q[4],                p.l4,      0,   -Math.PI / 2],
    [q[5] - Math.PI / 2,  0,         ARM_FLANGE, 0],
  ];
}

// FK for telemetry — world (Y-up) coordinates
export function armFK(joints, params) {
  const chain = new DHChain((q) => armDHRows(q, params));
  const [x, y, z] = dhToWorld(chain.eePos(joints.slice(0, 6)));
  return { x, y, z };
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

  // Pelvis — rounded shell
  const pelvis = rbox(hipSpacing + 30, 30, 40, 10, MAT.darkPolycarbonate, 'Pelvis');
  pelvis.position.y = 0;
  torso.add(pelvis);

  // Abdomen — waist shell, slimmer than chest
  const abdomen = rbox(58, 50, 34, 12, MAT.whitePolycarbonate, 'Abdomen Panel');
  abdomen.position.y = 45;
  torso.add(abdomen);

  // Chest — smooth clearcoat shell
  const chest = rbox(80, 70, 38, 12, MAT.whitePolycarbonate, 'Chest Panel');
  chest.position.y = 110;
  torso.add(chest);

  // Dark chest plate inset — sits proud of the shell front
  const chestPlate = rbox(54, 46, 6, 6, MAT.darkPolycarbonate, 'Chest Plate');
  chestPlate.position.set(0, 112, 18);
  torso.add(chestPlate);

  // Chest vent strip across the upper chest
  const chestVents = vents(54, 6, MAT.darkSteel);
  chestVents.position.set(0, 132, 19);
  torso.add(chestVents);

  // Chest internal frame (hidden structural mass for COM/printability)
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

  const headBody = rbox(48, 50, 40, 8, MAT.whitePolycarbonate, 'Head Shell');
  headGroup.add(headBody);

  // Glossy dark face visor — wraps the front, the robot's "face"
  const visor = rbox(42, 24, 8, 5, MAT.darkPolycarbonate, 'Vision Visor');
  visor.position.set(0, 2, 18);
  headGroup.add(visor);

  // Sensor accent inside the visor
  const sensorBar = box(28, 3, 2, MAT.cyan);
  sensorBar.position.set(0, 4, 22.5);
  headGroup.add(sensorBar);

  // Shoulders
  for (let side of [-1, 1]) {
    const shoulderGroup = new THREE.Group();
    shoulderGroup.position.set(side * (hipSpacing / 2 + 30), 120, 0);
    torso.add(shoulderGroup);

    const shoulderJoint = sphere(16, 16, MAT.chrome, 'Shoulder Joint');
    shoulderGroup.add(shoulderJoint);

    // Upper arm — euler XYZ: RX(roll) tilts the whole swing plane fore/aft,
    // RZ(pitch) swings the arm within that plane
    const armG = new THREE.Group();
    armG.rotation.x = side > 0 ? joints[5] : joints[2];
    armG.rotation.z = side > 0 ? joints[4] : joints[1];
    shoulderGroup.add(armG);

    // Upper arm capsule: box was 22 wide, 80 tall, centre y=-40.
    // radius 10 → capsule spans 60+20=80 along Y, same envelope/centre.
    const upperArm = capsule(10, 60, MAT.whitePolycarbonate, side > 0 ? 'R Upper Arm' : 'L Upper Arm');
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

    // Forearm capsule: box was 18 wide, 75 tall, centre y=-38.
    // radius 8.5 → spans 58+17=75, same envelope/centre.
    const forearm = capsule(8.5, 58, MAT.whitePolycarbonate, side > 0 ? 'R Forearm' : 'L Forearm');
    forearm.position.y = -38;
    forearmG.add(forearm);

    // Hand — rounded shell palm
    const hand = rbox(22, 20, 16, 4, MAT.darkPolycarbonate, side > 0 ? 'R Hand' : 'L Hand');
    hand.position.y = -82;
    forearmG.add(hand);

    // Fingers (simplified 3 fingers) — slim capsules
    for (let fi = -1; fi <= 1; fi++) {
      const finger = capsule(2.4, 15, MAT.darkSteel);
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

    // Thigh capsule: box was 28 wide, length=thigh, centre -thigh/2.
    // radius 13 → spans (thigh-26)+26 = thigh, same envelope/centre.
    const thighMesh = capsule(13, thigh - 26, MAT.whitePolycarbonate, side > 0 ? 'R Thigh' : 'L Thigh');
    thighMesh.position.y = -thigh / 2;
    thighG.add(thighMesh);

    // Thigh actuator cover — dark contrast core just inside the shell
    const thighActuator = cyl(10, 10, thigh - 18, 20, MAT.darkPolycarbonate, side > 0 ? 'R Thigh Actuator' : 'L Thigh Actuator');
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
    // Shin capsule: box was 24 wide, length=shin, centre -shin/2.
    // radius 11 → spans (shin-22)+22 = shin, same envelope/centre.
    const shinMesh = capsule(11, shin - 22, MAT.darkPolycarbonate, side > 0 ? 'R Shin' : 'L Shin');
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

  // Body + legs live in one group so a body pose transform moves hips
  // while leg IK keeps feet planted.
  const bodyGroup = new THREE.Group();
  bodyGroup.name = 'BodyGroup';
  root.add(bodyGroup);

  // Optional body pose injected by the IK controller, which owns the key:
  // it sets _bodyPose before each rebuild and deletes it on deactivate.
  const bp = params._bodyPose;
  if (bp) {
    bodyGroup.position.set(bp.x || 0, bp.y || 0, bp.z || 0);
    bodyGroup.rotation.set(bp.rx || 0, bp.ry || 0, bp.rz || 0);
  }

  const bodyY = femur + tibia + 10;

  // Body — rounded polycarbonate shell (Spot torso)
  const body = rbox(bodyW, 30, bodyLen, 10, MAT.darkPolycarbonate, 'Main Body');
  body.position.y = bodyY;
  bodyGroup.add(body);

  // Body top cover — rounded shell
  const cover = rbox(bodyW - 10, 20, bodyLen - 10, 8, MAT.darkPolycarbonate, 'Body Cover');
  cover.position.y = bodyY + 18;
  bodyGroup.add(cover);

  // Thin accent stripe along the top spine
  const stripe = rbox(14, 4, bodyLen - 30, 2, MAT.cyan);
  stripe.position.y = bodyY + 30;
  bodyGroup.add(stripe);

  // Side fairings — thin rounded panels on each flank
  const fairingL = rbox(6, 22, bodyLen - 40, 3, MAT.aluminium);
  fairingL.position.set(bodyW / 2 + 1, bodyY + 4, 0);
  bodyGroup.add(fairingL);

  const fairingR = rbox(6, 22, bodyLen - 40, 3, MAT.aluminium);
  fairingR.position.set(-bodyW / 2 - 1, bodyY + 4, 0);
  bodyGroup.add(fairingR);

  // Sensor head — rounded housing with camera lens + sensor dot
  const sensorHead = rbox(bodyW - 20, 24, 30, 6, MAT.darkPolycarbonate, 'Sensor Head');
  sensorHead.position.set(0, bodyY + 20, bodyLen / 2 - 5);
  bodyGroup.add(sensorHead);

  // Chrome camera lens (faces forward, +Z)
  const lens = cyl(8, 8, 6, 20, MAT.chrome, 'Camera Lens');
  lens.rotation.x = Math.PI / 2;
  lens.position.set(0, bodyY + 20, bodyLen / 2 + 9);
  bodyGroup.add(lens);

  // Small cyan sensor dot
  const sensorDot = sphere(3, 10, MAT.cyan);
  sensorDot.position.set(bodyW / 2 - 22, bodyY + 26, bodyLen / 2 + 9);
  bodyGroup.add(sensorDot);

  // LiDAR puck — chamfered cylinder with a vent ring
  const lidar = chamferCyl(12, 14, 3, 20, MAT.titanium, 'LiDAR');
  lidar.position.set(0, bodyY + 40, bodyLen / 2 - 5);
  bodyGroup.add(lidar);

  const lidarVents = vents(20, 6, MAT.blackAnodised);
  lidarVents.position.set(0, bodyY + 40, bodyLen / 2 - 5 + 12);
  bodyGroup.add(lidarVents);

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
    bodyGroup.add(legGroup);

    // Hip yaw joint
    const hipYaw = new THREE.Group();
    hipYaw.rotation.y = joints[ji];
    legGroup.add(hipYaw);

    const hipDisk = chamferCyl(14, 20, 3, 16, MAT.chrome, `${prefix} Hip`);
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

    // Capsule femur — beefy load-bearing upper leg; length keeps centre at -femur/2
    const femurLink = capsule(12, femur - 24, MAT.darkPolycarbonate, `${prefix} Femur`);
    femurLink.position.y = -femur / 2;
    hipPitch.add(femurLink);

    // Knee joint
    const kneeGroup = new THREE.Group();
    kneeGroup.position.y = -femur;
    kneeGroup.rotation.x = joints[ji + 2];
    hipPitch.add(kneeGroup);

    const kneeDisk = chamferCyl(12, 18, 3, 16, MAT.chrome, `${prefix} Knee`);
    kneeDisk.rotation.z = Math.PI / 2;
    kneeGroup.add(kneeDisk);

    // Capsule tibia — load-bearing lower leg, slightly slimmer than femur; centre stays at -tibia/2
    const tibiaLink = capsule(10, tibia - 20, MAT.aluminium, `${prefix} Tibia`);
    tibiaLink.position.y = -tibia / 2;
    kneeGroup.add(tibiaLink);

    // Foot — rubber, slightly flattened; CENTRE position unchanged for IK stance
    const foot = sphere(9, 12, MAT.rubber, `${prefix} Foot`);
    foot.scale.y = 0.7;
    foot.position.y = -tibia - 2;
    kneeGroup.add(foot);
  }

  return root;
}

// ─────────────────────────────────────────────────────────────
// ROBOT 4 — MARS ROVER (ROCKER-BOGIE)
// ─────────────────────────────────────────────────────────────
// Sample-arm link lengths — shared by the mesh, telemetry FK, and arm IK
export const ROVER_ARM_L1 = 70;
export const ROVER_ARM_L2 = 60;

export function buildRover(joints, params) {
  const { chassisL = 200, chassisW = 140, wheelR = 40, wheelW = 20 } = params;
  const root = new THREE.Group();
  const bodyY = wheelR + 30;

  // Main chassis frame — boxy but bevelled titanium tub (Perseverance look)
  const chassis = rbox(chassisW, 22, chassisL, 8, MAT.titanium, 'Chassis Frame');
  chassis.position.y = bodyY;
  root.add(chassis);

  // Science deck / body — aluminium shell on top
  const deck = rbox(chassisW - 10, 16, chassisL - 30, 8, MAT.aluminium, 'Science Deck');
  deck.position.y = bodyY + 19;
  root.add(deck);

  // Equipment-bay inset — dark polycarbonate panel recessed into the deck top
  const bay = rbox(chassisW - 34, 4, chassisL - 70, 3, MAT.darkPolycarbonate);
  bay.position.y = bodyY + 28;
  root.add(bay);

  // RTG — rear nuclear battery: dark finned block elevated off the −Z end
  const rtg = new THREE.Group();
  rtg.position.set(0, bodyY + 36, -chassisL / 2 - 6);
  root.add(rtg);
  const rtgCore = chamferCyl(20, 70, 4, 16, MAT.steelDark, 'RTG Core');
  rtgCore.rotation.x = Math.PI / 2;       // axis along Z, sticking out the back
  rtg.add(rtgCore);
  for (let f = 0; f < 6; f++) {
    const fin = cyl(30, 30, 2, 16, MAT.steelDark);
    fin.rotation.x = Math.PI / 2;
    fin.position.z = -28 + f * 11;
    rtg.add(fin);
  }

  // Solar panels — kept for structure, cleaner dark-blue panel material
  const solarL = box(chassisW + 50, 4, chassisL - 60, MAT.darkPolycarbonate, 'Solar Panel L');
  solarL.position.set(-(chassisW + 50) / 2, bodyY + 30, 0);
  root.add(solarL);

  const solarR = box(chassisW + 50, 4, chassisL - 60, MAT.darkPolycarbonate, 'Solar Panel R');
  solarR.position.set((chassisW + 50) / 2, bodyY + 30, 0);
  root.add(solarR);

  // Antenna masts — thin cyl + sphere tip, on the deck
  for (const ax of [-chassisW / 2 + 18, chassisW / 2 - 18]) {
    const ant = cyl(2, 2, 60, 8, MAT.aluminium);
    ant.position.set(ax, bodyY + 30 + 30, chassisL / 2 - 30);
    root.add(ant);
    const tip = sphere(4, 10, MAT.cyan);
    tip.position.set(ax, bodyY + 30 + 60, chassisL / 2 - 30);
    root.add(tip);
  }

  // Camera mast
  const mast = cyl(5, 5, 80, 8, MAT.aluminium, 'Camera Mast');
  mast.position.set(0, bodyY + 40 + 40, -chassisL / 2 + 20);
  root.add(mast);

  const camHead = rbox(30, 20, 20, 4, MAT.blackAnodised, 'Camera Head');
  camHead.position.set(0, bodyY + 90, -chassisL / 2 + 20);
  root.add(camHead);

  // Mast camera lens (chrome, faces forward +Z) + cyan sensor
  const lens = cyl(6, 6, 8, 16, MAT.chrome);
  lens.rotation.x = Math.PI / 2;
  lens.position.set(0, bodyY + 90, -chassisL / 2 + 32);
  root.add(lens);
  const sensor = box(6, 6, 2, MAT.cyan);
  sensor.position.set(11, bodyY + 90, -chassisL / 2 + 30);
  root.add(sensor);

  // Sample arm — 2-link (shoulder pitch + elbow) reaching in the rover's
  // forward vertical plane. RotX(q) lifts the forward (−Z) link up by q.
  const armMount = new THREE.Group();
  armMount.name = 'ArmMount';
  armMount.position.set(chassisW / 2 - 10, bodyY + 28, -chassisL / 2 + 30);
  root.add(armMount);

  const mountPost = cyl(8, 10, 24, 12, MAT.aluminium, 'Arm Mount Post');
  mountPost.position.y = -12;
  armMount.add(mountPost);

  const shoulderGroup = new THREE.Group();
  shoulderGroup.rotation.x = joints[5];
  armMount.add(shoulderGroup);

  const shoulderJoint = cyl(7, 7, 18, 14, MAT.chrome, 'Arm Shoulder Joint');
  shoulderJoint.rotation.z = Math.PI / 2;
  shoulderGroup.add(shoulderJoint);

  const upperLink = rbox(10, 10, ROVER_ARM_L1, 3, MAT.aluminium, 'Arm Upper Link');
  upperLink.position.z = -ROVER_ARM_L1 / 2;
  shoulderGroup.add(upperLink);

  const elbowGroup = new THREE.Group();
  elbowGroup.position.z = -ROVER_ARM_L1;
  elbowGroup.rotation.x = joints[6];
  shoulderGroup.add(elbowGroup);

  const elbowJoint = cyl(6, 6, 16, 14, MAT.chrome, 'Arm Elbow Joint');
  elbowJoint.rotation.z = Math.PI / 2;
  elbowGroup.add(elbowJoint);

  const foreLink = rbox(8, 8, ROVER_ARM_L2, 2.5, MAT.aluminium, 'Arm Forearm Link');
  foreLink.position.z = -ROVER_ARM_L2 / 2;
  elbowGroup.add(foreLink);

  const scoop = rbox(16, 8, 14, 3, MAT.steelDark, 'Arm Scoop');
  scoop.position.z = -ROVER_ARM_L2 - 6;
  elbowGroup.add(scoop);

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

  // joints: [0]=wheel spin angle, [1]=FL steer, [2]=FR steer, [3]=RL steer, [4]=RR steer
  const steerByIndex = [joints[1], joints[2], 0, 0, joints[3], joints[4]];

  for (let i = 0; i < 6; i++) {
    const [sx, , wz] = wheelPositions[i];
    const wg = new THREE.Group();
    wg.position.set(sx * (chassisW / 2 + wheelW / 2), wheelR, wz);
    wg.rotation.y = steerByIndex[i]; // steer about vertical axis
    wg.name = `WheelGroup${i}`;

    // spin rolls the whole wheel assembly about the axle (local X after steer)
    const spinGroup = new THREE.Group();
    spinGroup.rotation.x = joints[0];
    wg.add(spinGroup);

    const tyre = cyl(wheelR, wheelR, wheelW, 24, MAT.rubber, `${wheelNames[i]} Tyre`);
    tyre.rotation.z = Math.PI / 2;
    spinGroup.add(tyre);

    const hub = cyl(wheelR * 0.45, wheelR * 0.45, wheelW + 4, 16, MAT.aluminium, `${wheelNames[i]} Hub`);
    hub.rotation.z = Math.PI / 2;
    spinGroup.add(hub);

    for (let s = 0; s < 5; s++) {
      const spoke = box(wheelR * 0.75, 3, 3, MAT.darkSteel);
      spoke.rotation.z = (s / 5) * Math.PI;
      spinGroup.add(spoke);
    }

    // Grouser cleats — raised ridges around the tyre circumference (spin with wheel)
    for (let c = 0; c < 8; c++) {
      const a = (c / 8) * Math.PI * 2;
      const cleat = box(wheelW + 2, 4, wheelR * 0.28, MAT.steelDark);
      cleat.position.set(0, Math.sin(a) * wheelR, Math.cos(a) * wheelR);
      cleat.rotation.x = -a;
      spinGroup.add(cleat);
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

  // ── PEDESTAL — solid cast-aluminium column on a bolted steel base flange
  const baseFlange = chamferCyl(48, 22, 5, 28, MAT.steelDark, 'Pedestal Base');
  baseFlange.position.y = 11;
  root.add(baseFlange);

  const baseBolts = boltCircle(40, 8, MAT.steelDark);
  baseBolts.position.y = 22;
  root.add(baseBolts);

  const pedestal = chamferCyl(32, pedestalH - 22, 5, 28, MAT.aluminium, 'Pedestal Column');
  pedestal.position.y = 22 + (pedestalH - 22) / 2;
  root.add(pedestal);

  // Pedestal cap
  const cap = chamferCyl(42, 22, 4, 28, MAT.blackAnodised, 'Pedestal Cap');
  cap.position.y = pedestalH + 9;
  root.add(cap);

  // Status LED on the pedestal cap
  const led = box(14, 4, 6, MAT.green, 'Status LED');
  led.position.set(0, pedestalH + 14, 38);
  root.add(led);

  // ── INNER ARM
  const inner = new THREE.Group();
  inner.position.y = pedestalH + 20;
  inner.rotation.y = joints[0];
  root.add(inner);

  // Cast-alloy casing spanning the inner link, with a parting-line seam
  const innerLink = rbox(l1 + 24, 40, 30, 7, MAT.aluminium, 'Inner Arm Link');
  innerLink.position.x = l1 / 2;
  inner.add(innerLink);

  const innerSeam = box(l1 + 26, 1.5, 4, MAT.darkPolycarbonate);
  innerSeam.position.set(l1 / 2, 0, 15.2);
  inner.add(innerSeam);

  // Shoulder motor
  const shoulderMot = chamferCyl(27, 44, 4, 24, MAT.blackAnodised, 'Shoulder Motor');
  inner.add(shoulderMot);

  // Elbow motor
  const elbowMot = chamferCyl(21, 40, 4, 24, MAT.steelDark, 'Elbow Motor');
  elbowMot.position.x = l1;
  inner.add(elbowMot);

  // ── OUTER ARM
  const outer = new THREE.Group();
  outer.position.x = l1;
  outer.rotation.y = joints[1];
  inner.add(outer);

  const outerLink = rbox(l2 + 20, 34, 26, 7, MAT.aluminium, 'Outer Arm Link');
  outerLink.position.x = l2 / 2;
  outer.add(outerLink);

  const outerSeam = box(l2 + 22, 1.5, 4, MAT.darkPolycarbonate);
  outerSeam.position.set(l2 / 2, 0, 13.2);
  outer.add(outerSeam);

  // ── DECORATIVE CABLE — rubber loop from pedestal cap to the elbow joint
  const cableCurve = new THREE.CatmullRomCurve3([
    new THREE.Vector3(0, pedestalH + 24, 30),
    new THREE.Vector3(l1 * 0.35, pedestalH + 60, 36),
    new THREE.Vector3(l1 * 0.75, pedestalH + 50, 30),
    new THREE.Vector3(l1, pedestalH + 28, 18),
  ]);
  const cable = mesh(new THREE.TubeGeometry(cableCurve, 24, 3, 8, false), MAT.rubber);
  root.add(cable);

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

  const eeHead = chamferCyl(17, 22, 4, 24, MAT.steelDark, 'End Effector Head');
  eeGroup.add(eeHead);

  // Tool chuck detail under the flange
  const chuck = chamferCyl(10, 10, 3, 20, MAT.blackAnodised, 'Tool Chuck');
  chuck.position.y = -15;
  eeGroup.add(chuck);

  // Vacuum nozzle
  const nozzle = cyl(4, 8, 24, 12, MAT.chrome, 'Vacuum Nozzle');
  nozzle.position.y = -28;
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

// Telemetry FK — right palm world position from the slider joints
export function dexArmFK(joints, params) {
  const [x, y, z] = dexArmChainFK([joints[0], joints[1], joints[2]], params, 1);
  return { x, y, z };
}

/**
 * World position of one palm centre. q = [shoulderPitch, shoulderYaw, elbowFlex].
 * Models the wrist-neutral pose: wrist joints (3-4) pivot the hand about the
 * wrist ball and are not part of the position IK chain (kinematics.ikJoints).
 */
export function dexArmChainFK(q, params, side) {
  const torsoH = 180, torsoW = 80;
  const mount = matTrans(side * (torsoW / 2 + 18), 150 + torsoH * 0.82, 0);
  // shoulderGroup: rotation.z = q0*side, rotation.y = q1*side (euler XYZ => RX·RY·RZ, RX=0)
  const shoulder = matMul(matRotY(q[1] * side), matRotZ(q[0] * side));
  const tPose = matRotZ(side * Math.PI / 2);
  const upper = matTrans(0, -params.upperArmLen, 0);
  const elbow = matRotZ(-q[2] * side);
  const fore = matTrans(0, -8 - params.forearmLen, 0);
  const palm = matTrans(0, -20 - params.palmLen / 2, 0);
  let T = matMul(mount, shoulder);
  T = matMul(T, tPose);
  T = matMul(T, upper);
  T = matMul(T, elbow);
  T = matMul(T, fore);
  T = matMul(T, palm);
  return matPoint(T, [0, 0, 0]);
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
    joints: [0, Math.PI / 6, -Math.PI / 3, -Math.PI / 6, 0, 0, 18],
    jointNames: [
      'Base (J1)', 'Shoulder (J2)', 'Elbow (J3)',
      'Wrist 1 Pitch (J4)', 'Wrist 2 Roll (J5)', 'Wrist 3 Pitch (J6)',
      'Gripper Gap',
    ],
    jointLimits: [
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: -360, max: 360, step: 1, isAngle: true },
      { min: 8,    max: 45,  step: 1, isAngle: false },
    ],
    params: { l1: 81, l2: 213, l3: 196, l4: 67 },
    paramDefs: [
      { label: 'Base Height (d1)', key: 'l1', min: 50,  max: 150, step: 1, unit: 'mm' },
      { label: 'Upper Arm (a2)',   key: 'l2', min: 100, max: 280, step: 1, unit: 'mm' },
      { label: 'Forearm (a3)',     key: 'l3', min: 80,  max: 260, step: 1, unit: 'mm' },
      { label: 'Wrist (d5)',       key: 'l4', min: 40,  max: 120, step: 1, unit: 'mm' },
    ],
    ikSupported: true,
    kinematics: {
      type: 'dh',
      dof: 6,
      rows: armDHRows,
      // UR5e anchor (half scale): d1=162.5, a2=425, a3=392.2, d4=133.3,
      // d5=99.7, d6=99.6 mm full scale; ±360° joints; 180°/s all joints.
      anchor: 'Universal Robots UR5e (half scale)',
      speeds: [180, 180, 180, 180, 180, 180], // deg/s, UR5e max joint speed
      facts: 'UR5e: 5 kg payload · 850 mm reach · ±360° joints',
    },
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
      { min: -120, max: 120, step: 1, isAngle: true },
      { min: -130, max: 0,   step: 1, isAngle: true },
      { min: -180, max: 90,  step: 1, isAngle: true },
      { min: -120, max: 120, step: 1, isAngle: true },
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
    kinematics: {
      type: 'limbs',
      arm: (p) => ({ upper: 82, fore: 82, shoulderY: 120, shoulderX: p.hipSpacing / 2 + 30 }),
      leg: (p) => ({ thigh: p.thigh, shin: p.shin, hipY: -15, hipX: p.hipSpacing / 2 }),
      rootY: (p) => p.thigh + p.shin + p.footH + 15,
      // mass fractions for ground-projected COM (coarse anthropomorphic split)
      masses: { torso: 0.50, head: 0.07, arm: 0.05, thigh: 0.10, shin: 0.06, foot: 0.015 },
      anchor: 'Optimus-class proportions',
    },
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
    kinematics: {
      type: 'quad-legs',
      // legs: name, hip offset from body centre (local), side sign for coxa
      legs: (p) => {
        const H = p.femur + p.tibia + 10;
        return [
          { name: 'FR', joint0: 0, hip: [ p.bodyW / 2, H,  p.bodyLen / 2 - 20], side:  1 },
          { name: 'FL', joint0: 3, hip: [-p.bodyW / 2, H,  p.bodyLen / 2 - 20], side: -1 },
          { name: 'BL', joint0: 6, hip: [-p.bodyW / 2, H, -p.bodyLen / 2 + 20], side: -1 },
          { name: 'BR', joint0: 9, hip: [ p.bodyW / 2, H, -p.bodyLen / 2 + 20], side:  1 },
        ];
      },
      footOffset: 2, // foot sphere centre sits 2 below tibia end — add to tibia in IK calls
      anchor: 'Boston Dynamics Spot-class leg geometry',
    },
  },

  rover: {
    name: 'Mars Rover',
    builder: buildRover,
    // telemetry: sample-arm scoop position (world, Y up)
    fk: (joints, p) => {
      const f = ROVER_ARM_L1 * Math.cos(joints[5]) + ROVER_ARM_L2 * Math.cos(joints[5] + joints[6]);
      const h = ROVER_ARM_L1 * Math.sin(joints[5]) + ROVER_ARM_L2 * Math.sin(joints[5] + joints[6]);
      return {
        x: p.chassisW / 2 - 10,
        y: p.wheelR + 30 + 28 + h,
        z: -p.chassisL / 2 + 30 - f,
      };
    },
    joints: [0, 0, 0, 0, 0, 0.6, -1.2],
    jointNames: ['Wheel Spin', 'FL Steer', 'FR Steer', 'RL Steer', 'RR Steer', 'Arm Shoulder', 'Arm Elbow'],
    jointLimits: [
      { min: -360, max: 360, step: 5, isAngle: true },
      { min: -50,  max: 50,  step: 1, isAngle: true },
      { min: -50,  max: 50,  step: 1, isAngle: true },
      { min: -50,  max: 50,  step: 1, isAngle: true },
      { min: -50,  max: 50,  step: 1, isAngle: true },
      { min: -30,  max: 100, step: 1, isAngle: true },
      { min: -150, max: 10,  step: 1, isAngle: true },
    ],
    params: { chassisL: 200, chassisW: 140, wheelR: 40, wheelW: 20 },
    paramDefs: [
      { label: 'Chassis Length', key: 'chassisL', min: 150, max: 300, step: 10, unit: 'mm' },
      { label: 'Chassis Width',  key: 'chassisW', min: 100, max: 200, step: 10, unit: 'mm' },
      { label: 'Wheel Radius',   key: 'wheelR',   min: 25,  max: 65,  step: 5,  unit: 'mm' },
      { label: 'Wheel Width',    key: 'wheelW',   min: 12,  max: 35,  step: 2,  unit: 'mm' },
    ],
    ikSupported: false,
    kinematics: {
      type: 'ackermann',
      // corner-wheel geometry from chassis params
      geometry: (p) => ({ wheelbase: p.chassisL - 50, track: p.chassisW + p.wheelW }),
      anchor: 'NASA Perseverance — 4 corner-wheel steering',
    },
  },

  scara: {
    name: 'SCARA Arm',
    builder: buildSCARA,
    fk: (joints, params) => {
      const p = scaraFK(joints[0], joints[1], params.l1, params.l2);
      return { x: p.x, y: params.pedestalH - joints[2], z: p.z };
    },
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
    ikSupported: true,
    kinematics: {
      type: 'scara',
      // DH for expert panel + FK; q = [q1, q2, zSlide(mm), q4]
      rows: (q, p) => [
        [q[0], p.pedestalH, p.l1, 0],
        [q[1], 0,           p.l2, 0],
        [0,   -q[2],        0,    0],   // prismatic Z slide
        [q[3], 0,           0,    0],
      ],
      prismatic: [2],
      zTravel: 130,
      anchor: 'Epson LS6-B class (600 mm reach family, scaled)',
    },
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
    kinematics: {
      type: 'mixer',
      anchor: 'X-quad convention (FR/BL CCW, FL/BR CW)',
    },
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
    ikSupported: true,
    kinematics: {
      type: 'numeric-arms',
      fkFn: dexArmChainFK,
      ikJoints: [0, 1, 2], // shoulder pitch, shoulder yaw, elbow drive position IK
      anchor: 'Shadow Hand finger ranges; bimanual torso layout',
    },
  },
};
