/**
 * materials.js
 * Shared PBR material library for all robot parts.
 * Materials are created once and reused across all robots.
 */
import * as THREE from 'three';

const M = new THREE.MeshStandardMaterial;

function mat(color, roughness, metalness, extra = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness, ...extra });
}

export const MAT = {
  // ── Structural frames
  darkSteel:     mat(0x263040, 0.55, 0.75),   // dark blue-steel alloy
  aluminium:     mat(0xb8c4d0, 0.35, 0.80),   // brushed aluminium
  blackAnodised: mat(0x18222e, 0.40, 0.70),   // black anodised aluminium
  titanium:      mat(0x8090a0, 0.30, 0.85),   // raw titanium

  // ── Body panels / covers
  whitePolycarbonate: mat(0xecf0f5, 0.50, 0.10), // Tesla Optimus body panels
  darkPolycarbonate:  mat(0x1a2030, 0.45, 0.15), // dark shell panels
  carbonFiber:        mat(0x1c1c24, 0.60, 0.30), // carbon fibre weave look

  // ── Actuators / joints
  chrome:   mat(0xd0dce8, 0.10, 0.95),  // polished chrome joints
  brass:    mat(0xc8902a, 0.20, 0.85),  // brass gears
  copper:   mat(0xb84820, 0.25, 0.90),  // copper winding details
  steelDark: mat(0x3a4455, 0.40, 0.80), // dark steel fasteners

  // ── Wheels / tyres
  rubber: mat(0x141820, 0.90, 0.02),    // rubber tyre

  // ── Accent / glow
  cyan:   mat(0x00b4d8, 0.30, 0.30, { emissive: 0x007090, emissiveIntensity: 0.6 }),
  orange: mat(0xf05a28, 0.40, 0.20, { emissive: 0x601808, emissiveIntensity: 0.3 }),
  green:  mat(0x22c55e, 0.40, 0.15, { emissive: 0x0a5028, emissiveIntensity: 0.4 }),

  // ── Environment
  floor:    mat(0x1e2a3a, 0.80, 0.10),
  platform: mat(0x263040, 0.75, 0.15),
};

// Make sure they all cast/receive shadows
for (const m of Object.values(MAT)) {
  m.side = THREE.FrontSide;
}
