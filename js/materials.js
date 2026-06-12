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
