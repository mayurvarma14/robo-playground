# Visual Overhaul — Pro Rendering + Geometry Polish

**Date:** 2026-06-13
**Status:** Approved
**Cycle:** 2 of the "toy → serious learning platform" upgrade. Follows the Real Kinematics Core cycle.

## Goal

Make all 7 robots look professional-grade without external assets: a studio rendering
overhaul (environment reflections, filmic tone mapping, soft shadows, reflective floor)
plus a geometry polish pass on every robot. Kinematics, IK, parametric sliders, STL
export, and the zero-build philosophy are untouched.

## Why rendering first

The toy look is mostly a lighting problem, not a geometry problem: PBR metals without an
environment map have nothing to reflect and read as flat plastic, and the current
6-light rig over-fills, flattening all form. Fixing the pipeline upgrades every robot at
once; geometry polish then compounds it.

## Part A — Rendering pipeline (`js/viewport.js`, `js/materials.js`)

| Change | Detail |
|---|---|
| Environment | `RoomEnvironment` (three addon, procedural — no files) through `PMREMGenerator`, assigned to `scene.environment`. Gives all metals/clearcoats real reflections. |
| Tone mapping | `renderer.toneMapping = ACESFilmicToneMapping`, exposure tuned ~1.0–1.2; `outputColorSpace = SRGBColorSpace`. |
| Shadows | Keep PCF soft shadow map; tighten the shadow camera frustum to the robot bounds for crisper contact; reduce shadow acne via bias tuning. |
| Floor | Replace the matte disc with a large floor plane: dark, low-roughness `MeshStandardMaterial` with `envMapIntensity` tuned for a subtle studio-floor reflection; keep the grid overlay. A radial-gradient canvas texture darkens the contact area under the robot. |
| Light rig | Env map supplies ambient/fill: drop the 6-light rig to key (directional, shadow-casting) + warm rim + low hemisphere. Remove ambient/fill/under-fill or reduce to near zero. |
| Materials | Retune every entry in `materials.js` against the new environment (roughness/metalness pairs per real-world reference). Body panels move to `MeshPhysicalMaterial` with `clearcoat` (automotive panel look). Brushed metals get a slight procedural roughness variation via a small canvas texture. Theme switch keeps working (env intensity adjusts per theme). |

Wireframe toggle, gizmo, triads, trace, and `_isHelper` guards continue to work; helper
materials are unlit (`MeshBasicMaterial`/lines) and unaffected by tone mapping concerns.

## Part B — Geometry polish (`js/robots.js`)

Rules: same `THREE.Group` skeletons, same joint group transforms, same mesh names used
by export, same params and joint indices. Only the visual shells change. All kinematics
configs and tests stay valid.

**Shared helpers** (top of robots.js, used by all builders):
- `rbox` — `RoundedBoxGeometry` (three addon) wrapper matching the `box` helper signature
- `capsule` — `CapsuleGeometry` limb segments
- `chamferCyl` — cylinder with chamfered edge profile via `LatheGeometry`
- `boltCircle(radius, n)` — ring of small bolt-head cylinders for joint faces
- `vents(w, n)` — recessed vent-slot strip

**Per robot (visual deltas only):**
- **Arm** — rounded joint housings with bolt circles and UR-style accent rings (blue caps,
  light-grey links), cable conduit tube running shoulder→wrist, gripper with chamfered
  fingers and rubber pads.
- **Humanoid** — capsule upper arms/forearms/thighs/shins, rounded torso shell with
  chest vents, smooth head with glossy visor (clearcoat), rounded shoulder caps.
- **Quadruped** — rounded body shell with side fairings, capsule legs, rubber
  hemispherical feet, sensor head with camera lens ring; lidar puck with vents.
- **Rover** — rounded chassis with greebles (RTG finned block at rear, two antennas,
  camera lens detail on mast head), wheel cleat ridges, gold-foil-toned body accents.
- **SCARA** — rounded arm casings with seam lines, pedestal base bezel with bolt circle,
  status LED strip, cable loop from base to elbow.
- **Drone** — rounded center body with camera gimbal sphere, motor bell cylinders with
  cooling fins, aerofoil-profile props (extruded shape), thin landing gear struts.
- **Bimanual** — rounded torso column with vents, capsule arms, chamfered palm blocks,
  rounded finger segments.

## Out of scope

External mesh/texture assets (GLB, HDR, image files), physics, post-processing passes
(SSAO/bloom — would add render-target complexity), URDF.

## Testing & verification

- All 39 `?test=1` tests stay green (no math touched).
- Per-robot screenshot review in the browser at standard camera angles.
- Regression: sliders rebuild, STL export (named parts list intact), sequencer, IK
  gizmo/sticks, theme toggle (light + dark), wireframe toggle, expert panel.
- Performance sanity: rebuild-per-frame paths (drone/rover drive) stay smooth — rounded
  geometry adds vertices, so segment counts stay modest (radius segments ≤ 4 for
  RoundedBox, ≤ 24 radial for cylinders).
