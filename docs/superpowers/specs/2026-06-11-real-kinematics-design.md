# Real Kinematics Core — Design Spec

**Date:** 2026-06-11
**Status:** Approved
**Cycle:** 1 of the "toy → serious learning platform" upgrade. Later cycles (out of scope here): physics, URDF, trajectory planning, teach pendant, scripting, sensors.

## Goal

Replace the playground's ad-hoc joint control and 2-link IK with industry-correct kinematics across all 7 robots, presented so learners can see and understand the underlying math. Models, limits, and dimensions are anchored to published specs of real robots so what users learn transfers to physical hardware.

## Architecture

```
js/kinematics.js   NEW — core engine (pure math, no DOM, no rendering):
  ├─ DHChain        DH parameter table → FK via Matrix4 composition,
  │                 numeric Jacobian, damped-least-squares (DLS) IK,
  │                 joint limit clamping, manipulability index
  ├─ AnalyticalIK   closed-form solvers: SCARA (2R planar + Z + roll),
  │                 3-DOF leg (hip ab/ad + hip pitch + knee), 2-link planar
  ├─ RoverKin       Ackermann steering (per-wheel steer angles from turn radius)
  └─ QuadMixer      X-quad motor mixing (thrust/roll/pitch/yaw → 4 motor outputs)

js/robots.js       each robot definition gains a `kinematics` config block:
                   DH table or limb definitions, joint limits (rad), max joint
                   velocities, dimensions from published specs

js/main.js         IK panel rewrite: gizmo wiring, expert-mode toggle,
                   pose readout, per-robot control mapping

js/viewport.js     TransformControls target gizmo, per-joint RGB frame triads
                   (toggleable), end-effector trace line

index.html / css   expert panel markup: DH table, EE pose + 4×4 matrix,
                   manipulability bar, per-robot extras
```

**Data flow:** slider / gizmo input → kinematics solve → joint angles → existing robot `THREE.Group` rotations. No mesh rebuild on motion; math drives the meshes already there. `kinematics.js` stays renderer-agnostic except for using Three.js `Matrix4`/`Vector3` as the math types.

**Why custom code, not a library:** JS IK libraries are stale and CCD-based (not how industrial robots solve), and a readable DLS solver is itself teaching material. Keeps the zero-build, zero-dependency philosophy.

## Per-robot kinematics

| Robot | Method | Real-world anchor |
|---|---|---|
| 6-DOF arm | DH FK + DLS Jacobian IK, full pose (XYZ + RPY), manipulability/singularity warning | UR5e published DH parameters, joint limits, joint speeds |
| SCARA | Analytical IK (2R planar + Z + roll), elbow left/right configuration toggle | Epson LS6-B dimensions, annular workspace |
| Bimanual | Per-arm DLS IK on shared chain; fingers remain FK sliders | Shadow Hand finger ranges |
| Humanoid | Per-limb analytical 3-DOF IK (arm reach targets, leg foot placement), ground-projected COM marker that turns red when outside the foot support polygon | Optimus-class proportions |
| Quadruped | Per-leg 3-DOF analytical IK; body-pose control — drag body XYZ + RPY while feet stay planted | Spot-class leg segment lengths and joint ranges |
| Rover | Ackermann steering with per-wheel angles and turn-radius display, flat-ground drive, 2-link arm IK | Perseverance wheelbase / steer geometry |
| Drone | Motor mixer: thrust/roll/pitch/yaw inputs → 4 motor outputs with attitude tilt response (kinematic only, no full dynamics) | Standard X-quad mixing convention |

All real-world parameter values (DH tables, limits, speeds, dimensions) are verified against manufacturer/published sources during implementation, not invented.

## IK interaction

- **Target gizmo:** Three.js `TransformControls` on the IK target — translate arrows + rotate rings, switchable. Numeric XYZ + RPY fields synced both ways.
- **Quadruped:** gizmo attaches to the body (body-pose mode).
- **Drone:** virtual stick pads (thrust/yaw, pitch/roll) instead of gizmo.
- **Unreachable target:** solver returns best-effort pose + `unreachable` flag; gizmo tints red — matches real teach-pendant behavior.

## UI modes

**Simple mode (default):** current UI plus IK gizmo and a frame-triad toggle. No math visible.

**Expert mode (header toggle):** reveals a panel with:
- Live DH parameter table (θ, d, a, α per joint; θ updates as joints move)
- End-effector pose: position (mm), RPY (deg), quaternion
- Live 4×4 EE transform matrix (monospace)
- Manipulability index with color bar — red near singularities
- Per-robot extras: quadruped per-leg foot positions; drone mixer matrix + per-motor %

## Error handling

- Unreachable IK target → best-effort pose + red gizmo, no snap-back.
- Joint limit reached during IK → clamp + limit badge on the affected slider.
- Near singularity → warning indicator; DLS damping guarantees no NaN/exploding poses.

## Testing

Kinematics is pure math — testable without tooling. `js/tests.js`, loaded only with `?test=1` URL param, results to console:
- FK round-trip: known DH configurations vs expected poses
- IK→FK consistency: solve a target, FK of the solution within 0.1 mm
- Analytical vs numerical agreement for SCARA
- Joint limit clamping behavior
- Mixer matrix sanity (pure thrust → equal motors, pure roll → differential pairs)

Manual verification: load each robot, drag gizmo through workspace, confirm smooth tracking, red-flag behavior at boundaries, expert panel values plausible against published specs.

## Out of scope (future cycles)

Physics simulation, URDF import/export, trajectory planning (MoveJ/MoveL, velocity profiles), teach-pendant jog modes, robot scripting, sensor simulation, learning curriculum.
