<p align="center">
  <img src="assets/logo.svg" width="88" alt="voxgait logo"/>
</p>

# voxgait

Pure Dart procedural foot-stepping locomotion & gait solver for biped, quadruped, and hexapod characters with terrain slope adaptation and pelvis bobbing.

## Features

- **`GaitCycle` & `GaitLegDef`**: Presets for `biped`, `quadrupedTrot`, `quadrupedWalk`, and `hexapodTripod` with speed-scaled stride frequency and stance duty factor.
- **`ProceduralGaitSolver`**: Phase-driven & distance-triggered stepping, minimum simultaneous support / diagonal stability gating, velocity-lead foot placement, parabolic step arcs, terrain slope pitch/roll adaptation, and pelvis vertical bobbing.
