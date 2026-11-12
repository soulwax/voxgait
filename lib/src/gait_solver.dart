import 'dart:math' as math;

import 'gait_cycle.dart';
import 'math.dart';

/// Terrain height + surface normal returned by a [GaitTerrainSampler].
final class GaitTerrainSample {
  final double groundY;
  final GaitVec3 normal;

  const GaitTerrainSample({
    required this.groundY,
    this.normal = GaitVec3.up,
  });

  static const flatZero = GaitTerrainSample(groundY: 0.0, normal: GaitVec3.up);
}

typedef GaitTerrainSampler = GaitTerrainSample Function(
  double worldX,
  double worldZ,
);

/// Solved per-leg world foot IK target, arc lift, and surface normal alignment.
final class GaitFootTarget {
  final String legId;
  final GaitVec3 worldPosition;
  final GaitVec3 restAnchorWorld;
  final GaitVec3 plantedPosition;
  final GaitVec3 swingGoalPosition;
  final GaitVec3 groundNormal;
  final double phase;
  final bool isGrounded;
  final bool isStepping;
  final double swingProgress;
  final double stepArcLift;

  const GaitFootTarget({
    required this.legId,
    required this.worldPosition,
    required this.restAnchorWorld,
    required this.plantedPosition,
    required this.swingGoalPosition,
    required this.groundNormal,
    required this.phase,
    required this.isGrounded,
    required this.isStepping,
    required this.swingProgress,
    required this.stepArcLift,
  });
}

/// Complete frame output from [ProceduralGaitSolver.step].
final class GaitPoseSnapshot {
  final List<GaitFootTarget> feet;
  final double globalPhase;
  final double horizontalSpeed;
  final double activeFrequencyHz;
  final double pelvisVerticalOffset;
  final double bodyPitchRad;
  final double bodyRollRad;
  final double averageGroundY;
  final GaitVec3 groundSlopeNormal;

  const GaitPoseSnapshot({
    required this.feet,
    required this.globalPhase,
    required this.horizontalSpeed,
    required this.activeFrequencyHz,
    required this.pelvisVerticalOffset,
    required this.bodyPitchRad,
    required this.bodyRollRad,
    required this.averageGroundY,
    required this.groundSlopeNormal,
  });

  int get groundedCount => feet.where((f) => f.isGrounded).length;
  int get steppingCount => feet.where((f) => f.isStepping).length;

  GaitFootTarget? footById(String id) {
    for (final f in feet) {
      if (f.legId == id) return f;
    }
    return null;
  }
}

final class _LegRuntimeState {
  final GaitLegDef def;
  GaitVec3 plantedWorld;
  GaitVec3 swingStartWorld;
  GaitVec3 swingGoalWorld;
  GaitVec3 lastNormal;
  double phase;
  bool stepping = false;
  double localSwingTimer = 0.0;
  double localSwingDuration = 0.25;

  _LegRuntimeState({
    required this.def,
    required this.plantedWorld,
    required this.lastNormal,
    required this.phase,
  })  : swingStartWorld = plantedWorld,
        swingGoalWorld = plantedWorld;
}

/// Procedural foot-stepping IK solver supporting velocity-lead prediction,
/// parabolic step arcs, minimum support gating, slope adaptation & pelvis bobbing.
final class ProceduralGaitSolver {
  GaitCycle cycle;
  double pelvisBobAmplitude;
  double velocityLeadTime;
  double slopeAdaptationRate;
  double stepHeightScale;

  final Map<String, _LegRuntimeState> _legs = {};
  double _globalPhase = 0.0;
  double _smoothedPitchRad = 0.0;
  double _smoothedRollRad = 0.0;
  double _smoothedPelvisBob = 0.0;
  bool _initialized = false;
  GaitPoseSnapshot? _lastSnapshot;

  ProceduralGaitSolver({
    GaitCycle? cycle,
    this.pelvisBobAmplitude = 0.055,
    this.velocityLeadTime = 0.18,
    this.slopeAdaptationRate = 10.0,
    this.stepHeightScale = 1.0,
  }) : cycle = cycle ?? GaitCycle.biped();

  GaitPoseSnapshot? get lastSnapshot => _lastSnapshot;
  double get globalPhase => _globalPhase;

  void setCycle(GaitCycle nextCycle) {
    cycle = nextCycle;
    reset();
  }

  void reset() {
    _legs.clear();
    _globalPhase = 0.0;
    _smoothedPitchRad = 0.0;
    _smoothedRollRad = 0.0;
    _smoothedPelvisBob = 0.0;
    _initialized = false;
    _lastSnapshot = null;
  }

  static double _smoothstep(double t) {
    final u = t.clamp(0.0, 1.0).toDouble();
    return u * u * (3.0 - 2.0 * u);
  }

  static GaitVec3 _restWorld(
    GaitVec3 bodyPos,
    GaitVec3 forward,
    GaitVec3 right,
    GaitVec3 localOffset,
  ) {
    return bodyPos +
        right * localOffset.x +
        GaitVec3.up * localOffset.y +
        forward * localOffset.z;
  }

  GaitVec3 _computeLeadTarget({
    required GaitVec3 restWorld,
    required GaitVec3 horizontalVel,
    required double maxStrideRadius,
    required GaitTerrainSampler sampler,
  }) {
    var lead = horizontalVel * velocityLeadTime;
    final len = lead.length;
    if (len > maxStrideRadius && len > 1e-6) {
      lead = lead * (maxStrideRadius / len);
    }
    final cand = restWorld + lead;
    final hit = sampler(cand.x, cand.z);
    return GaitVec3(cand.x, hit.groundY, cand.z);
  }

  void _ensureInitialized({
    required GaitVec3 bodyWorldPos,
    required GaitVec3 bodyForward,
    required GaitVec3 bodyRight,
    required GaitTerrainSampler sampler,
  }) {
    if (_initialized && _legs.length == cycle.legs.length) return;
    _legs.clear();
    for (final leg in cycle.legs) {
      final rest = _restWorld(bodyWorldPos, bodyForward, bodyRight, leg.restOffsetLocal);
      final hit = sampler(rest.x, rest.z);
      _legs[leg.id] = _LegRuntimeState(
        def: leg,
        plantedWorld: GaitVec3(rest.x, hit.groundY, rest.z),
        lastNormal: hit.normal.normalized,
        phase: GaitCycle.wrapPhase(leg.phaseOffset),
      );
    }
    _initialized = true;
  }

  /// Advances gait phases and resolves all world foot IK targets, slope tilt & pelvis bobbing.
  GaitPoseSnapshot step({
    required GaitVec3 bodyWorldPos,
    required GaitVec3 bodyForward,
    required GaitVec3 bodyRight,
    required GaitVec3 velocity,
    required double dt,
    GaitTerrainSampler? sampleTerrain,
  }) {
    final sampler = sampleTerrain ?? ((_, __) => GaitTerrainSample.flatZero);

    // Orthonormalize horizontal basis vectors
    final fwdFlat = GaitVec3(bodyForward.x, 0.0, bodyForward.z).normalized;
    final fwd = fwdFlat.lengthSquared > 1e-6 ? fwdFlat : GaitVec3.forward;
    final rightFlat = GaitVec3(bodyRight.x, 0.0, bodyRight.z).normalized;
    final right = rightFlat.lengthSquared > 1e-6
        ? rightFlat
        : GaitVec3.up.cross(fwd).normalized;

    _ensureInitialized(
      bodyWorldPos: bodyWorldPos,
      bodyForward: fwd,
      bodyRight: right,
      sampler: sampler,
    );

    final horizVel = GaitVec3(velocity.x, 0.0, velocity.z);
    final horizSpeed = horizVel.length;
    final freq = cycle.effectiveFrequencyHz(horizSpeed);
    final moving = freq > 0.0;

    if (moving && dt > 0.0) {
      _globalPhase = GaitCycle.wrapPhase(_globalPhase + dt * freq);
    }

    var currentGroundedCount =
        _legs.values.where((l) => !l.stepping).length;

    // 1. Complete swings or initiate new swings under minimum support gating
    for (final leg in cycle.legs) {
      final st = _legs[leg.id]!;
      final rest = _restWorld(bodyWorldPos, fwd, right, leg.restOffsetLocal);
      final restSample = sampler(rest.x, rest.z);
      final restGround = GaitVec3(rest.x, restSample.groundY, rest.z);
      final leadTarget = _computeLeadTarget(
        restWorld: restGround,
        horizontalVel: horizVel,
        maxStrideRadius: leg.maxStrideRadius,
        sampler: sampler,
      );

      final cyclePhase = GaitCycle.wrapPhase(_globalPhase + leg.phaseOffset);
      st.phase = cyclePhase;

      if (st.stepping) {
        st.swingGoalWorld = leadTarget;
        if (moving) {
          if (cycle.isStancePhase(cyclePhase)) {
            st.stepping = false;
            st.plantedWorld = leadTarget;
            st.lastNormal = sampler(leadTarget.x, leadTarget.z).normal.normalized;
            currentGroundedCount++;
          }
        } else {
          st.localSwingTimer += math.max(0.0, dt);
          if (st.localSwingTimer >= st.localSwingDuration) {
            st.stepping = false;
            st.plantedWorld = leadTarget;
            st.lastNormal = sampler(leadTarget.x, leadTarget.z).normal.normalized;
            currentGroundedCount++;
          }
        }
      } else {
        // Keep planted foot anchored onto live terrain height under it
        final plantedHit = sampler(st.plantedWorld.x, st.plantedWorld.z);
        st.plantedWorld = GaitVec3(
          st.plantedWorld.x,
          plantedHit.groundY,
          st.plantedWorld.z,
        );
        st.lastNormal = plantedHit.normal.normalized;

        final wantsCycleSwing = moving && !cycle.isStancePhase(cyclePhase);
        final driftDist = st.plantedWorld.distanceTo(restGround);
        final wantsReplantSwing =
            !moving && driftDist > cycle.replantDistanceThreshold;

        final canLift = currentGroundedCount - 1 >= cycle.minSimultaneousSupport;
        if ((wantsCycleSwing || wantsReplantSwing) && canLift) {
          st.stepping = true;
          st.swingStartWorld = st.plantedWorld;
          st.swingGoalWorld = leadTarget;
          st.localSwingTimer = 0.0;
          st.localSwingDuration = 1.0 / (math.max(1.0, cycle.baseStepFrequencyHz) * 2.5);
          currentGroundedCount--;
        }
      }
    }

    // 2. Evaluate final world positions, step arc lift & slope normal estimates
    final feet = <GaitFootTarget>[];
    var groundYSum = 0.0;
    var normalSum = GaitVec3.zero;
    var forwardWeightedY = 0.0;
    var forwardWeight = 0.0;
    var rightWeightedY = 0.0;
    var rightWeight = 0.0;

    for (final leg in cycle.legs) {
      final st = _legs[leg.id]!;
      final rest = _restWorld(bodyWorldPos, fwd, right, leg.restOffsetLocal);
      final restHit = sampler(rest.x, rest.z);
      final restGround = GaitVec3(rest.x, restHit.groundY, rest.z);

      double progress = 0.0;
      double lift = 0.0;
      GaitVec3 worldPos = st.plantedWorld;

      if (st.stepping) {
        progress = moving
            ? cycle.swingProgress(st.phase)
            : (st.localSwingTimer / math.max(1e-4, st.localSwingDuration))
                .clamp(0.0, 1.0)
                .toDouble();
        final ease = _smoothstep(progress);
        final basePos = GaitVec3.lerp(st.swingStartWorld, st.swingGoalWorld, ease);
        lift = 4.0 * progress * (1.0 - progress) * leg.stepHeight * stepHeightScale;
        worldPos = GaitVec3(basePos.x, basePos.y + lift, basePos.z);
      }

      final groundHit = sampler(worldPos.x, worldPos.z);
      final gNorm = groundHit.normal.lengthSquared > 1e-6
          ? groundHit.normal.normalized
          : GaitVec3.up;

      groundYSum += groundHit.groundY;
      normalSum = normalSum + gNorm;

      final rel = rest - bodyWorldPos;
      final fwdProj = rel.dot(fwd);
      final rightProj = rel.dot(right);
      forwardWeightedY += groundHit.groundY * fwdProj;
      forwardWeight += fwdProj * fwdProj;
      rightWeightedY += groundHit.groundY * rightProj;
      rightWeight += rightProj * rightProj;

      feet.add(
        GaitFootTarget(
          legId: leg.id,
          worldPosition: worldPos,
          restAnchorWorld: restGround,
          plantedPosition: st.plantedWorld,
          swingGoalPosition: st.swingGoalWorld,
          groundNormal: gNorm,
          phase: st.phase,
          isGrounded: !st.stepping,
          isStepping: st.stepping,
          swingProgress: progress,
          stepArcLift: lift,
        ),
      );
    }

    final footCount = math.max(1, feet.length);
    final avgGroundY = groundYSum / footCount;
    final avgSlopeNormal = normalSum.lengthSquared > 1e-6
        ? normalSum.normalized
        : GaitVec3.up;

    // Derive target body pitch (forward slope) & roll (lateral slope) from ground gradient
    final slopeFwdComp = avgSlopeNormal.dot(fwd);
    final slopeRightComp = avgSlopeNormal.dot(right);
    final slopeUpComp = math.max(0.1, avgSlopeNormal.dot(GaitVec3.up));

    // Fall back to foot height differential if plane normal was close to flat while samples vary
    var targetPitch = -math.atan2(slopeFwdComp, slopeUpComp);
    if (targetPitch.abs() < 1e-4 && forwardWeight > 1e-4) {
      targetPitch = -math.atan(forwardWeightedY / forwardWeight);
    }
    var targetRoll = math.atan2(slopeRightComp, slopeUpComp);
    if (targetRoll.abs() < 1e-4 && rightWeight > 1e-4) {
      targetRoll = math.atan(rightWeightedY / rightWeight);
    }

    final alpha = dt <= 0.0
        ? 1.0
        : (1.0 - math.exp(-slopeAdaptationRate * dt)).clamp(0.0, 1.0).toDouble();
    _smoothedPitchRad += (targetPitch - _smoothedPitchRad) * alpha;
    _smoothedRollRad += (targetRoll - _smoothedRollRad) * alpha;

    // 2x stride harmonic vertical pelvis bob while moving
    final bobTarget = moving
        ? -pelvisBobAmplitude *
            (0.5 - 0.5 * math.cos(4.0 * math.pi * _globalPhase))
        : 0.0;
    _smoothedPelvisBob += (bobTarget - _smoothedPelvisBob) * alpha;

    final snapshot = GaitPoseSnapshot(
      feet: List<GaitFootTarget>.unmodifiable(feet),
      globalPhase: _globalPhase,
      horizontalSpeed: horizSpeed,
      activeFrequencyHz: freq,
      pelvisVerticalOffset: _smoothedPelvisBob,
      bodyPitchRad: _smoothedPitchRad,
      bodyRollRad: _smoothedRollRad,
      averageGroundY: avgGroundY,
      groundSlopeNormal: avgSlopeNormal,
    );
    _lastSnapshot = snapshot;
    return snapshot;
  }
}
