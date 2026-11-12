import 'dart:math' as math;

import 'math.dart';

/// Standard multi-legged locomotion coordination presets.
enum GaitPreset {
  biped,
  quadrupedTrot,
  quadrupedWalk,
  hexapodTripod,
}

/// Definition of an articulated leg attached to a character chassis.
final class GaitLegDef {
  final String id;

  /// Rest anchor offset relative to body origin `(right, heightOffset, forward)`.
  final GaitVec3 restOffsetLocal;

  /// Normalized phase offset `[0, 1)` within the global gait cycle.
  final double phaseOffset;

  /// Maximum parabolic swing clearance height in meters.
  final double stepHeight;

  /// Maximum stride reach radius around the rest anchor.
  final double maxStrideRadius;

  /// Optional paired leg identifier that steps simultaneously (e.g., diagonal trot pair).
  final String? syncGroupId;

  const GaitLegDef({
    required this.id,
    required this.restOffsetLocal,
    required this.phaseOffset,
    this.stepHeight = 0.28,
    this.maxStrideRadius = 0.85,
    this.syncGroupId,
  });
}

/// Phase oscillator, duty-factor curve & leg coordination specification.
final class GaitCycle {
  final GaitPreset preset;
  final List<GaitLegDef> legs;

  /// Fraction `[0, 1)` of a stride cycle during which a foot remains planted on ground.
  double stanceDutyFactor;

  /// Baseline stride cycle frequency (Hz) when starting to walk.
  double baseStepFrequencyHz;

  /// Additional stride frequency gained per m/s of horizontal movement speed.
  double speedFrequencyScale;

  /// Horizontal speed threshold (m/s) below which the character is considered idle.
  double moveSpeedThreshold;

  /// World-space drift distance (m) triggering an adaptive replant step while idle/turning.
  double replantDistanceThreshold;

  /// Minimum number of grounded feet enforced simultaneously for static/dynamic stability.
  int minSimultaneousSupport;

  GaitCycle({
    required this.preset,
    required List<GaitLegDef> legs,
    this.stanceDutyFactor = 0.60,
    this.baseStepFrequencyHz = 1.4,
    this.speedFrequencyScale = 0.35,
    this.moveSpeedThreshold = 0.12,
    this.replantDistanceThreshold = 0.35,
    int? minSimultaneousSupport,
  })  : legs = List<GaitLegDef>.unmodifiable(legs),
        minSimultaneousSupport =
            minSimultaneousSupport ?? math.max(1, legs.length ~/ 2);

  factory GaitCycle.biped({
    double stanceWidth = 0.34,
    double stepHeight = 0.26,
    double stanceDutyFactor = 0.60,
    double baseStepFrequencyHz = 1.45,
  }) {
    final halfW = stanceWidth * 0.5;
    return GaitCycle(
      preset: GaitPreset.biped,
      stanceDutyFactor: stanceDutyFactor,
      baseStepFrequencyHz: baseStepFrequencyHz,
      minSimultaneousSupport: 1,
      legs: [
        GaitLegDef(
          id: 'left',
          restOffsetLocal: GaitVec3(-halfW, 0.0, 0.0),
          phaseOffset: 0.0,
          stepHeight: stepHeight,
          syncGroupId: 'A',
        ),
        GaitLegDef(
          id: 'right',
          restOffsetLocal: GaitVec3(halfW, 0.0, 0.0),
          phaseOffset: 0.5,
          stepHeight: stepHeight,
          syncGroupId: 'B',
        ),
      ],
    );
  }

  factory GaitCycle.quadrupedTrot({
    double halfWidth = 0.28,
    double halfLength = 0.52,
    double stepHeight = 0.24,
    double stanceDutyFactor = 0.58,
    double baseStepFrequencyHz = 1.7,
  }) {
    return GaitCycle(
      preset: GaitPreset.quadrupedTrot,
      stanceDutyFactor: stanceDutyFactor,
      baseStepFrequencyHz: baseStepFrequencyHz,
      minSimultaneousSupport: 2,
      legs: [
        GaitLegDef(
          id: 'front_left',
          restOffsetLocal: GaitVec3(-halfWidth, 0.0, halfLength),
          phaseOffset: 0.0,
          stepHeight: stepHeight,
          syncGroupId: 'diag_a',
        ),
        GaitLegDef(
          id: 'rear_right',
          restOffsetLocal: GaitVec3(halfWidth, 0.0, -halfLength),
          phaseOffset: 0.0,
          stepHeight: stepHeight,
          syncGroupId: 'diag_a',
        ),
        GaitLegDef(
          id: 'front_right',
          restOffsetLocal: GaitVec3(halfWidth, 0.0, halfLength),
          phaseOffset: 0.5,
          stepHeight: stepHeight,
          syncGroupId: 'diag_b',
        ),
        GaitLegDef(
          id: 'rear_left',
          restOffsetLocal: GaitVec3(-halfWidth, 0.0, -halfLength),
          phaseOffset: 0.5,
          stepHeight: stepHeight,
          syncGroupId: 'diag_b',
        ),
      ],
    );
  }

  factory GaitCycle.quadrupedWalk({
    double halfWidth = 0.28,
    double halfLength = 0.52,
    double stepHeight = 0.20,
    double stanceDutyFactor = 0.74,
    double baseStepFrequencyHz = 1.25,
  }) {
    return GaitCycle(
      preset: GaitPreset.quadrupedWalk,
      stanceDutyFactor: stanceDutyFactor,
      baseStepFrequencyHz: baseStepFrequencyHz,
      minSimultaneousSupport: 3,
      legs: [
        GaitLegDef(
          id: 'rear_left',
          restOffsetLocal: GaitVec3(-halfWidth, 0.0, -halfLength),
          phaseOffset: 0.0,
          stepHeight: stepHeight,
        ),
        GaitLegDef(
          id: 'front_left',
          restOffsetLocal: GaitVec3(-halfWidth, 0.0, halfLength),
          phaseOffset: 0.25,
          stepHeight: stepHeight,
        ),
        GaitLegDef(
          id: 'rear_right',
          restOffsetLocal: GaitVec3(halfWidth, 0.0, -halfLength),
          phaseOffset: 0.50,
          stepHeight: stepHeight,
        ),
        GaitLegDef(
          id: 'front_right',
          restOffsetLocal: GaitVec3(halfWidth, 0.0, halfLength),
          phaseOffset: 0.75,
          stepHeight: stepHeight,
        ),
      ],
    );
  }

  factory GaitCycle.hexapodTripod({
    double width = 0.52,
    double strideLength = 0.55,
    double stepHeight = 0.20,
    double stanceDutyFactor = 0.56,
    double baseStepFrequencyHz = 2.0,
  }) {
    return GaitCycle(
      preset: GaitPreset.hexapodTripod,
      stanceDutyFactor: stanceDutyFactor,
      baseStepFrequencyHz: baseStepFrequencyHz,
      minSimultaneousSupport: 3,
      legs: [
        GaitLegDef(
          id: 'front_left',
          restOffsetLocal: GaitVec3(-width, 0.0, strideLength),
          phaseOffset: 0.0,
          stepHeight: stepHeight,
          syncGroupId: 'tripod_a',
        ),
        GaitLegDef(
          id: 'mid_right',
          restOffsetLocal: GaitVec3(width * 1.15, 0.0, 0.0),
          phaseOffset: 0.0,
          stepHeight: stepHeight,
          syncGroupId: 'tripod_a',
        ),
        GaitLegDef(
          id: 'rear_left',
          restOffsetLocal: GaitVec3(-width, 0.0, -strideLength),
          phaseOffset: 0.0,
          stepHeight: stepHeight,
          syncGroupId: 'tripod_a',
        ),
        GaitLegDef(
          id: 'front_right',
          restOffsetLocal: GaitVec3(width, 0.0, strideLength),
          phaseOffset: 0.5,
          stepHeight: stepHeight,
          syncGroupId: 'tripod_b',
        ),
        GaitLegDef(
          id: 'mid_left',
          restOffsetLocal: GaitVec3(-width * 1.15, 0.0, 0.0),
          phaseOffset: 0.5,
          stepHeight: stepHeight,
          syncGroupId: 'tripod_b',
        ),
        GaitLegDef(
          id: 'rear_right',
          restOffsetLocal: GaitVec3(width, 0.0, -strideLength),
          phaseOffset: 0.5,
          stepHeight: stepHeight,
          syncGroupId: 'tripod_b',
        ),
      ],
    );
  }

  double effectiveFrequencyHz(double horizontalSpeed) {
    if (horizontalSpeed < moveSpeedThreshold) return 0.0;
    return baseStepFrequencyHz + horizontalSpeed * speedFrequencyScale;
  }

  static double wrapPhase(double phase) {
    final p = phase % 1.0;
    return p < 0.0 ? p + 1.0 : p;
  }

  bool isStancePhase(double legPhase) =>
      wrapPhase(legPhase) < stanceDutyFactor.clamp(0.1, 0.95);

  /// Normalized `[0, 1]` progress through swing when `!isStancePhase(legPhase)`.
  double swingProgress(double legPhase) {
    final duty = stanceDutyFactor.clamp(0.1, 0.95);
    final p = wrapPhase(legPhase);
    if (p < duty) return 0.0;
    return ((p - duty) / (1.0 - duty)).clamp(0.0, 1.0).toDouble();
  }
}
