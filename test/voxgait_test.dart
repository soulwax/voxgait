import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:voxgait/voxgait.dart';

void main() {
  test('GaitCycle presets enforce phase offsets, duty factor & speed-scaled frequency', () {
    final biped = GaitCycle.biped();
    expect(biped.legs.length, 2);
    expect(biped.effectiveFrequencyHz(0.0), 0.0);
    expect(biped.effectiveFrequencyHz(3.0), greaterThan(biped.baseStepFrequencyHz));
    expect(biped.isStancePhase(0.2), isTrue);
    expect(biped.isStancePhase(0.75), isFalse);
    expect(biped.swingProgress(0.8), closeTo(0.5, 1e-6));

    final trot = GaitCycle.quadrupedTrot();
    expect(trot.legs.length, 4);
    expect(trot.legs[0].phaseOffset, 0.0);
    expect(trot.legs[1].phaseOffset, 0.0);
    expect(trot.legs[2].phaseOffset, 0.5);
    expect(trot.legs[3].phaseOffset, 0.5);

    final hex = GaitCycle.hexapodTripod();
    expect(hex.legs.length, 6);
    expect(hex.minSimultaneousSupport, 3);
  });

  test('ProceduralGaitSolver gates biped support, lifts parabolic step arcs, and bobs pelvis', () {
    final solver = ProceduralGaitSolver(
      cycle: GaitCycle.biped(stepHeight: 0.30, stanceDutyFactor: 0.60),
      pelvisBobAmplitude: 0.06,
      velocityLeadTime: 0.20,
    );

    var bodyX = 0.0;
    var bodyZ = 0.0;
    const vel = GaitVec3(0.0, 0.0, 2.5);
    var maxLiftObserved = 0.0;
    var sawNegativeBob = false;

    for (var i = 0; i < 60; i++) {
      const dt = 0.025;
      bodyZ += vel.z * dt;
      final pose = solver.step(
        bodyWorldPos: GaitVec3(bodyX, 0.0, bodyZ),
        bodyForward: GaitVec3.forward,
        bodyRight: GaitVec3.right,
        velocity: vel,
        dt: dt,
      );

      // Minimum support guarantee: at least 1 foot always grounded on biped
      expect(pose.groundedCount, greaterThanOrEqualTo(1));
      for (final f in pose.feet) {
        if (f.stepArcLift > maxLiftObserved) maxLiftObserved = f.stepArcLift;
      }
      if (pose.pelvisVerticalOffset < -0.01) {
        sawNegativeBob = true;
      }
    }

    expect(maxLiftObserved, greaterThan(0.18));
    expect(sawNegativeBob, isTrue);
  });

  test('ProceduralGaitSolver adapts body pitch and foot elevation to sloped terrain', () {
    final solver = ProceduralGaitSolver(
      cycle: GaitCycle.quadrupedTrot(),
      slopeAdaptationRate: 30.0,
    );

    // 20-degree upward ramp along +Z
    final slopeAngle = 20.0 * math.pi / 180.0;
    final tanSlope = math.tan(slopeAngle);
    final normalY = math.cos(slopeAngle);
    final normalZ = -math.sin(slopeAngle);

    GaitTerrainSample rampSampler(double x, double z) => GaitTerrainSample(
          groundY: z * tanSlope,
          normal: GaitVec3(0.0, normalY, normalZ),
        );

    GaitPoseSnapshot? lastPose;
    var z = 0.0;
    for (var i = 0; i < 40; i++) {
      const dt = 0.03;
      z += 1.5 * dt;
      lastPose = solver.step(
        bodyWorldPos: GaitVec3(0.0, z * tanSlope, z),
        bodyForward: GaitVec3.forward,
        bodyRight: GaitVec3.right,
        velocity: GaitVec3(0.0, 1.5 * tanSlope, 1.5),
        dt: dt,
        sampleTerrain: rampSampler,
      );
    }

    expect(lastPose, isNotNull);
    final frontFoot = lastPose!.footById('front_left')!;
    final rearFoot = lastPose.footById('rear_left')!;
    expect(frontFoot.worldPosition.y, greaterThan(rearFoot.worldPosition.y));
    expect(lastPose.bodyPitchRad.abs(), greaterThan(0.25));
  });
}
