import 'dart:math' as math;

/// Lightweight value-type 3D vector for pure-Dart gait kinematics.
final class GaitVec3 {
  final double x;
  final double y;
  final double z;

  const GaitVec3(this.x, this.y, this.z);

  static const zero = GaitVec3(0.0, 0.0, 0.0);
  static const up = GaitVec3(0.0, 1.0, 0.0);
  static const forward = GaitVec3(0.0, 0.0, 1.0);
  static const right = GaitVec3(1.0, 0.0, 0.0);

  GaitVec3 operator +(GaitVec3 o) => GaitVec3(x + o.x, y + o.y, z + o.z);
  GaitVec3 operator -(GaitVec3 o) => GaitVec3(x - o.x, y - o.y, z - o.z);
  GaitVec3 operator *(double s) => GaitVec3(x * s, y * s, z * s);

  double dot(GaitVec3 o) => x * o.x + y * o.y + z * o.z;

  GaitVec3 cross(GaitVec3 o) => GaitVec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x,
      );

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  GaitVec3 get normalized {
    final l = length;
    return l < 1e-9 ? zero : GaitVec3(x / l, y / l, z / l);
  }

  double distanceTo(GaitVec3 o) => (this - o).length;

  static GaitVec3 lerp(GaitVec3 a, GaitVec3 b, double t) {
    final u = t.clamp(0.0, 1.0).toDouble();
    return GaitVec3(
      a.x + (b.x - a.x) * u,
      a.y + (b.y - a.y) * u,
      a.z + (b.z - a.z) * u,
    );
  }
}
