package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.ShotTable.Row;
import frc.robot.subsystems.turret.ShotTable.Source;

class ShotTableTest {
  private static final double H = FieldConstants.kMidTargetHeight;

  @Test
  void emptyTableUsesOldFormula() {
    Row row = new Row("MID", H, new double[][] {});
    for (double d = 2.0; d <= 4.0; d += 0.25) {
      double old = Math.max(TurretConstants.kMinAngleRad, Math.min(TurretConstants.kMaxAngleRad,
          Math.PI / 2 - Math.atan2(H - TurretConstants.kTurretToRobotCenter.getZ() + d * d * 0.019358, d)));
      ShotTable.Result r = ShotTable.compute(row, H, d);
      assertEquals(Source.FORMULA, r.source());
      assertEquals(old, r.pitchRad(), 1e-12);
    }
  }

  @Test
  void interpolatesInsideTable() {
    Row row = new Row("MID", H, new double[][] {{3.0, 1.0}, {2.0, 0.8}});
    ShotTable.Result r = ShotTable.compute(row, H, 2.5);
    assertEquals(Source.TABLE, r.source());
    assertEquals(0.9, r.pitchRad(), 1e-9);
    assertEquals(0.8, ShotTable.compute(row, H, 2.0).pitchRad(), 1e-9);
    assertEquals(1.0, ShotTable.compute(row, H, 3.0).pitchRad(), 1e-9);
  }

  @Test
  void extrapolationIsContinuousAtEdges() {
    Row row = new Row("MID", H, new double[][] {{2.0, 0.8}, {3.0, 1.0}});
    double inside = ShotTable.compute(row, H, 3.0).pitchRad();
    ShotTable.Result outside = ShotTable.compute(row, H, 3.0001);
    assertEquals(Source.EXTRAPOLATED, outside.source());
    assertEquals(inside, outside.pitchRad(), 1e-3);
    // 超出範圍時變化量 = 公式的變化量
    double formulaDelta = ShotTable.ballisticPitchRad(H, 3.3) - ShotTable.ballisticPitchRad(H, 3.0);
    assertEquals(1.0 + formulaDelta, ShotTable.compute(row, H, 3.3).pitchRad(), 1e-9);
  }

  @Test
  void resultIsClampedToTurretRange() {
    Row row = new Row("MID", H, new double[][] {{2.0, 0.1}, {3.0, 5.0}});
    assertEquals(TurretConstants.kMinAngleRad, ShotTable.compute(row, H, 2.0).pitchRad(), 1e-12);
    assertEquals(TurretConstants.kMaxAngleRad, ShotTable.compute(row, H, 3.0).pitchRad(), 1e-12);
  }
}
