package frc.robot.subsystems.turretlidar;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.turretlidar.TurretLidar.Beam;

class TurretLidarTest {
  private static final double EXPECTED = 2.5;

  @Test
  void disconnectedIsNoData() {
    assertEquals(Beam.NO_DATA, TurretLidar.classify(false, 2.5, 1000, EXPECTED));
    assertEquals(Beam.NO_DATA, TurretLidar.classify(true, Double.NaN, 0, EXPECTED));
  }

  @Test
  void zeroReadingMeansThroughHole() {
    // TF-Luna 前方沒東西／太遠時回傳 0
    assertEquals(Beam.THROUGH_HOLE, TurretLidar.classify(true, 0.0, 0, EXPECTED));
    assertEquals(Beam.THROUGH_HOLE, TurretLidar.classify(true, 2.5, 20, EXPECTED)); // 訊號太弱
  }

  @Test
  void fartherThanBoardMeansThroughHole() {
    assertEquals(Beam.THROUGH_HOLE, TurretLidar.classify(true, 4.0, 800, EXPECTED));
  }

  @Test
  void nearBoardDistanceIsBoard() {
    assertEquals(Beam.BOARD, TurretLidar.classify(true, 2.45, 800, EXPECTED));
    assertEquals(Beam.BOARD, TurretLidar.classify(true, 2.75, 800, EXPECTED));
  }

  @Test
  void muchCloserIsBlocked() {
    assertEquals(Beam.BLOCKED, TurretLidar.classify(true, 1.0, 800, EXPECTED));
  }

  @Test
  void rowAtHeight() {
    assertEquals("LOW", TurretLidar.rowAtHeight(1.1));
    assertEquals("MID", TurretLidar.rowAtHeight(1.95));
    assertEquals("NONE", TurretLidar.rowAtHeight(0.52));
    assertEquals("NONE", TurretLidar.rowAtHeight(1.5)); // 兩列之間的板子
  }
}
