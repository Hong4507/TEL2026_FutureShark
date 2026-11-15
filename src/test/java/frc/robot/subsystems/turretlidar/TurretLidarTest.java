package frc.robot.subsystems.turretlidar;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

  /** 光束打在最近那一列洞中心 ±(0.20 − 0.05) m 內才判斷穿洞（距離不同會穿過不同列）。 */
  @Test
  void validRangeAroundNearestRow() {
    assertTrue(TurretLidar.inValidRange(1.80));
    assertTrue(TurretLidar.inValidRange(1.94));
    assertTrue(TurretLidar.inValidRange(1.66));
    assertTrue(TurretLidar.inValidRange(1.20)); // 低列
    assertFalse(TurretLidar.inValidRange(1.50)); // 低列和中列中間的板子
    assertFalse(TurretLidar.inValidRange(1.96));
    assertFalse(TurretLidar.inValidRange(Double.NaN)); // 沒有牆距
  }

  /** 11/15：光束仰角 23°（和鉛直線 67°），鏡頭高 0.48029 m。 */
  @Test
  void idealDistancesAt23Degrees() {
    assertEquals(23.0, TurretLidar.kMountPitchDeg, 1e-12);
    assertEquals(1.696, TurretLidar.idealHorizontalMeters(0), 0.002);
    assertEquals(3.109, TurretLidar.idealHorizontalMeters(1), 0.002);
    assertEquals(4.523, TurretLidar.idealHorizontalMeters(2), 0.002);
    // 在理想距離時光束剛好打在列中心
    for (int row = 0; row < 3; row++) {
      assertEquals(TurretLidar.kRowHeights[row],
          TurretLidar.beamHeightAtBoard(TurretLidar.idealHorizontalMeters(row)), 1e-9);
    }
  }

  /** Elastic 顯示和最近那一列的距離差：正 = 太遠、負 = 太近。 */
  @Test
  void beamTextShowsSignedDistanceError() {
    double ideal = TurretLidar.idealHorizontalMeters(1);
    assertEquals(0.42, TurretLidar.distanceErrorMeters(ideal + 0.42), 1e-9);
    assertEquals(-0.30, TurretLidar.distanceErrorMeters(ideal - 0.30), 1e-9);
    assertEquals("中列 太遠 +0.42m", TurretLidar.beamText(Beam.OUT_OF_RANGE, ideal + 0.42));
    assertEquals("中列 太近 -0.30m", TurretLidar.beamText(Beam.OUT_OF_RANGE, ideal - 0.30));
    assertEquals("中列 打在板上 +0.05m", TurretLidar.beamText(Beam.BOARD, ideal + 0.05));
    assertEquals("中列 穿過洞 -0.02m", TurretLidar.beamText(Beam.THROUGH_HOLE, ideal - 0.02));
    assertEquals("沒有牆距", TurretLidar.beamText(Beam.OUT_OF_RANGE, Double.NaN));
    assertTrue(TurretLidar.beamText(Beam.OUT_OF_RANGE, TurretLidar.idealHorizontalMeters(0) - 0.4).startsWith("低列 太近"));
  }
}
