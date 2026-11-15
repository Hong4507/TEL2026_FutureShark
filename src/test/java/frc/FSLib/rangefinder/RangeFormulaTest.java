package frc.FSLib.rangefinder;

import static frc.FSLib.rangefinder.RangeSensorConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import frc.FSLib.rangefinder.DualTFLunaRangeSensor.Solution;
import frc.FSLib.rangefinder.RangeSensorConstants.RangeReference;

/** Robot-Brief §8.1（GRID_WALL 新公式）／§8.2（LOW_WALL = 舊公式）。 */
class RangeFormulaTest {
  private static final double EPS = 1e-5;

  @Test
  void constantsMatchMeasuredGeometry() {
    assertEquals(0.07042, kRightLunaInboardMeters, 1e-12);
    assertEquals(0.4826, kLeftLunaInboardMeters, 1e-12);
    assertEquals(0.17072, kBaselineMeters, 1e-12);
  }

  /** 測試表第 1 列：擺正時左 Luna 本來就多 0.41218 m → yaw = 0。 */
  @Test
  void squareToWall() {
    Solution s = grid(2.41218, 2.0);
    assertTrue(s.valid());
    assertEquals(0.0, s.yawRad(), 1e-9);
    assertEquals(1.92958, s.wallDistance(), EPS);
    assertEquals(1.92958 + kShooterInboardMeters, s.targetDistance(), EPS);
    if (kShooterInboardMeters == 0.25) assertEquals(2.17958, s.targetDistance(), EPS);
  }

  /** 測試表第 2 列：左多 2 cm → ±6.68°，牆距 = 平均 × cos。 */
  @Test
  void tilted() {
    Solution s = grid(2.43218, 2.0);
    double expectedYaw = Math.atan2(0.02, 0.17072) * kYawSign;
    assertEquals(Math.toRadians(6.68) * kYawSign, s.yawRad(), Math.toRadians(0.01));
    assertEquals(expectedYaw, s.yawRad(), 1e-9);
    assertEquals(1.93958 * Math.cos(expectedYaw), s.wallDistance(), EPS);
    assertEquals(1.92642, s.wallDistance(), 1e-4);
    assertEquals(1.92642 + kShooterInboardMeters, s.targetDistance(), 1e-4);
    if (kShooterInboardMeters == 0.25) assertEquals(2.17642, s.targetDistance(), 1e-4);
  }

  /** 測試表第 3 列：兩邊都無效 → 預設 3.0、沒有角度、沒有牆距。 */
  @Test
  void bothInvalidFallsBack() {
    Solution s = DualTFLunaRangeSensor.solve(RangeReference.GRID_WALL, 0, 0, false, false);
    assertFalse(s.valid());
    assertEquals(3.0, s.targetDistance(), 1e-12);
    assertEquals(kFallbackDistanceMeters, s.targetDistance(), 1e-12);
    assertTrue(Double.isNaN(s.yawRad()));
    assertTrue(Double.isNaN(s.wallDistance()));
  }

  /** 測試表第 4 列：只有右邊有效 → 不算角度，用單邊距離。 */
  @Test
  void onlyRightValid() {
    Solution s = DualTFLunaRangeSensor.solve(RangeReference.GRID_WALL, 0, 2.0, false, true);
    assertTrue(s.valid());
    assertTrue(Double.isNaN(s.yawRad()));
    assertEquals(2.0 - kRightLunaInboardMeters, s.wallDistance(), 1e-9);
    assertEquals(2.0 - kRightLunaInboardMeters + kShooterInboardMeters, s.targetDistance(), 1e-9);
  }

  @Test
  void onlyLeftValid() {
    Solution s = DualTFLunaRangeSensor.solve(RangeReference.GRID_WALL, 2.41218, 0, true, false);
    assertTrue(Double.isNaN(s.yawRad()));
    assertEquals(1.92958 - kLunaTrimMeters, s.wallDistance(), EPS);
  }

  /** getYaw 用的角度：π/2 = 正對。 */
  @Test
  void yawRadIsCenteredOnHalfPi() {
    assertEquals(Math.PI / 2, DualTFLunaRangeSensor.yawRad(RangeReference.GRID_WALL, 2.41218, 2.0), 1e-9);
  }

  // ===== LOW_WALL：和 11/14 版本的公式完全一樣 =====

  /** 11/14 版本 DualTFLunaRangeSensor.getTargetDistance()（原封不動）。 */
  private static double oldDistance(double left, double right, boolean lValid, boolean rValid) {
    double baseline = 0.30, wallToTarget = 2.5, offset = 0.04;
    if (lValid && rValid) {
      double tilt = Math.atan2(right - left, baseline);
      double perpendicular = (left + right) / 2 * Math.cos(tilt);
      return perpendicular + wallToTarget / Math.cos(tilt) + offset;
    } else if (lValid) {
      return left + wallToTarget + offset;
    } else if (rValid) {
      return right + wallToTarget + offset;
    }
    return 3.0;
  }

  /** 11/14 版本 yawRad（kYawSign = +1）。 */
  private static double oldYaw(double left, double right) {
    return Math.PI / 2 + 1.0 * Math.atan2(right - left, 0.30);
  }

  @Test
  void lowWallMatchesOldFormulaExactly() {
    for (double l = 0.3; l <= 1.5; l += 0.07) {
      for (double r = 0.3; r <= 1.5; r += 0.11) {
        for (int mask = 0; mask < 4; mask++) {
          boolean lv = (mask & 1) != 0, rv = (mask & 2) != 0;
          Solution s = DualTFLunaRangeSensor.solve(RangeReference.LOW_WALL, l, r, lv, rv);
          assertEquals(oldDistance(l, r, lv, rv), s.targetDistance(), 1e-12, l + "," + r + "," + mask);
        }
        assertEquals(oldYaw(l, r), DualTFLunaRangeSensor.yawRad(RangeReference.LOW_WALL, l, r), 1e-12);
      }
    }
  }

  /** LOW_WALL 下 wallDistance = 右 frame 到九宮格（給砲台 Luna 用）。 */
  @Test
  void lowWallWallDistanceIncludesLowWallGap() {
    Solution s = DualTFLunaRangeSensor.solve(RangeReference.LOW_WALL, 0.5, 0.5, true, true);
    assertEquals(0.5 + 2.5, s.wallDistance(), 1e-9);
  }

  private static Solution grid(double left, double right) {
    return DualTFLunaRangeSensor.solve(RangeReference.GRID_WALL, left, right, true, true);
  }
}
