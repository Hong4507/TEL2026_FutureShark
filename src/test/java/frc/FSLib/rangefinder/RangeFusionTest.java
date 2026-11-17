package frc.FSLib.rangefinder;

import static frc.FSLib.rangefinder.RangeSensorConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;

import frc.FSLib.rangefinder.RangeFusion.Result;
import frc.FSLib.rangefinder.RangeFusion.Source;
import frc.FSLib.rangefinder.RangeSensorConstants.RangeReference;

/** 距離融合：Luna×2 → 一顆被擋用另一顆＋IMU → 兩顆都被擋用 Limelight → 預設。 */
class RangeFusionTest {
  private static final double WALL = 2.40; // 右 frame 到牆
  private static final OptionalDouble NONE = OptionalDouble.empty();

  /** 車身相對牆面偏 yaw（rad，kYawSign 方向）時，兩顆 Luna 的理想讀值。 */
  private static double[] readings(double wall, double yaw) {
    // 反解 GRID_WALL 公式：dL_f − dR_f = B tan(yaw/sign)，(dL_f + dR_f)/2 × cos(yaw) = wall
    double y = yaw / kYawSign;
    double mean = wall / Math.cos(yaw);
    double diff = kBaselineMeters * Math.tan(y);
    double dL = mean + diff / 2 + kLeftLunaInboardMeters + kLunaTrimMeters;
    double dR = mean - diff / 2 + kRightLunaInboardMeters;
    return new double[] {dL, dR};
  }

  private static RangeFusion learned(double gyro) {
    RangeFusion f = new RangeFusion(RangeReference.GRID_WALL);
    double[] r = readings(WALL, 0);
    f.update(0, r[0], r[1], true, true, gyro, NONE); // 正對牆時建立基準
    return f;
  }

  @Test
  void dualLunaIsDefaultAndLearnsWallHeading() {
    RangeFusion f = new RangeFusion(RangeReference.GRID_WALL);
    double[] r = readings(WALL, 0);
    Result res = f.update(0, r[0], r[1], true, true, 1.0, NONE);
    assertEquals(Source.LUNA_DUAL, res.source());
    assertEquals(WALL + kShooterInboardMeters, res.targetDistance(), 1e-6);
    assertEquals(1.0, f.getWallHeading(), 1e-9);
  }

  @Test
  void turnedRobotStaysDualWhenImuAgrees() {
    RangeFusion f = learned(1.0);
    double yaw = Math.toRadians(15);
    double[] r = readings(WALL, yaw);
    Result res = f.update(0.1, r[0], r[1], true, true, 1.0 + yaw, NONE);
    assertEquals(Source.LUNA_DUAL, res.source());
    assertEquals(WALL + kShooterInboardMeters, res.targetDistance(), 1e-6);
  }

  @Test
  void oneLunaBlockedUsesOtherPlusImu() {
    RangeFusion f = learned(1.0);
    double yaw = Math.toRadians(8);
    for (boolean leftBlocked : new boolean[] {true, false}) {
      double[] r = readings(WALL, yaw);
      if (leftBlocked) r[0] -= 1.2; else r[1] -= 1.2; // 對手擋在前面：讀值變近
      Result res = f.update(0.2, r[0], r[1], true, true, 1.0 + yaw, NONE);
      assertEquals(Source.LUNA_SINGLE_IMU, res.source(), "leftBlocked=" + leftBlocked);
      // 單邊牆距 = d_frame × cos(yaw)，和真正的牆距差在 基線/2 × sin(yaw) 量級（幾公分內）
      assertEquals(WALL + kShooterInboardMeters, res.targetDistance(), 0.02, "leftBlocked=" + leftBlocked);
      assertEquals(yaw, res.yawRad(), 1e-9, "偏角改用 IMU");
    }
  }

  @Test
  void oneLunaInvalidUsesOtherPlusImu() {
    RangeFusion f = learned(1.0);
    double[] r = readings(WALL, 0);
    Result res = f.update(0.2, 0, r[1], false, true, 1.0, NONE);
    assertEquals(Source.LUNA_SINGLE_IMU, res.source());
    assertEquals(WALL + kShooterInboardMeters, res.targetDistance(), 1e-6);
    assertEquals(0, res.yawRad(), 1e-9); // LaserAlign 還有角度可以用
  }

  @Test
  void bothBlockedSwitchesToVisionAfterDelay() {
    RangeFusion f = learned(1.0);
    double[] r = readings(1.0, 0); // 兩顆都讀到 1.0 m 外的對手
    OptionalDouble vision = OptionalDouble.of(WALL + kShooterInboardMeters);
    assertEquals(Source.LUNA_DUAL, f.update(1.0, r[0], r[1], true, true, 1.0, vision).source());
    assertEquals(Source.LUNA_DUAL, f.update(1.1, r[0], r[1], true, true, 1.0, vision).source());
    Result res = f.update(1.25, r[0], r[1], true, true, 1.0, vision);
    assertEquals(Source.VISION, res.source());
    assertEquals(vision.getAsDouble(), res.targetDistance(), 1e-9);
    // 擋住的東西離開 → 馬上回到 Luna
    double[] clear = readings(WALL, 0);
    assertEquals(Source.LUNA_DUAL, f.update(1.3, clear[0], clear[1], true, true, 1.0, vision).source());
  }

  @Test
  void lunaFartherThanVisionKeepsLuna() {
    RangeFusion f = learned(1.0);
    double[] r = readings(WALL, 0);
    OptionalDouble vision = OptionalDouble.of(WALL + kShooterInboardMeters - 0.8); // 視覺說比較近 → 不信視覺
    for (double t = 0; t < 1; t += 0.05) {
      assertEquals(Source.LUNA_DUAL, f.update(t, r[0], r[1], true, true, 1.0, vision).source());
    }
  }

  @Test
  void noLunaUsesVisionThenFallback() {
    RangeFusion f = new RangeFusion(RangeReference.GRID_WALL);
    assertEquals(Source.VISION, f.update(0, 0, 0, false, false, 0, OptionalDouble.of(2.7)).source());
    Result res = f.update(0, 0, 0, false, false, 0, NONE);
    assertEquals(Source.FALLBACK, res.source());
    assertEquals(kFallbackDistanceMeters, res.targetDistance(), 1e-12);
  }

  @Test
  void blockedBeforeReferenceDoesNotLearnGarbage() {
    RangeFusion f = new RangeFusion(RangeReference.GRID_WALL);
    double[] r = readings(WALL, 0);
    r[0] -= 1.2; // 一開機就被擋：偏角很大 → 不建立基準
    f.update(0, r[0], r[1], true, true, 0.5, NONE);
    assertTrue(Double.isNaN(f.getWallHeading()));
  }

  @Test
  void wallHeadingTracksImuDriftSlowly() {
    RangeFusion f = learned(0.0);
    double[] r = readings(WALL, 0);
    for (int i = 0; i < 200; i++) f.update(i * 0.01, r[0], r[1], true, true, Math.toRadians(2), NONE); // IMU 漂了 2°
    assertEquals(Math.toRadians(2), f.getWallHeading(), Math.toRadians(0.1));
  }
}
