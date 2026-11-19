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
    double yaw = Math.toRadians(8);
    for (boolean leftBlocked : new boolean[] {true, false}) {
      RangeFusion f = learned(1.0);
      double[] r = readings(WALL, yaw);
      if (leftBlocked) r[0] -= 1.2; else r[1] -= 1.2; // 對手擋在前面：讀值變近
      // 剛被擋的前 kBlockedConfirmSec 還是 Luna×2（避免雜訊一跳就切）
      assertEquals(Source.LUNA_DUAL, f.update(0.2, r[0], r[1], true, true, 1.0 + yaw, NONE).source());
      Result res = null;
      for (double t = 0.4; t < 3.0; t += 0.05) {
        res = f.update(t, r[0], r[1], true, true, 1.0 + yaw, NONE);
        assertEquals(Source.LUNA_SINGLE_IMU, res.source(), "leftBlocked=" + leftBlocked + " t=" + t);
      }
      // 補上沿牆偏移後和真正的牆距一致
      assertEquals(WALL + kShooterInboardMeters, res.targetDistance(), 1e-6, "leftBlocked=" + leftBlocked);
      assertEquals(yaw, res.yawRad(), 1e-9, "偏角改用 IMU");
      assertEquals(0, f.getWallHeadingResets(), "被擋（偏角很大）不能重設基準");
    }
  }

  @Test
  void singleLunaWithImuMatchesTrueWallWhenTurned() {
    for (double deg : new double[] {-20, -8, 8, 20}) {
      double yaw = Math.toRadians(deg);
      double[] r = readings(WALL, yaw);
      RangeFusion f = learned(1.0);
      Result onlyRight = f.update(0.1, 0, r[1], false, true, 1.0 + yaw, NONE);
      Result onlyLeft = f.update(0.2, r[0], 0, true, false, 1.0 + yaw, NONE);
      assertEquals(WALL + kShooterInboardMeters, onlyRight.targetDistance(), 1e-6, "right deg=" + deg);
      assertEquals(WALL + kShooterInboardMeters, onlyLeft.targetDistance(), 1e-6, "left deg=" + deg);
    }
  }

  @Test
  void lunaNoiseOfAFewDegreesStaysDual() {
    RangeFusion f = learned(1.0);
    for (int i = 0; i < 100; i++) {
      double[] r = readings(WALL, 0);
      r[i % 2] += 0.015; // 1.5 cm 雜訊 ≈ 5°
      assertEquals(Source.LUNA_DUAL, f.update(0.1 + i * 0.02, r[0], r[1], true, true, 1.0, NONE).source());
    }
  }

  @Test
  void staleImuReferenceIsResetInsteadOfStuckInSingle() {
    RangeFusion f = learned(1.0);
    double[] r = readings(WALL, 0);
    double driftedGyro = 1.0 + Math.toRadians(30); // IMU 歪了／換了一面牆，但兩顆 Luna 看起來正對牆
    Result res = null;
    for (double t = 0.1; t < 1.5; t += 0.02) res = f.update(t, r[0], r[1], true, true, driftedGyro, NONE);
    assertEquals(Source.LUNA_DUAL, res.source());
    assertEquals(1, f.getWallHeadingResets());
    assertEquals(driftedGyro, f.getWallHeading(), 1e-9);
    // 重設之後一切正常
    assertEquals(Source.LUNA_DUAL, f.update(2.0, r[0], r[1], true, true, driftedGyro, NONE).source());
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

  /** 11/19：開機打不到牆 → 用起點角度設基準，一顆被擋時單邊+IMU 也能用；之後 Luna×2 照常一致。 */
  @Test
  void seededWallHeadingWorksWithoutDualLearning() {
    RangeFusion f = new RangeFusion(RangeReference.GRID_WALL);
    f.seedWallHeading(1.0, 0.0); // 車子擺在起點、正對牆，IMU 讀 1.0 rad
    assertEquals(1.0, f.getWallHeading(), 1e-9);

    double yaw = Math.toRadians(12);
    double[] r = readings(WALL, yaw);
    Result single = f.update(0, r[0], r[1], true, false, 1.0 + yaw, NONE); // 右邊無效
    assertEquals(Source.LUNA_SINGLE_IMU, single.source());
    assertEquals(WALL, single.wallDistance(), 1e-6);
    assertEquals(yaw, single.yawRad(), 1e-9);

    Result dual = f.update(0.1, r[0], r[1], true, true, 1.0 + yaw, NONE);
    assertEquals(Source.LUNA_DUAL, dual.source());
    assertEquals(0, f.getLunaImuDiffRad(), 1e-6);
  }

  @Test
  void seedWithKnownStartAngle() {
    RangeFusion f = new RangeFusion(RangeReference.GRID_WALL);
    f.seedWallHeading(0.5, Math.toRadians(5)); // 起點本來就偏 5°
    double[] r = readings(WALL, Math.toRadians(5));
    Result res = f.update(0, r[0], r[1], true, true, 0.5, NONE);
    assertEquals(Source.LUNA_DUAL, res.source());
    assertEquals(0, f.getLunaImuDiffRad(), 1e-6);
  }

  /** 11/19：init 時車頭朝牆（−90°）→ Luna 打到別的平面也不會把基準重設掉；轉 90° 回來後單邊+IMU 正確。 */
  @Test
  void startFacingWallDoesNotRebaseOnOtherSurface() {
    RangeFusion f = new RangeFusion(RangeReference.GRID_WALL);
    double start = 0.3;
    f.seedWallHeading(start, Math.toRadians(-90));
    double[] side = readings(1.0, 0); // Luna 打到側邊一面平牆
    for (int i = 0; i <= 100; i++) f.update(i * 0.02, side[0], side[1], true, true, start, NONE);
    assertEquals(0, f.getWallHeadingResets());
    assertEquals(start + Math.toRadians(90), f.getWallHeading(), 1e-9);

    // 逆時針轉 90° → 右 frame 正對九宮格；一顆被擋
    double yaw = Math.toRadians(5);
    double[] r = readings(WALL, yaw);
    Result res = f.update(3, r[0], r[1], false, true, start + Math.toRadians(90) + yaw, NONE);
    assertEquals(Source.LUNA_SINGLE_IMU, res.source());
    assertEquals(WALL, res.wallDistance(), 1e-6);
  }
}
