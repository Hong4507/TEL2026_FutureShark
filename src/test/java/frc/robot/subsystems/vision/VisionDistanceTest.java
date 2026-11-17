package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import frc.robot.subsystems.vision.Vision.Detection;

/** 視覺估距離：用和 Jetson 相同定義的 tx／ty／框寬，從已知幾何反推。 */
class VisionDistanceTest {
  /** 鏡頭到板面水平距離 depth、洞在鏡頭右側 right、高度 z 時，Jetson 會輸出的偵測。 */
  static Detection hole(double depth, double right, double z) {
    double up = z - kCameraHeightMeters;
    double pitch = Math.toRadians(kCameraPitchDeg);
    double zc = depth * Math.cos(pitch) + up * Math.sin(pitch);
    double yc = -depth * Math.sin(pitch) + up * Math.cos(pitch);
    double tx = Math.atan2(right, zc);
    double ty = Math.atan2(yc, Math.hypot(right, zc));
    double w = Math.atan((right + kHoleDiameterMeters / 2) / zc) - Math.atan((right - kHoleDiameterMeters / 2) / zc);
    return new Detection(kClassHole, tx, ty, 0.9, w, -1);
  }

  static List<Detection> grid(double depth, double shift) {
    List<Detection> out = new ArrayList<>();
    for (double y : new double[] {-0.9, 0, 0.9}) {
      for (double z : new double[] {kLowTargetHeight, kMidTargetHeight, kHighTargetHeight}) {
        out.add(hole(depth, y + shift, z));
      }
    }
    return out;
  }

  @ParameterizedTest(name = "鏡頭到板面 {0} m")
  @CsvSource({"2.4", "2.66", "3.1", "3.9"})
  void estimatesTargetDistance(double depth) {
    var est = Vision.estimateTargetDistance(grid(depth, 0.1), 1.0);
    assertTrue(est.isPresent());
    assertEquals(depth - kCameraBehindShooterMeters, est.getAsDouble(), 0.03);
  }

  @Test
  void widthScaleCalibrates() {
    double depth = 3.0;
    double base = Vision.estimateTargetDistance(grid(depth, 0), 1.0).getAsDouble() + kCameraBehindShooterMeters;
    double scaled = Vision.estimateTargetDistance(grid(depth, 0), 1.1).getAsDouble() + kCameraBehindShooterMeters;
    assertEquals(base * 1.1, scaled, 1e-9);
  }

  @Test
  void needsThreeCompleteHoles() {
    List<Detection> two = grid(2.66, 0).subList(0, 2);
    assertTrue(Vision.estimateTargetDistance(two, 1.0).isEmpty());
    // 被畫面邊緣切到的框不算
    List<Detection> edge = new ArrayList<>();
    for (int i = 0; i < 3; i++) edge.add(new Detection(kClassHole, Math.toRadians(30), 0, 0.9, Math.toRadians(5), -1));
    assertTrue(Vision.estimateTargetDistance(edge, 1.0).isEmpty());
    // 加分洞不算（直徑不同）
    List<Detection> bonus = new ArrayList<>();
    for (Detection d : grid(2.66, 0)) bonus.add(new Detection(kClassBonus, d.txRad(), d.tyRad(), 0.9, d.widthRad(), 0));
    assertTrue(Vision.estimateTargetDistance(bonus, 1.0).isEmpty());
  }

  @Test
  void medianRejectsOneBadBox() {
    List<Detection> dets = new ArrayList<>(grid(2.66, 0));
    dets.add(new Detection(kClassHole, 0, -0.2, 0.9, Math.toRadians(30), -1)); // 誤判的大框
    assertEquals(2.66 - kCameraBehindShooterMeters, Vision.estimateTargetDistance(dets, 1.0).getAsDouble(), 0.03);
  }
}
