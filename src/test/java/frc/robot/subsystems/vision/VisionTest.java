package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.vision.Vision.Column;
import frc.robot.subsystems.vision.Vision.Detection;

class VisionTest {
  private static final double kDistance = 2.5;
  private static final double kEps = 1e-6;

  /** 在「相對射球點 errorMeters」的位置放一個洞，反推攝影機看到的 tx。 */
  private static Detection holeAt(double errorMeters, int cls) {
    double sign = kImageRightIsRobotForward ? 1 : -1;
    double tx = Math.atan((errorMeters + kShooterForwardOfCameraMeters) / (sign * kDistance));
    return new Detection(cls, tx, 0, 0.9, 0.1, cls == kClassBonus ? 0 : -1);
  }

  /** 一整欄：三個大洞加一個加分洞。 */
  private static List<Detection> column(double errorMeters) {
    List<Detection> out = new ArrayList<>();
    for (int i = 0; i < 3; i++) out.add(holeAt(errorMeters, kClassHole));
    out.add(holeAt(errorMeters, kClassBonus));
    return out;
  }

  @Test
  void errorMetersRoundTrips() {
    assertEquals(0.37, Vision.errorMeters(holeAt(0.37, kClassHole), kDistance), kEps);
  }

  @Test
  void threeColumnsSortedFrontToBack() {
    List<Detection> d = new ArrayList<>();
    d.addAll(column(-0.5));
    d.addAll(column(1.3));
    d.addAll(column(0.4));
    List<Double> cols = Vision.columnErrors(d, kDistance);
    assertEquals(3, cols.size());
    assertEquals(1.3, cols.get(0), kEps);
    assertEquals(0.4, cols.get(1), kEps);
    assertEquals(-0.5, cols.get(2), kEps);
    assertEquals(1.3, Vision.columnError(cols, Column.LEFT).getAsDouble(), kEps);
    assertEquals(0.4, Vision.columnError(cols, Column.MID).getAsDouble(), kEps);
    assertEquals(-0.5, Vision.columnError(cols, Column.RIGHT).getAsDouble(), kEps);
  }

  @Test
  void adjacentTwoColumnsAreAmbiguous() {
    List<Detection> d = new ArrayList<>();
    d.addAll(column(0.9));
    d.addAll(column(0.0));
    List<Double> cols = Vision.columnErrors(d, kDistance);
    assertEquals(2, cols.size());
    assertTrue(Vision.columnError(cols, Column.LEFT).isEmpty());
  }

  @Test
  void outerTwoColumnsInferMid() {
    List<Detection> d = new ArrayList<>();
    d.addAll(column(1.0));
    d.addAll(column(-0.8));
    List<Double> cols = Vision.columnErrors(d, kDistance);
    assertEquals(0.1, Vision.columnError(cols, Column.MID).getAsDouble(), kEps);
  }

  @Test
  void nearestRespectsGate() {
    List<Double> cols = List.of(0.9, 0.05, -0.85);
    assertEquals(0.05, Vision.nearest(cols, 0.2, 0.3).getAsDouble(), kEps);
    assertTrue(Vision.nearest(cols, 0.45, 0.3).isEmpty());
  }
}
