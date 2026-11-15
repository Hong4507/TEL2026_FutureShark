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
    // ty = 0（在相機光軸高度）時，水平 tan = tan(tx) / cos(仰角)
    double tx = Math.atan(Math.cos(Math.toRadians(kCameraPitchDeg)) * (errorMeters + kShooterForwardOfCameraMeters) / (sign * kDistance));
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

  // ---------- 邊界情況 ----------

  private static void assertNoColumn(List<Double> cols) {
    for (Column c : Column.values()) {
      assertTrue(Vision.columnError(cols, c).isEmpty(), c + " 應該分辨不出來，實際 " + Vision.columnError(cols, c));
    }
  }

  @Test
  void nothingSeen() {
    List<Double> cols = Vision.columnErrors(List.of(), kDistance);
    assertTrue(cols.isEmpty());
    assertNoColumn(cols);
    assertTrue(Vision.nearest(cols, 0, Double.POSITIVE_INFINITY).isEmpty());
  }

  @Test
  void singleColumnIsAmbiguous() {
    List<Double> cols = Vision.columnErrors(column(0.3), kDistance);
    assertEquals(1, cols.size());
    assertNoColumn(cols);
  }

  @Test
  void partialColumnAtImageEdgeStillClusters() {
    // 畫面邊緣只露出一個洞：仍然算一欄，不會拆成兩欄
    List<Detection> d = new ArrayList<>();
    d.addAll(column(0.9));
    d.addAll(column(0.0));
    d.add(holeAt(-0.9, kClassHole));
    List<Double> cols = Vision.columnErrors(d, kDistance);
    assertEquals(3, cols.size());
    assertEquals(-0.9, Vision.columnError(cols, Column.RIGHT).getAsDouble(), kEps);
  }

  @Test
  void noisyBoxesInOneColumnStayOneColumn() {
    // 同一欄的框左右各差 ±8 cm（框不準），不能被拆成兩欄
    List<Detection> d = new ArrayList<>();
    for (double e : new double[] {0.82, 0.98, 0.9}) d.add(holeAt(e, kClassHole));
    d.addAll(column(0.0));
    d.addAll(column(-0.9));
    List<Double> cols = Vision.columnErrors(d, kDistance);
    assertEquals(3, cols.size());
    assertEquals(0.9, Vision.columnError(cols, Column.LEFT).getAsDouble(), 0.01);
  }

  @Test
  void falseBoxNextToTwoColumnsIsNotTakenAsGrid() {
    // 看到中、右兩欄，加上一個誤判的框（燈、隊服）在中欄前方 0.45 m。
    // 三群但間距不像九宮格，不能把假框當成左欄，也不能把中欄當成別欄。
    List<Detection> d = new ArrayList<>();
    d.addAll(column(0.0));
    d.addAll(column(-0.9));
    d.add(holeAt(0.45, kClassHole));
    List<Double> cols = Vision.columnErrors(d, kDistance);
    assertEquals(3, cols.size());
    assertNoColumn(cols);
  }

  @Test
  void falseBoxFarAheadDoesNotBreakFullGrid() {
    // 三欄都看到，再加一個遠處的假框：仍然要找得到正確的三欄
    List<Detection> d = new ArrayList<>();
    d.addAll(column(0.9));
    d.addAll(column(0.0));
    d.addAll(column(-0.9));
    d.add(holeAt(2.4, kClassHole));
    List<Double> cols = Vision.columnErrors(d, kDistance);
    assertEquals(4, cols.size());
    assertEquals(0.9, Vision.columnError(cols, Column.LEFT).getAsDouble(), kEps);
    assertEquals(0.0, Vision.columnError(cols, Column.MID).getAsDouble(), kEps);
    assertEquals(-0.9, Vision.columnError(cols, Column.RIGHT).getAsDouble(), kEps);
  }

  @Test
  void falseBoxBetweenColumnsDoesNotBreakFullGrid() {
    List<Detection> d = new ArrayList<>();
    d.addAll(column(0.9));
    d.addAll(column(0.0));
    d.addAll(column(-0.9));
    d.add(holeAt(-0.45, kClassHole));
    List<Double> cols = Vision.columnErrors(d, kDistance);
    assertEquals(4, cols.size());
    assertEquals(0.0, Vision.columnError(cols, Column.MID).getAsDouble(), kEps);
    assertEquals(-0.9, Vision.columnError(cols, Column.RIGHT).getAsDouble(), kEps);
  }

  @Test
  void falseBoxFarFromSingleColumnIsNotTakenAsOuterPair() {
    // 只看到中欄，加上 2.5 m 外的假框：間距不像「左、右兩欄」（1.8 m），不能配成一對
    List<Detection> d = new ArrayList<>(column(0.0));
    d.add(holeAt(2.5, kClassHole));
    assertNoColumn(Vision.columnErrors(d, kDistance));
  }

  @Test
  void distanceErrorStillFindsGrid() {
    // TF-Luna 讀成 2.0 m（實際 2.5 m）：欄距看起來縮成 0.72 m，仍然要認得出三欄
    List<Detection> d = new ArrayList<>();
    d.addAll(column(0.9));
    d.addAll(column(0.0));
    d.addAll(column(-0.9));
    List<Double> cols = Vision.columnErrors(d, 2.0);
    assertEquals(3, cols.size());
    assertTrue(Vision.columnError(cols, Column.MID).isPresent());
  }

  @Test
  void outerPairWithDistanceErrorStillInfersMid() {
    // 只看到左右兩欄、距離讀小 20%：間距 1.44 m，仍然認得出是左右兩欄
    List<Detection> d = new ArrayList<>();
    d.addAll(column(0.9));
    d.addAll(column(-0.9));
    List<Double> cols = Vision.columnErrors(d, 2.0);
    assertTrue(Vision.columnError(cols, Column.MID).isPresent());
  }

  /** 攝影機仰 30°：直接正前方 tx = 0 → 0；不同高度的洞，同一個沿牆位置要換算成同一個值。 */
  @org.junit.jupiter.api.Test
  void pitchedCameraBearingIsHeightIndependent() {
    assertEquals(0, Vision.horizontalBearingTan(0, 0.3), 1e-12);
    double pitch = Math.toRadians(kCameraPitchDeg);
    double lateral = 0.5, depth = 2.6;
    for (double up : new double[] {0.7, 1.3, 1.9, 2.4}) {
      double zc = depth * Math.cos(pitch) + up * Math.sin(pitch);
      double yc = -depth * Math.sin(pitch) + up * Math.cos(pitch);
      double tx = Math.atan2(lateral, zc);
      double ty = Math.atan2(yc, Math.hypot(lateral, zc));
      assertEquals(lateral / depth, Vision.horizontalBearingTan(tx, ty), 1e-9, "up=" + up);
    }
  }
}
