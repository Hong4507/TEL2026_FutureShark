package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.*;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

import frc.FSLib.util.Util;

/**
 * 射球仰角查表：「到目標的水平距離 → 仰角」，每一列（低／中／高／加分格）各一張表，表之間線性插值。
 *
 * <p>距離用 {@code RangeSensor/TargetDistance}（兩顆 TF-Luna 算出來的距離，和 ColumnShot 用的是同一個值）。
 * 仰角是 {@link Turret#setPitch} 的設定值（rad），和 ManualTurret 的 log 同單位。
 *
 * <p>怎麼量：Checklist-1114.md §5.6。Manual 模式在每個距離調到命中，每射一發 console 會印一行
 * {@code ShotTable 樣本 ...}，把命中的那一行的 {@code {距離, 仰角},} 貼到下面對應的表。
 *
 * <p>規則：
 * <ul>
 *   <li>某列的表是空的 → 用原本的彈道公式（{@link #ballisticPitchRad}），所以表空著時行為和以前一樣。</li>
 *   <li>距離在表的範圍內 → 線性插值。</li>
 *   <li>距離超出範圍 → 用彈道公式，再加上「最靠近那一端的實測值 − 公式值」的差（讓兩邊接起來，不會突然跳）。</li>
 * </ul>
 */
public final class ShotTable {
  private ShotTable() {}

  // ===== 實測資料：{TargetDistance (m), 仰角 (rad)}，順序不拘 =====
  // 11/15 上機量測後用 tools/measure/apply_measurements.py 填入（1 點就能用＝公式加固定修正；多點更準）

  /** 低列（kLowTargetHeight）。 */
  static final double[][] kLow = {
    {3.040, 1.3350},  // 5/5
    {3.230, 1.3320},  // 5/5
    {3.300, 1.3500},  // 5/5
    {3.905, 1.3744},  // 5/5
  };
  /** 中列（kMidTargetHeight）。 */
  static final double[][] kMid = {
    {3.020, 1.1330},  // 5/5
    {3.270, 1.1900},  // 5/5
    {3.480, 1.1940},  // 5/5
    {3.905, 1.2298},  // 5/5
  };
  /** 高列（kHighTargetHeight）。 */
  static final double[][] kHigh = {
    {3.040, 0.9610},  // 5/5
    {3.480, 1.0390},  // 4/5
    {3.800, 1.0820},  // 5/5
    {3.887, 1.0915},  // 5/5
  };
  /** 加分格（kSpecialTargetHeight）。 */
  static final double[][] kBonus = {
  };

  /** 彈道公式裡的係數（原本寫死在 ColumnShot／AssistTurret）。 */
  public static final double kBallisticCoefficient = 0.019358;

  public enum Source { TABLE, EXTRAPOLATED, FORMULA }

  private static final Row[] kRows = {
    new Row("LOW", kLowTargetHeight, kLow),
    new Row("MID", kMidTargetHeight, kMid),
    new Row("HIGH", kHighTargetHeight, kHigh),
    new Row("BONUS", kSpecialTargetHeight, kBonus),
  };

  record Row(String name, double heightMeters, double[][] points, InterpolatingDoubleTreeMap map,
      double minDistance, double maxDistance) {
    Row(String name, double heightMeters, double[][] points) {
      this(name, heightMeters, points, toMap(points), min(points), max(points));
    }

    /** 1 點就能用：那個距離剛好是實測值，其他距離 = 公式 + (實測 − 公式) 的固定差（§8.5 時間不夠每列只量 1 點）。 */
    boolean hasTable() {
      return points.length >= 1;
    }
  }

  /** 計算結果（給 log 與測試用）。 */
  public record Result(double pitchRad, Source source, String row) {}

  /** 依目標高度與距離算仰角（已限制在砲台行程內），並寫 log。 */
  public static double pitchRad(double targetHeightMeters, double distanceMeters) {
    Result r = compute(targetHeightMeters, distanceMeters);
    Logger.recordOutput("ShotTable/Row", r.row());
    Logger.recordOutput("ShotTable/Source", r.source().name());
    Logger.recordOutput("ShotTable/DistanceMeters", distanceMeters);
    Logger.recordOutput("ShotTable/PitchRad", r.pitchRad());
    return r.pitchRad();
  }

  /** 純計算，不寫 log。 */
  public static Result compute(double targetHeightMeters, double distanceMeters) {
    return compute(findRow(targetHeightMeters), targetHeightMeters, distanceMeters);
  }

  static Result compute(Row row, double targetHeightMeters, double distanceMeters) {
    double formula = ballisticPitchRad(targetHeightMeters, distanceMeters);
    if (row == null || !row.hasTable()) {
      return new Result(clamp(formula), Source.FORMULA, row == null ? "OTHER" : row.name());
    }
    if (distanceMeters >= row.minDistance() && distanceMeters <= row.maxDistance()) {
      return new Result(clamp(row.map().get(distanceMeters)), Source.TABLE, row.name());
    }
    double edge = distanceMeters < row.minDistance() ? row.minDistance() : row.maxDistance();
    double offset = row.map().get(edge) - ballisticPitchRad(targetHeightMeters, edge);
    return new Result(clamp(formula + offset), Source.EXTRAPOLATED, row.name());
  }

  /** 原本的彈道公式（未限制行程）。 */
  public static double ballisticPitchRad(double targetHeightMeters, double distanceMeters) {
    return Math.PI / 2 - Math.atan2(
      targetHeightMeters - TurretConstants.kTurretToRobotCenter.getZ()
        + Math.pow(distanceMeters, 2) * kBallisticCoefficient,
      distanceMeters);
  }

  private static Row findRow(double heightMeters) {
    for (Row r : kRows) {
      if (Math.abs(r.heightMeters() - heightMeters) < 0.05) return r;
    }
    return null;
  }

  private static double clamp(double pitch) {
    return Util.clamp(pitch, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
  }

  private static InterpolatingDoubleTreeMap toMap(double[][] points) {
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    for (double[] p : points) {
      if (p.length != 2) throw new IllegalArgumentException("ShotTable 每一點要是 {距離, 仰角}");
      map.put(p[0], p[1]);
    }
    return map;
  }

  private static double min(double[][] points) {
    double m = Double.POSITIVE_INFINITY;
    for (double[] p : points) m = Math.min(m, p[0]);
    return m;
  }

  private static double max(double[][] points) {
    double m = Double.NEGATIVE_INFINITY;
    for (double[] p : points) m = Math.max(m, p[0]);
    return m;
  }
}
