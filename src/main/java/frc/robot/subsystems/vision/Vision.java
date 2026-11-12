package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.SubsystemBase;

import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;

/**
 * 九宮格偵測結果。只負責「看」，不控制任何馬達，所以 command 不需要 require 它。
 *
 * <p>位置都換算成「沿機器人前進方向、相對射球點」的距離（公尺）：
 * 正值代表目標在射球點前方，機器人要往前開。
 */
public class Vision extends SubsystemBase {
  /** 九宮格的三欄。機器人朝 +y（車頭朝左欄）時，LEFT 在最前方。 */
  public enum Column { LEFT, MID, RIGHT }

  public record Detection(int classId, double txRad, double tyRad, double confidence, double widthRad, double litScore) {}

  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  private final LoggedNetworkNumber minConfidence =
    new LoggedNetworkNumber("/Tuning/Vision/MinConfidence", kDefaultMinConfidence);
  private final LoggedNetworkNumber litThreshold =
    new LoggedNetworkNumber("/Tuning/Vision/LitThreshold", kDefaultLitThreshold);

  private List<Detection> detections = List.of();

  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    List<Detection> parsed = new ArrayList<>();
    double[] t = inputs.targets;
    for (int i = 0; i + kTargetStride <= t.length; i += kTargetStride) {
      Detection d = new Detection((int) t[i + kIdxClass], t[i + kIdxTx], t[i + kIdxTy],
        t[i + kIdxConfidence], t[i + kIdxWidth], t[i + kIdxLitScore]);
      if (d.confidence() >= minConfidence.get()) parsed.add(d);
    }
    detections = Collections.unmodifiableList(parsed);

    double distance = rangeSensor.getTargetDistance();
    Logger.recordOutput("Vision/DetectionCount", detections.size());
    Logger.recordOutput("Vision/ColumnErrorsMeters", getColumnErrors(distance).stream().mapToDouble(Double::doubleValue).toArray());
    Logger.recordOutput("Vision/LitErrorMeters", getLitError(distance).orElse(Double.NaN));
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  /** 這個週期是否有新的一張畫面結果。控制迴圈只在有新畫面時更新量測值。 */
  public boolean hasNewFrame() {
    return inputs.newFrame;
  }

  public List<Detection> getDetections() {
    return detections;
  }

  /** 目標沿機器人前進方向、相對射球點的位置（公尺）。 */
  public static double errorMeters(Detection d, double distanceMeters) {
    double sign = kImageRightIsRobotForward ? 1 : -1;
    return sign * distanceMeters * Math.tan(d.txRad()) - kShooterForwardOfCameraMeters;
  }

  /**
   * 把所有洞（大洞與加分洞）依沿牆位置分欄，回傳各欄中心相對射球點的位置，
   * 由前到後排序（index 0 最前方，也就是 LEFT 那一側）。
   */
  public List<Double> getColumnErrors(double distanceMeters) {
    return columnErrors(detections, distanceMeters);
  }

  /** {@link #getColumnErrors(double)} 的純計算版本（方便單元測試）。 */
  public static List<Double> columnErrors(List<Detection> detections, double distanceMeters) {
    List<double[]> points = new ArrayList<>(); // {error, weight}
    for (Detection d : detections) {
      points.add(new double[] {errorMeters(d, distanceMeters), d.confidence()});
    }
    points.sort((a, b) -> Double.compare(b[0], a[0]));

    List<Double> columns = new ArrayList<>();
    double sum = 0, weight = 0, last = Double.NaN;
    for (double[] p : points) {
      if (!Double.isNaN(last) && last - p[0] > kColumnClusterMeters) {
        columns.add(sum / weight);
        sum = 0;
        weight = 0;
      }
      sum += p[0] * p[1];
      weight += p[1];
      last = p[0];
    }
    if (weight > 0) columns.add(sum / weight);
    return columns;
  }

  /**
   * 指定欄位相對射球點的位置。需要能分辨是哪一欄：看到三欄，或看到相距兩欄的左右兩欄。
   * 只看到一欄或相鄰兩欄時無法分辨，回傳 empty。
   */
  public OptionalDouble getColumnError(Column column, double distanceMeters) {
    return columnError(getColumnErrors(distanceMeters), column);
  }

  /** {@link #getColumnError(Column, double)} 的純計算版本：輸入已排序（前到後）的欄位位置。 */
  public static OptionalDouble columnError(List<Double> cols, Column column) {
    if (cols.size() == 3) {
      return OptionalDouble.of(cols.get(column.ordinal()));
    }
    if (cols.size() == 2 && cols.get(0) - cols.get(1) > 1.5 * kColumnSpacingMeters) {
      return switch (column) {
        case LEFT -> OptionalDouble.of(cols.get(0));
        case RIGHT -> OptionalDouble.of(cols.get(1));
        case MID -> OptionalDouble.of((cols.get(0) + cols.get(1)) / 2);
      };
    }
    return OptionalDouble.empty();
  }

  /** 亮燈的加分洞那一欄相對射球點的位置；沒有亮燈的加分洞就回傳 empty。 */
  public OptionalDouble getLitError(double distanceMeters) {
    Detection best = null;
    for (Detection d : detections) {
      if (d.classId() == kClassBonus && d.litScore() >= litThreshold.get()
          && (best == null || d.litScore() > best.litScore())) {
        best = d;
      }
    }
    if (best == null) return OptionalDouble.empty();
    double litError = errorMeters(best, distanceMeters);
    // 用同一欄所有洞的平均位置，比單看小小的加分洞準
    return OptionalDouble.of(nearest(getColumnErrors(distanceMeters), litError, kColumnClusterMeters).orElse(litError));
  }

  /** 射球點目前對著哪一欄（誤差在半個欄距以內）；分辨不出來就回傳 empty。給記分板用。 */
  public java.util.Optional<Column> getAimedColumn(double distanceMeters) {
    Column best = null;
    double bestError = kColumnSpacingMeters / 2;
    for (Column c : Column.values()) {
      OptionalDouble e = getColumnError(c, distanceMeters);
      if (e.isPresent() && Math.abs(e.getAsDouble()) < bestError) {
        best = c;
        bestError = Math.abs(e.getAsDouble());
      }
    }
    return java.util.Optional.ofNullable(best);
  }

  /** 亮燈加分格在哪一欄；看不到亮燈或分辨不出欄位時回傳 empty。 */
  public java.util.Optional<Column> getLitColumn(double distanceMeters) {
    OptionalDouble lit = getLitError(distanceMeters);
    if (lit.isEmpty()) return java.util.Optional.empty();
    for (Column c : Column.values()) {
      OptionalDouble e = getColumnError(c, distanceMeters);
      if (e.isPresent() && Math.abs(e.getAsDouble() - lit.getAsDouble()) <= kColumnClusterMeters) {
        return java.util.Optional.of(c);
      }
    }
    return java.util.Optional.empty();
  }

  /** 離 {@code reference} 最近、且在 {@code gate} 以內的欄位。 */
  public static OptionalDouble nearest(List<Double> columns, double reference, double gate) {
    double best = Double.NaN;
    for (double c : columns) {
      if (Math.abs(c - reference) <= gate && (Double.isNaN(best) || Math.abs(c - reference) < Math.abs(best - reference))) {
        best = c;
      }
    }
    return Double.isNaN(best) ? OptionalDouble.empty() : OptionalDouble.of(best);
  }
}
