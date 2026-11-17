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
  private final LoggedNetworkNumber widthScale =
      new LoggedNetworkNumber("/Tuning/Vision/WidthScale", kDefaultWidthScale);
  /** 連續有效一段時間才採用視覺距離（避免偶爾一張誤判）。 */
  private final org.wpilib.math.filter.Debouncer distanceDebouncer =
      new org.wpilib.math.filter.Debouncer(0.3, org.wpilib.math.filter.Debouncer.DebounceType.RISING);
  private OptionalDouble distanceEstimate = OptionalDouble.empty();

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

    // 視覺估距離（先算，下面 RangeSensors 可能會用到它）
    OptionalDouble raw = inputs.connected
        ? estimateTargetDistance(detections, widthScale.get()) : OptionalDouble.empty();
    boolean stable = distanceDebouncer.calculate(raw.isPresent());
    distanceEstimate = stable ? raw : OptionalDouble.empty();
    Logger.recordOutput("Vision/DistanceRawMeters", raw.orElse(Double.NaN));
    Logger.recordOutput("Vision/DistanceMeters", distanceEstimate.orElse(Double.NaN));

    // 每次都重新拿（RangeSensors 可能被換成融合版，它會用到上面的視覺距離）
    TargetRangeSensor rangeSensor = RangeSensors.get();
    double distance = Vision.cameraDistance(rangeSensor.getTargetDistance());
    Logger.recordOutput("Vision/DetectionCount", detections.size());
    Logger.recordOutput("Vision/ColumnErrorsMeters", getColumnErrors(distance).stream().mapToDouble(Double::doubleValue).toArray());
    Logger.recordOutput("Vision/LitErrorMeters", getLitError(distance).orElse(Double.NaN));
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  /** Jetson 回報的處理幀率。 */
  public double getFps() {
    return inputs.connected ? inputs.fps : 0.0;
  }

  /** 這個週期是否有新的一張畫面結果。控制迴圈只在有新畫面時更新量測值。 */
  public boolean hasNewFrame() {
    return inputs.newFrame;
  }

  public List<Detection> getDetections() {
    return detections;
  }

  /**
   * 視覺估的距離（和 RangeSensor/TargetDistance 同定義：DownShooter 輪心到九宮格牆面），
   * 連續 0.3 s 有效才給；沒有就 empty。
   */
  public OptionalDouble getDistanceEstimate() {
    return distanceEstimate;
  }

  /**
   * 用大洞的框寬估距離：取所有「完整」大洞（沒被畫面邊緣切到）各自的估計值的中位數。至少 {@link VisionConstants#kMinHolesForDistance} 個。
   *
   * <p>Jetson 的框寬 w = atan(x右) − atan(x左)（同一列像素的兩邊），x = X/Zc，所以 Zc ≈ D / (2 tan(w/2) (1 + x²))。
   * 相機往上仰 θ：水平深度 = Zc × (cos θ − y sin θ)，y = tan(ty) √(1 + x²)。最後減掉鏡頭在輪心後方的距離。
   */
  public static OptionalDouble estimateTargetDistance(List<Detection> detections, double widthScale) {
    double halfH = Math.toRadians(kHorizontalFovDeg) / 2 - Math.toRadians(1);
    double halfV = Math.toRadians(kVerticalFovDeg) / 2 - Math.toRadians(1);
    double pitch = Math.toRadians(kCameraPitchDeg);
    List<Double> values = new ArrayList<>();
    for (Detection d : detections) {
      if (d.classId() != kClassHole || !(d.widthRad() > 0)) continue;
      if (Math.abs(d.txRad()) + d.widthRad() / 2 > halfH) continue;
      if (Math.abs(d.tyRad()) + d.widthRad() / 2 > halfV) continue;
      double x = Math.tan(d.txRad());
      double y = Math.tan(d.tyRad()) * Math.sqrt(1 + x * x);
      double zc = kHoleDiameterMeters * widthScale / (2 * Math.tan(d.widthRad() / 2) * (1 + x * x));
      double depth = zc * (Math.cos(pitch) - y * Math.sin(pitch));
      values.add(depth - kCameraBehindShooterMeters);
    }
    if (values.size() < kMinHolesForDistance) return OptionalDouble.empty();
    values.sort(null);
    int n = values.size();
    double median = n % 2 == 1 ? values.get(n / 2) : (values.get(n / 2 - 1) + values.get(n / 2)) / 2;
    return OptionalDouble.of(median);
  }

  /** DownShooter 輪心到板面距離（TargetDistance）→ 攝影機鏡頭到板面距離。 */
  public static double cameraDistance(double targetDistanceMeters) {
    return targetDistanceMeters + kCameraBehindShooterMeters;
  }

  /** 目標沿機器人前進方向、相對射球點的位置（公尺）。distanceMeters 是攝影機鏡頭到板面的水平距離。 */
  public static double errorMeters(Detection d, double distanceMeters) {
    double sign = kImageRightIsRobotForward ? 1 : -1;
    return sign * distanceMeters * horizontalBearingTan(d.txRad(), d.tyRad()) - kShooterForwardOfCameraMeters;
  }

  /**
   * 攝影機仰角 {@link VisionConstants#kCameraPitchDeg} 時，(tx, ty) → 水平面上偏離鏡頭朝向的角度的 tan。
   *
   * <p>Jetson 的定義：tx = atan(x)、ty = atan(y / √(1 + x²))，(x, y, 1) 是去畸變後的相機座標射線（y 往上）。
   * 相機往上仰 θ：射線在水平面上的前進分量 = cos θ − y·sin θ，側向分量 = x。
   */
  public static double horizontalBearingTan(double txRad, double tyRad) {
    double x = Math.tan(txRad);
    double y = Math.tan(tyRad) * Math.sqrt(1 + x * x);
    double pitch = Math.toRadians(kCameraPitchDeg);
    return x / (Math.cos(pitch) - y * Math.sin(pitch));
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

  /**
   * {@link #getColumnError(Column, double)} 的純計算版本：輸入已排序（前到後）的欄位位置。
   *
   * <p>只接受間距像九宮格的組合，避免把誤判的框（燈、隊服）當成一欄：
   * <ul>
   *   <li>三欄：兩個間距要差不多，而且是欄距的 {@link #kMinScale}~{@link #kMaxScale} 倍
   *       （TF-Luna 讀錯或用預設距離時，欄距看起來會縮放）。看到四群以上時，挑最像九宮格的三群，多的當雜訊。</li>
   *   <li>只看到兩群：間距要像「左、右兩欄」（{@link #kMinPairGapMeters}~{@link #kMaxPairGapMeters}），
   *       中欄取中點。下限比較嚴，免得把相鄰兩欄當成左右兩欄。</li>
   * </ul>
   */
  public static OptionalDouble columnError(List<Double> cols, Column column) {
    double[] grid = findGrid(cols);
    return grid == null ? OptionalDouble.empty() : OptionalDouble.of(grid[column.ordinal()]);
  }

  /** 三欄時，欄距看起來可以是實際的幾倍。兩個間距必須相等，所以範圍放寬也不容易誤認。 */
  static final double kMinScale = 0.6;
  static final double kMaxScale = 1.6;
  /** 只看到兩群時，當成「左、右兩欄」的間距範圍（實際 1.8 m）。 */
  static final double kMinPairGapMeters = 1.5 * kColumnSpacingMeters;
  static final double kMaxPairGapMeters = 2.5 * kColumnSpacingMeters;
  /** 三欄的兩個間距最多差多少（同一張畫面兩個間距的縮放相同，只有框的誤差）。 */
  static final double kMaxGapMismatchMeters = 0.2 * kColumnSpacingMeters;

  /** 回傳 {LEFT, MID, RIGHT} 的位置；認不出九宮格就回傳 null。 */
  private static double[] findGrid(List<Double> cols) {
    double minGap = kMinScale * kColumnSpacingMeters;
    double maxGap = kMaxScale * kColumnSpacingMeters;

    double[] best = null;
    double bestMismatch = kMaxGapMismatchMeters;
    for (int i = 0; i < cols.size(); i++) {
      for (int j = i + 1; j < cols.size(); j++) {
        double g1 = cols.get(i) - cols.get(j);
        if (g1 < minGap || g1 > maxGap) continue;
        for (int k = j + 1; k < cols.size(); k++) {
          double g2 = cols.get(j) - cols.get(k);
          if (g2 < minGap || g2 > maxGap) continue;
          double mismatch = Math.abs(g1 - g2);
          if (mismatch <= bestMismatch) {
            bestMismatch = mismatch;
            best = new double[] {cols.get(i), cols.get(j), cols.get(k)};
          }
        }
      }
    }
    if (best != null) return best;

    // 只看到左右兩欄、中欄被擋住。多於兩群又湊不出三欄時無法判斷，不猜。
    if (cols.size() == 2) {
      double gap = cols.get(0) - cols.get(1);
      if (gap > kMinPairGapMeters && gap <= kMaxPairGapMeters) {
        return new double[] {cols.get(0), (cols.get(0) + cols.get(1)) / 2, cols.get(1)};
      }
    }
    return null;
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
