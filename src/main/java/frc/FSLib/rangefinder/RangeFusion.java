package frc.FSLib.rangefinder;

import static frc.FSLib.rangefinder.RangeSensorConstants.*;

import java.util.OptionalDouble;

import org.wpilib.math.util.MathUtil;

import frc.FSLib.rangefinder.DualTFLunaRangeSensor.Solution;
import frc.FSLib.rangefinder.RangeSensorConstants.RangeReference;

/**
 * 距離融合（11/17）：車身兩顆 TF-Luna ＋ IMU ＋ Limelight。純計算＋少量狀態，單元測試直接用。
 *
 * <p>關鍵：擋在前面的東西只會讓 Luna 讀得比較<b>近</b>。
 * <ol>
 *   <li><b>Luna×2</b>：兩顆都有效，而且算出的偏角和 IMU 一致（差 &lt; {@link #kConsistencyRad}）→ 用原本的公式（最準）。
 *       順便更新「牆面方向」= IMU 角度 − Luna 偏角（慢慢平均，抵消 IMU 漂移）。</li>
 *   <li><b>Luna單邊+IMU</b>：兩顆不一致（一顆被擋）或只剩一顆有效 → 用比較遠的那顆，牆距 = d_frame × cos(IMU 偏角)。</li>
 *   <li><b>Limelight</b>：上面算出的距離比 Limelight 近超過 {@link #kVisionOverrideMeters}，持續 {@link #kVisionOverrideSec}
 *       （兩顆都被擋，例如對手整台車擋在前面）；或兩顆 Luna 都無效 → 用 Limelight。Luna 比 Limelight 遠時不換（比較可能是視覺誤判）。</li>
 *   <li><b>預設</b>：都沒有 → {@link RangeSensorConstants#kFallbackDistanceMeters}。</li>
 * </ol>
 * 單邊＋IMU 只在 GRID_WALL 用；LOW_WALL 維持原本的單邊公式。
 */
public final class RangeFusion {
  public enum Source {
    LUNA_DUAL("Luna×2"), LUNA_SINGLE_IMU("Luna單邊+IMU"), VISION("Limelight"), FALLBACK("預設");

    public final String label;

    Source(String label) {
      this.label = label;
    }
  }

  /** @param yawRad 相對牆面的偏角（0 = 正對）；不知道 = NaN。 */
  public record Result(double targetDistance, double wallDistance, double yawRad, Source source) {}

  /** Luna 偏角和 IMU 偏角差超過這個 → 判斷有一顆被擋。 */
  static final double kConsistencyRad = Math.toRadians(4);
  /** 還沒有牆面方向基準時，只在 Luna 偏角小於這個時才建立基準（被擋的那顆通常會讓偏角很誇張）。 */
  static final double kLearnInitialRad = Math.toRadians(10);
  /** 有基準之後，Luna 偏角在這個範圍內才拿來更新基準。 */
  static final double kLearnMaxRad = Math.toRadians(25);
  /** 基準更新的平滑係數（每週期）。 */
  static final double kLearnAlpha = 0.05;
  /** Luna 比 Limelight 近超過這個值 → 懷疑兩顆都被擋。 */
  static final double kVisionOverrideMeters = 0.3;
  /** 要持續多久才換成 Limelight（避免來回跳）。 */
  static final double kVisionOverrideSec = 0.2;

  private final RangeReference reference;
  private double wallHeading = Double.NaN;
  private double disagreeSince = Double.NaN;

  public RangeFusion() {
    this(kRangeReference);
  }

  public RangeFusion(RangeReference reference) {
    this.reference = reference;
  }

  /** 牆面方向基準（陀螺儀角度，rad）；還沒建立 = NaN。 */
  public double getWallHeading() {
    return wallHeading;
  }

  /**
   * @param left,right    濾波後的 Luna 原始讀值
   * @param gyroYawRad    陀螺儀累積偏航（不會被 setPose 重設）；沒有 IMU = NaN
   * @param vision        Limelight 估的 TargetDistance（已經過穩定判斷）
   */
  public Result update(double nowSec, double left, double right, boolean leftValid, boolean rightValid,
      double gyroYawRad, OptionalDouble vision) {
    double imuYaw = Double.isNaN(wallHeading) || Double.isNaN(gyroYawRad)
        ? Double.NaN : MathUtil.angleModulus(gyroYawRad - wallHeading);
    Result luna = fromLuna(left, right, leftValid, rightValid, gyroYawRad, imuYaw);

    if (luna == null) {
      disagreeSince = Double.NaN;
      if (vision.isPresent()) {
        return new Result(vision.getAsDouble(), vision.getAsDouble() - kShooterInboardMeters, imuYaw, Source.VISION);
      }
      return new Result(kFallbackDistanceMeters, Double.NaN, imuYaw, Source.FALLBACK);
    }

    if (vision.isPresent() && luna.targetDistance() < vision.getAsDouble() - kVisionOverrideMeters) {
      if (Double.isNaN(disagreeSince)) disagreeSince = nowSec;
      if (nowSec - disagreeSince >= kVisionOverrideSec) {
        return new Result(vision.getAsDouble(), vision.getAsDouble() - kShooterInboardMeters, luna.yawRad(), Source.VISION);
      }
    } else {
      disagreeSince = Double.NaN;
    }
    return luna;
  }

  /** 只看 Luna（＋IMU）。兩顆都無效 → null。 */
  private Result fromLuna(double left, double right, boolean leftValid, boolean rightValid,
      double gyroYawRad, double imuYaw) {
    if (!leftValid && !rightValid) return null;
    boolean grid = reference == RangeReference.GRID_WALL;
    if (leftValid && rightValid) {
      Solution dual = DualTFLunaRangeSensor.solve(reference, left, right, true, true);
      boolean consistent = Double.isNaN(imuYaw) || Math.abs(MathUtil.angleModulus(dual.yawRad() - imuYaw)) < kConsistencyRad;
      if (consistent || !grid) {
        learn(dual.yawRad(), gyroYawRad);
        return new Result(dual.targetDistance(), dual.wallDistance(), dual.yawRad(), Source.LUNA_DUAL);
      }
      // 不一致：一顆被擋。被擋的那顆一定讀得比較近 → 用比較遠的那顆
      double dL = left - kLeftLunaInboardMeters - kLunaTrimMeters;
      double dR = right - kRightLunaInboardMeters;
      return single(Math.max(dL, dR), imuYaw);
    }
    if (!grid) {
      Solution s = DualTFLunaRangeSensor.solve(reference, left, right, leftValid, rightValid);
      return new Result(s.targetDistance(), s.wallDistance(), imuYaw, Source.LUNA_SINGLE_IMU);
    }
    double dFrame = leftValid ? left - kLeftLunaInboardMeters - kLunaTrimMeters : right - kRightLunaInboardMeters;
    return single(dFrame, imuYaw);
  }

  private static Result single(double dFrame, double imuYaw) {
    double wall = dFrame * Math.cos(Double.isNaN(imuYaw) ? 0 : imuYaw);
    return new Result(wall + kShooterInboardMeters, wall, imuYaw, Source.LUNA_SINGLE_IMU);
  }

  private void learn(double lunaYaw, double gyroYawRad) {
    if (Double.isNaN(gyroYawRad) || Double.isNaN(lunaYaw)) return;
    double candidate = gyroYawRad - lunaYaw;
    if (Double.isNaN(wallHeading)) {
      if (Math.abs(lunaYaw) < kLearnInitialRad) wallHeading = candidate;
    } else if (Math.abs(lunaYaw) < kLearnMaxRad) {
      wallHeading += kLearnAlpha * MathUtil.angleModulus(candidate - wallHeading);
    }
  }
}
