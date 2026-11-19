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
 *
 * <p>11/19 修正（實機：單邊+IMU 距離怪、兩顆都在範圍內卻常被切走）：
 * <ul>
 *   <li>一致門檻 4° → {@link #kConsistencyRad}（基線 17 cm，讀值差 1 cm ≈ 3.4°，雜訊就會超過 4°）。</li>
 *   <li>不一致要持續 {@link #kBlockedConfirmSec} 才換單邊（避免一跳就切）。</li>
 *   <li>舊版只在「一致」時更新牆面基準 → 基準一歪（IMU 漂移、在別面牆學到、第一筆雜訊）就永遠不一致、永遠卡在單邊。
 *       現在：兩顆 Luna 偏角很小（看起來就是一面平牆、沒被擋）但和 IMU 一直不一致超過 {@link #kRebaseSec} → 直接重設基準。</li>
 *   <li>單邊公式補上 Luna 沿牆偏移：牆距 = d_frame × cos θ ∓ (B/2) sin θ（原本少這項，偏 15° 差約 2 cm）。</li>
 *   <li>換 Limelight 的門檻 0.3 → {@link #kVisionOverrideMeters} m（視覺 WidthScale 還沒校正，誤差 10% 就會誤切）。</li>
 * </ul>
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

  /** Luna 偏角和 IMU 偏角差超過這個 → 判斷有一顆被擋。被擋時差距通常很大（擋 10 cm ≈ 30°）。 */
  static final double kConsistencyRad = Math.toRadians(10);
  /** 不一致要持續多久才真的換單邊。 */
  static final double kBlockedConfirmSec = 0.15;
  /** 兩顆 Luna 偏角小於 {@link #kLearnInitialRad} 卻和 IMU 一直不一致這麼久 → 判斷是 IMU 基準歪了，重設。 */
  static final double kRebaseSec = 1.0;
  /** 還沒有牆面方向基準時，只在 Luna 偏角小於這個時才建立基準（被擋的那顆通常會讓偏角很誇張）。 */
  static final double kLearnInitialRad = Math.toRadians(10);
  /**
   * 11/19：IMU 說車身相對九宮格牆偏超過這個 → 車身 Luna 根本沒對著九宮格（例如開機時車頭朝牆、Luna 打到側邊），
   * 不拿 Luna 學習／重設基準，也不用單邊+IMU（cos 接近 0 會算出亂的距離）。
   */
  static final double kMaxImuYawRad = Math.toRadians(45);
  /** 有基準之後，Luna 偏角在這個範圍內才拿來更新基準。 */
  static final double kLearnMaxRad = Math.toRadians(25);
  /** 基準更新的平滑係數（每週期）。 */
  static final double kLearnAlpha = 0.05;
  /** Luna 比 Limelight 近超過這個值 → 懷疑兩顆都被擋。 */
  static final double kVisionOverrideMeters = 0.5;
  /** 要持續多久才換成 Limelight（避免來回跳）。 */
  static final double kVisionOverrideSec = 0.2;

  private final RangeReference reference;
  private double wallHeading = Double.NaN;
  private double disagreeSince = Double.NaN;
  private double inconsistentSince = Double.NaN;
  private double lastLunaImuDiffRad = Double.NaN;
  private int wallHeadingResets = 0;

  public RangeFusion() {
    this(kRangeReference);
  }

  public RangeFusion(RangeReference reference) {
    this.reference = reference;
  }

  /** 最近一次兩顆 Luna 偏角 − IMU 偏角（rad）；沒有兩顆或沒有基準 = NaN。超過 {@link #kConsistencyRad} 就算不一致。 */
  public double getLunaImuDiffRad() {
    return lastLunaImuDiffRad;
  }

  /** 牆面基準被重設（判斷 IMU 基準歪掉）的次數。 */
  public int getWallHeadingResets() {
    return wallHeadingResets;
  }

  /** 牆面方向基準（陀螺儀角度，rad）；還沒建立 = NaN。 */
  public double getWallHeading() {
    return wallHeading;
  }

  /**
   * 11/19：開機時兩顆 Luna 不一定打得到牆 → 用「已知的擺放角度」直接設定牆面基準（例如自動開始時車子擺在起點）。
   * 之後 Luna×2 一致時照常慢慢修正；差太多時照常由 {@link #kRebaseSec} 重設。
   *
   * @param gyroYawRad   現在的陀螺儀累積偏航；NaN = 不做事
   * @param yawToWallRad 車身現在相對牆面的偏角（0 = 正對，方向同 {@code RangeSensor/ImuYawErrorDeg}）
   */
  public void seedWallHeading(double gyroYawRad, double yawToWallRad) {
    if (Double.isNaN(gyroYawRad) || Double.isNaN(yawToWallRad)) return;
    wallHeading = gyroYawRad - yawToWallRad;
    inconsistentSince = Double.NaN;
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
    Result luna = fromLuna(nowSec, left, right, leftValid, rightValid, gyroYawRad, imuYaw);

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
  private Result fromLuna(double nowSec, double left, double right, boolean leftValid, boolean rightValid,
      double gyroYawRad, double imuYaw) {
    lastLunaImuDiffRad = Double.NaN;
    if (!leftValid && !rightValid) {
      inconsistentSince = Double.NaN;
      return null;
    }
    boolean grid = reference == RangeReference.GRID_WALL;
    if (!Double.isNaN(imuYaw) && Math.abs(imuYaw) > kMaxImuYawRad) {
      // 沒對著九宮格：只回傳 Luna 自己的公式，不學習、不重設、不用 IMU
      inconsistentSince = Double.NaN;
      Solution s = DualTFLunaRangeSensor.solve(reference, left, right, leftValid, rightValid);
      return new Result(s.targetDistance(), s.wallDistance(), imuYaw,
          leftValid && rightValid ? Source.LUNA_DUAL : Source.LUNA_SINGLE_IMU);
    }
    if (leftValid && rightValid) {
      Solution dual = DualTFLunaRangeSensor.solve(reference, left, right, true, true);
      Result dualResult = new Result(dual.targetDistance(), dual.wallDistance(), dual.yawRad(), Source.LUNA_DUAL);
      if (!Double.isNaN(imuYaw)) lastLunaImuDiffRad = MathUtil.angleModulus(dual.yawRad() - imuYaw);
      boolean consistent = Double.isNaN(lastLunaImuDiffRad) || Math.abs(lastLunaImuDiffRad) < kConsistencyRad;
      if (consistent || !grid) {
        inconsistentSince = Double.NaN;
        learn(dual.yawRad(), gyroYawRad);
        return dualResult;
      }
      if (Double.isNaN(inconsistentSince)) inconsistentSince = nowSec;
      double inconsistentFor = nowSec - inconsistentSince;
      // 兩顆看起來都是同一面平牆（偏角小）卻一直和 IMU 對不上 → IMU 基準歪了，不是被擋
      if (Math.abs(dual.yawRad()) < kLearnInitialRad && inconsistentFor >= kRebaseSec) {
        wallHeading = gyroYawRad - dual.yawRad();
        wallHeadingResets++;
        inconsistentSince = Double.NaN;
        lastLunaImuDiffRad = 0;
        return dualResult;
      }
      if (inconsistentFor < kBlockedConfirmSec) return dualResult;
      // 不一致：一顆被擋。被擋的那顆一定讀得比較近 → 用比較遠的那顆
      double dL = left - kLeftLunaInboardMeters - kLunaTrimMeters;
      double dR = right - kRightLunaInboardMeters;
      return dL >= dR ? single(dL, true, imuYaw) : single(dR, false, imuYaw);
    }
    inconsistentSince = Double.NaN;
    if (!grid) {
      Solution s = DualTFLunaRangeSensor.solve(reference, left, right, leftValid, rightValid);
      return new Result(s.targetDistance(), s.wallDistance(), imuYaw, Source.LUNA_SINGLE_IMU);
    }
    return leftValid
        ? single(left - kLeftLunaInboardMeters - kLunaTrimMeters, true, imuYaw)
        : single(right - kRightLunaInboardMeters, false, imuYaw);
  }

  /**
   * 單邊＋IMU。GRID_WALL 公式反過來：dL_f = 中心距/cos θ + (B/2) tan θ'，dR_f = 中心距/cos θ − (B/2) tan θ'
   * （θ' = θ × kYawSign 是幾何角度）→ 牆距 = d_f cos θ ∓ (B/2) sin θ'。和 Luna×2 算出來的完全一樣。
   */
  private static Result single(double dFrame, boolean isLeft, double imuYaw) {
    double yaw = Double.isNaN(imuYaw) ? 0 : imuYaw;
    double halfOffset = kBaselineMeters / 2 * Math.sin(yaw * kYawSign);
    double wall = dFrame * Math.cos(yaw) + (isLeft ? -halfOffset : halfOffset);
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
