package frc.FSLib.rangefinder;

import static org.wpilib.units.Units.Radians;
import static frc.FSLib.rangefinder.RangeSensorConstants.*;

import frc.FSLib.rangefinder.RangeSensorConstants.RangeReference;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import org.wpilib.hardware.bus.I2C;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;

/**
 * 兩顆 TF-Luna 沿車身前後裝（左 Luna 靠車頭 {@code kBaselineMeters}），光束朝機器人右側（和射球同方向），
 * 直接接在 SystemCore 的 I2C 埠。量哪一面牆由 {@link RangeSensorConstants#kRangeReference} 決定（§8.1／§8.2）。
 *
 * <p>TF-Luna 必須是 I2C 模式：第 5 腳（CFG）接 GND 後上電。預設位址 0x10。
 * 兩顆接在不同 I2C 埠時可以都用 0x10；接在同一個埠時，必須先把其中一顆改成別的位址
 * （寫暫存器 0x22 = 新位址，再寫 0x20 = 0x01 存檔，重新上電）。
 *
 * <p>暫存器（little-endian）：0x00~0x01 距離（cm）、0x02~0x03 訊號強度、0x04~0x05 溫度（0.01°C）。
 */
public class DualTFLunaRangeSensor implements TargetRangeSensor {
  private static final int kDistLowRegister = 0x00;
  private static final int kReadLength = 6;
  /** 同一個控制週期內重複呼叫時，直接沿用上一次讀到的值，避免 I2C 被重複讀取。 */
  private static final double kCacheSeconds = 0.005;

  private final TFLuna left;
  private final TFLuna right;

  private Reading cached;
  private double cachedTime = Double.NEGATIVE_INFINITY;
  private Solution solution = new Solution(kFallbackDistanceMeters, Double.NaN, Double.NaN, false);

  private final TimedMovingAverage leftFilter = new TimedMovingAverage(kLunaFilterSec);
  private final TimedMovingAverage rightFilter = new TimedMovingAverage(kLunaFilterSec);

  public DualTFLunaRangeSensor(I2C.Port leftPort, int leftAddress, I2C.Port rightPort, int rightAddress) {
    if (leftPort == rightPort && leftAddress == rightAddress) {
      throw new IllegalArgumentException(
          "兩顆 TF-Luna 在同一個 I2C 埠且位址相同（0x" + Integer.toHexString(leftAddress)
              + "），請改接不同埠或修改其中一顆的位址");
    }
    left = new TFLuna("Left", leftPort, leftAddress);
    right = new TFLuna("Right", rightPort, rightAddress);
  }

  public DualTFLunaRangeSensor() {
    this(kLeftTFLunaPort, kLeftTFLunaAddress, kRightTFLunaPort, kRightTFLunaAddress);
  }

  /** 單顆 TF-Luna 的 I2C 讀取。 */
  private static final class TFLuna {
    private final String name;
    private final I2C i2c;
    private final byte[] buffer = new byte[kReadLength];

    double distanceMeters = Double.NaN;
    double strength = 0;
    double temperatureC = Double.NaN;
    boolean connected = false;

    TFLuna(String name, I2C.Port port, int address) {
      this.name = name;
      this.i2c = new I2C(port, address);
    }

    void update() {
      // read() 回傳 true 代表傳輸失敗（沒接、位址錯、線斷了）
      connected = !i2c.read(kDistLowRegister, kReadLength, buffer);
      if (connected) {
        distanceMeters = u16(0) / 100.0;
        strength = u16(2);
        temperatureC = u16(4) / 100.0;
      } else {
        distanceMeters = Double.NaN;
        strength = 0;
        temperatureC = Double.NaN;
      }
      String prefix = "RangeSensor/TFLuna/" + name;
      Logger.recordOutput(prefix + "Connected", connected);
      Logger.recordOutput(prefix + "Meters", distanceMeters);
      Logger.recordOutput(prefix + "Strength", strength);
      Logger.recordOutput(prefix + "TemperatureC", temperatureC);
    }

    boolean isValid() {
      return connected
          && distanceMeters >= kMinValidRangeMeters
          && distanceMeters <= kMaxValidRangeMeters
          && strength >= kMinStrength
          && strength != kSaturatedStrength;
    }

    private int u16(int offset) {
      return (buffer[offset] & 0xFF) | ((buffer[offset + 1] & 0xFF) << 8);
    }
  }

  /** 一次讀取的結果（距離已濾波）。 */
  private record Reading(double left, double right, boolean leftValid, boolean rightValid) {}

  /** 公式計算結果。yawRad 是相對正對牆面的偏差（0 = 正對，已乘 kYawSign）；單邊或都無效時 NaN。 */
  public record Solution(double targetDistance, double wallDistance, double yawRad, boolean valid) {}

  /** 依 {@link RangeSensorConstants#kRangeReference} 算距離與角度。純計算，單元測試用這個。 */
  public static Solution solve(double leftMeters, double rightMeters, boolean leftValid, boolean rightValid) {
    return solve(kRangeReference, leftMeters, rightMeters, leftValid, rightValid);
  }

  public static Solution solve(RangeReference reference, double leftMeters, double rightMeters,
      boolean leftValid, boolean rightValid) {
    return reference == RangeReference.LOW_WALL
        ? solveLowWall(leftMeters, rightMeters, leftValid, rightValid)
        : solveGridWall(leftMeters, rightMeters, leftValid, rightValid);
  }

  /**
   * GRID_WALL（Robot-Brief §8.1）：
   * <pre>
   * dR_frame  = dR − kRightLunaInboardMeters
   * dL_frame  = dL − kLeftLunaInboardMeters − kLunaTrimMeters
   * yaw       = atan2(dL_frame − dR_frame, kBaselineMeters) × kYawSign
   * wall_dist = (dL_frame + dR_frame) / 2 × cos(yaw)        // 右 frame 到九宮格牆
   * TargetDistance = wall_dist + kShooterInboardMeters
   * </pre>
   * 只有一邊有效：wall_dist = 那一邊的 d_frame（不算角度）。都無效：kFallbackDistanceMeters。
   */
  static Solution solveGridWall(double dL, double dR, boolean leftValid, boolean rightValid) {
    double dLFrame = dL - kLeftLunaInboardMeters - kLunaTrimMeters;
    double dRFrame = dR - kRightLunaInboardMeters;
    if (leftValid && rightValid) {
      double yaw = Math.atan2(dLFrame - dRFrame, kBaselineMeters) * kYawSign;
      double wall = (dLFrame + dRFrame) / 2 * Math.cos(yaw);
      return new Solution(wall + kShooterInboardMeters, wall, yaw, true);
    }
    if (leftValid) return new Solution(dLFrame + kShooterInboardMeters, dLFrame, Double.NaN, true);
    if (rightValid) return new Solution(dRFrame + kShooterInboardMeters, dRFrame, Double.NaN, true);
    return new Solution(kFallbackDistanceMeters, Double.NaN, Double.NaN, false);
  }

  /**
   * LOW_WALL（Robot-Brief §8.2 備案）：舊公式。kLowWall* 用預設值時和 11/14 版本算出來完全一樣。
   * wallDistance 回傳「右 frame 到九宮格牆」= TargetDistance − kSensorOffsetMeters（給砲台 Luna 用）。
   */
  static Solution solveLowWall(double l, double r, boolean leftValid, boolean rightValid) {
    double left = l - kLowWallLeftInboardMeters;
    double right = r - kLowWallRightInboardMeters;
    if (leftValid && rightValid) {
      double tilt = Math.atan2(right - left, kLowWallBaselineMeters);
      double perpendicular = (left + right) / 2 * Math.cos(tilt);
      double distance = perpendicular + kWallToTargetMeters / Math.cos(tilt) + kSensorOffsetMeters;
      return new Solution(distance, distance - kSensorOffsetMeters, tilt * kLowWallYawSign, true);
    }
    if (leftValid || rightValid) {
      double distance = (leftValid ? left : right) + kWallToTargetMeters + kSensorOffsetMeters;
      return new Solution(distance, distance - kSensorOffsetMeters, Double.NaN, true);
    }
    return new Solution(kFallbackDistanceMeters, Double.NaN, Double.NaN, false);
  }

  private Reading read() {
    double now = Timer.getTimestamp();
    if (cached != null && now - cachedTime < kCacheSeconds) {
      return cached;
    }
    left.update();
    right.update();
    boolean lValid = left.isValid();
    boolean rValid = right.isValid();
    double l = leftFilter.calculate(now, left.distanceMeters, lValid);
    double r = rightFilter.calculate(now, right.distanceMeters, rValid);
    Logger.recordOutput("RangeSensor/TFLuna/LeftValid", lValid);
    Logger.recordOutput("RangeSensor/TFLuna/RightValid", rValid);
    Logger.recordOutput("RangeSensor/TFLuna/LeftFilteredMeters", l);
    Logger.recordOutput("RangeSensor/TFLuna/RightFilteredMeters", r);
    // §8.2：開機第一件事看這兩個（黑板 Amp 太低 → 讀值 0 → 切 LOW_WALL）
    Logger.recordOutput("RangeSensor/LeftAmp", left.strength);
    Logger.recordOutput("RangeSensor/RightAmp", right.strength);
    // 擺正時這個值 = kLunaTrimMeters 要填的數字
    Logger.recordOutput("RangeSensor/RawFrameDiffMeters",
        (l - kLeftLunaInboardMeters) - (r - kRightLunaInboardMeters));

    cached = new Reading(l, r, lValid, rValid);
    cachedTime = now;
    solution = solve(l, r, lValid, rValid);
    Logger.recordOutput("RangeSensor/Reference", kRangeReference.name());
    Logger.recordOutput("RangeSensor/TargetDistance", solution.targetDistance());
    Logger.recordOutput("RangeSensor/WallDistance", solution.wallDistance());
    Logger.recordOutput("RangeSensor/YawErrorDeg", Math.toDegrees(solution.yawRad()));
    return cached;
  }

  /** 最近一次讀取時左邊那顆有回應（I2C 傳輸成功）。 */
  public boolean isLeftConnected() {
    read();
    return left.connected;
  }

  public boolean isRightConnected() {
    read();
    return right.connected;
  }

  @Override
  public double getTargetDistance() {
    read();
    return solution.targetDistance();
  }

  @Override
  public OptionalDouble getWallDistanceMeters() {
    read();
    return solution.valid() ? OptionalDouble.of(solution.wallDistance()) : OptionalDouble.empty();
  }

  /**
   * 左右兩顆的原始距離 → 偏航角（rad，π/2 = 正對牆面），依目前的 kRangeReference。純計算，模擬測試也用這個。
   */
  public static double yawRad(double leftMeters, double rightMeters) {
    return yawRad(kRangeReference, leftMeters, rightMeters);
  }

  public static double yawRad(RangeReference reference, double leftMeters, double rightMeters) {
    return Math.PI / 2 + solve(reference, leftMeters, rightMeters, true, true).yawRad();
  }

  @Override
  public Angle getYaw() {
    read();
    if (Double.isNaN(solution.yawRad())) {
      return null;
    }
    double yaw = Math.PI / 2 + solution.yawRad();
    Logger.recordOutput("RangeSensor/YawRad", yaw);
    return Radians.of(yaw);
  }
}
