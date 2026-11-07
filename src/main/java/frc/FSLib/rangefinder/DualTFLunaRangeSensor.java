package frc.FSLib.rangefinder;

import static org.wpilib.units.Units.Radians;
import static frc.FSLib.rangefinder.RangeSensorConstants.*;

import org.littletonrobotics.junction.Logger;

import org.wpilib.hardware.bus.I2C;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;

/**
 * 兩顆 TF-Luna 左右並排、朝正前方，直接接在 SystemCore 的 I2C 埠。
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

  /** 一次讀取的結果。 */
  private record Reading(double left, double right, boolean leftValid, boolean rightValid) {
    boolean bothValid() {
      return leftValid && rightValid;
    }

    /** 相對牆面的傾角（rad），右邊比較遠為正。 */
    double tiltRad() {
      return Math.atan2(right - left, kBaselineMeters);
    }
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
    Logger.recordOutput("RangeSensor/TFLuna/LeftValid", lValid);
    Logger.recordOutput("RangeSensor/TFLuna/RightValid", rValid);

    cached = new Reading(left.distanceMeters, right.distanceMeters, lValid, rValid);
    cachedTime = now;
    return cached;
  }

  @Override
  public double getTargetDistance() {
    Reading reading = read();
    double distance;
    if (reading.bothValid()) {
      double tilt = reading.tiltRad();
      double perpendicular = (reading.left() + reading.right()) / 2 * Math.cos(tilt);
      distance = perpendicular + kWallToTargetMeters / Math.cos(tilt) + kSensorOffsetMeters;
      Logger.recordOutput("RangeSensor/TFLuna/TiltRad", tilt);
    } else if (reading.leftValid()) {
      distance = reading.left() + kWallToTargetMeters + kSensorOffsetMeters;
    } else if (reading.rightValid()) {
      distance = reading.right() + kWallToTargetMeters + kSensorOffsetMeters;
    } else {
      distance = kFallbackDistanceMeters;
    }
    Logger.recordOutput("RangeSensor/TargetDistance", distance);
    return distance;
  }

  @Override
  public Angle getYaw() {
    Reading reading = read();
    if (!reading.bothValid()) {
      return null;
    }
    double yaw = Math.PI / 2 + kYawSign * reading.tiltRad();
    Logger.recordOutput("RangeSensor/YawRad", yaw);
    return Radians.of(yaw);
  }
}
