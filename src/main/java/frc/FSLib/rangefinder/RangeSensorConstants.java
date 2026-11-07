package frc.FSLib.rangefinder;

import org.wpilib.hardware.bus.I2C;

public final class RangeSensorConstants {
  private RangeSensorConstants() {}

  public static final String kRPLidarTable = "LaserScan";

  /**
   * 兩顆 TF-Luna 的 I2C 埠與位址（TF-Luna 預設 0x10）。
   * 預設一顆接 I2C 0、一顆接 I2C 1，兩顆都用出廠位址，不用改設定。
   * 若兩顆接同一個埠，必須先把其中一顆改成不同位址（例如 0x11），再改這裡。
   */
  public static final I2C.Port kLeftTFLunaPort = I2C.Port.PORT_0;
  public static final int kLeftTFLunaAddress = 0x10;
  public static final I2C.Port kRightTFLunaPort = I2C.Port.PORT_1;
  public static final int kRightTFLunaAddress = 0x10;

  /** 兩顆 TF-Luna 中心到中心的距離（公尺）。TODO: 填入實際量測值。 */
  public static final double kBaselineMeters = 0.30;

  /** TF-Luna 有效量測範圍（公尺）。 */
  public static final double kMinValidRangeMeters = 0.2;
  public static final double kMaxValidRangeMeters = 8.0;

  /** 訊號強度低於此值或等於 65535（過曝）時視為無效。 */
  public static final double kMinStrength = 100;
  public static final double kSaturatedStrength = 65535;

  /** 牆面到目標的距離（公尺），沿用 RPLidar 公式的初值，需實測校正。 */
  public static final double kWallToTargetMeters = 2.5;
  /** 感測器到量測基準點的 offset（公尺），沿用 RPLidar 公式的初值，需實測校正。 */
  public static final double kSensorOffsetMeters = 0.04;

  /** 偏航角方向。若 LaserAlign 越轉越歪，把它改成 -1。 */
  public static final double kYawSign = 1.0;

  /** 完全沒有有效資料時的距離（公尺），與 RPLidar 實作一致。 */
  public static final double kFallbackDistanceMeters = 3.0;
}
