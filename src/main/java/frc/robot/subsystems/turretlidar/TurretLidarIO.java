package frc.robot.subsystems.turretlidar;

import org.littletonrobotics.junction.AutoLog;

public interface TurretLidarIO {
  @AutoLog
  public static class TurretLidarIOInputs {
    /** Jetson 在 {@link TurretLidar#kTimeoutSeconds} 內有送新讀值。 */
    public boolean connected = false;
    /** 距離（公尺），TF-Luna 原始讀值，沒加任何 offset。沒資料 = NaN。 */
    public double distanceMeters = Double.NaN;
    /** 訊號強度（&lt; 100 不可靠，65535 = 過曝）。 */
    public double strength = 0.0;
    public double temperatureC = Double.NaN;
    public long heartbeat = 0;
    /** Jetson 端連續 I2C 讀取失敗次數（0 = 正常）。 */
    public long i2cErrors = 0;
  }

  public default void updateInputs(TurretLidarIOInputs inputs) {}
}
