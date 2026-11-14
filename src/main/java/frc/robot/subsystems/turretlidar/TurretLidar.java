package frc.robot.subsystems.turretlidar;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

import frc.FSLib.rangefinder.RangeSensorConstants;

/**
 * 砲台正前方（水平）的第三顆 TF-Luna，接在 Jetson 的 I2C，經 NetworkTables 傳過來。
 *
 * <p>目前只讀值、記 log、給 Elastic 顯示連線狀態，還沒有任何指令拿它來控制。
 * Manual 射擊時每發會連同這顆的距離一起印出（ShotTable 樣本），之後可以決定要不要改用它當查表距離。
 */
public class TurretLidar extends SubsystemBase {
  /** 超過這麼久沒收到新讀值就算斷線（Jetson 約 100 Hz 發布）。 */
  public static final double kTimeoutSeconds = 0.25;

  private final TurretLidarIO io;
  private final TurretLidarIOInputsAutoLogged inputs = new TurretLidarIOInputsAutoLogged();

  public TurretLidar(TurretLidarIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TurretLidar", inputs);
    Logger.recordOutput("TurretLidar/Valid", isValid());
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  /** 有連線且讀值可信（範圍與訊號強度門檻和左右兩顆 TF-Luna 相同）。 */
  public boolean isValid() {
    return inputs.connected
        && inputs.distanceMeters >= RangeSensorConstants.kMinValidRangeMeters
        && inputs.distanceMeters <= RangeSensorConstants.kMaxValidRangeMeters
        && inputs.strength >= RangeSensorConstants.kMinStrength
        && inputs.strength != RangeSensorConstants.kSaturatedStrength;
  }

  /** 原始距離（公尺）；讀值不可信時 empty。 */
  public OptionalDouble getDistanceMeters() {
    return isValid() ? OptionalDouble.of(inputs.distanceMeters) : OptionalDouble.empty();
  }
}
