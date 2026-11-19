package frc.robot.subsystems.drive;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;

/**
 * 使用 SystemCore 內建 IMU 的陀螺儀 IO。
 *
 * <p>SystemCore 沒有 MXP 擴充埠，原本插在 roboRIO MXP 上的 NavX 無法使用，因此改用內建 IMU。
 * 若 SystemCore 不是平放安裝，請把 {@link MountOrientation} 改成 LANDSCAPE 或 PORTRAIT。
 */
public class GyroIOOnboardIMU implements GyroIO {
  private final OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // WPILib 慣例：逆時針為正（NavX 是順時針為正，所以舊程式要取負號）
    double yaw = imu.getYawRadians();
    // 內建 IMU 不會「拔掉」，但讀值變 NaN／無限大就當作壞了（Elastic IMU 燈、READY 燈才看得到）
    inputs.connected = Double.isFinite(yaw);
    inputs.yawRad = inputs.connected ? yaw : 0.0;
    // 11/17 確認 SystemCore 平放（FLAT）：感測器 Z 軸 = 車身偏航軸。直立安裝時這行要改用對應的軸
    inputs.yawVelocityRadPerSec = imu.getGyroRateZ();
  }

  @Override
  public void resetYaw() {
    imu.resetYaw();
  }
}
