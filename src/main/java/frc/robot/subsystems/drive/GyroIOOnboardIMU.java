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
    // 內建 IMU 永遠存在
    inputs.connected = true;
    // WPILib 慣例：逆時針為正（NavX 是順時針為正，所以舊程式要取負號）
    inputs.yawRad = imu.getYawRadians();
    inputs.yawVelocityRadPerSec = imu.getGyroRateZ();
  }
}
