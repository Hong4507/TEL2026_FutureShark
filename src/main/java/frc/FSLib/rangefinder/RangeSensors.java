package frc.FSLib.rangefinder;

import frc.robot.Constants;

/** 依 {@link Constants#kRangeSensorType} 提供共用的感測器單例。 */
public final class RangeSensors {
  private static TargetRangeSensor instance;

  private RangeSensors() {}

  public static synchronized TargetRangeSensor get() {
    if (instance == null) {
      instance = switch (Constants.kRangeSensorType) {
        case kRPLidar -> new RPLidarRangeSensor(RangeSensorConstants.kRPLidarTable);
        case kDualTFLuna -> new DualTFLunaRangeSensor();
      };
    }
    return instance;
  }

  /**
   * 換成融合版（車身 Luna ＋ IMU ＋ Limelight，見 {@link RangeFusion}）。要在建立任何 command 之前呼叫（RobotContainer 建完子系統後）。
   * 不是兩顆 TF-Luna 時不做事。
   */
  public static synchronized void installFusion(java.util.function.DoubleSupplier gyroYawRad,
      java.util.function.Supplier<java.util.OptionalDouble> visionDistance) {
    if (get() instanceof DualTFLunaRangeSensor luna) {
      instance = new FusedRangeSensor(luna, gyroYawRad, visionDistance);
    }
  }

  /** 測試用：換成假的感測器（例如依模擬位置算距離）。傳 null 恢復預設。要在建立 command 之前呼叫。 */
  public static synchronized void setForTesting(TargetRangeSensor sensor) {
    instance = sensor;
  }
}
