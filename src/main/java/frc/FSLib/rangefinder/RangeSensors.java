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
}
