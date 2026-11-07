package frc.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;

public class Constants {
  public static final int kDriveXboxPort = 0;
  public static final double kDefaultPeriod = 0.01;

  /**
   * SystemCore 的 CAN 匯流排。SystemCore 有 5 個 CAN 埠 (CAN_S0 ~ CAN_S4)，
   * 請改成馬達實際接線的那一個。
   */
  public static final CANPort kCANPort = CANPort.CAN_S0;

  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.kReal : Mode.kSim;
  public static enum Mode {
    kReal,
    kSim,
    kReplay;
  }
  
  /** 距離／偏航感測器來源：RPLidar A1 或兩顆 TF-Luna。 */
  public static final RangeSensorType kRangeSensorType = RangeSensorType.kDualTFLuna;
  public static enum RangeSensorType {
    kRPLidar,
    kDualTFLuna
  }

  public static RobotMode kRobotMode = RobotMode.kDisable;
  public static enum RobotMode {
    kManual,
    kAssist,
    kAuto,
    kDisable
  }

  public static final class FieldConstants {
    public static final double kTargetX = 7.5;
    public static final double kLeftTargetY = 2.9;
    public static final double kMidTargetY = 2.0;
    public static final double kRightTargetY = 1.1;
    public static final double kLowTargetHeight = 1.2;
    public static final double kMidTargetHeight = 1.8;
    public static final double kHighTargetHeight = 2.4;
    public static final double kSpecialTargetHeight = 2.9;
  }

}