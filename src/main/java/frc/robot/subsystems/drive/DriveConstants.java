package frc.robot.subsystems.drive;

import org.wpilib.math.util.Units;

public class DriveConstants {
  // Hardward CAN IDs
  public static final int kFrontLeftMotorID = 14;
  public static final int kBackLeftMotorID = 22;
  public static final int kFrontRightMotorID = 27;
  public static final int kBackRightMotorID = 1;

  // Chassis mechanical constants
  public static final double kWheelRadius = Units.inchesToMeters(2);
  public static final double kWheelParameter = 2 * Math.PI * kWheelRadius;
  public static final double kTrackWidth = 0.43955208;
  public static final double kGearRatio = 6;
  public static final double kPositionConversionFactor = 1 / kGearRatio * kWheelParameter;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

  public static final double kSize = 495.3;

  public static final double kMaxSpeedMetersPerSec = 3.0;

  // Control system constants
  public static final double kP = 0.2;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0;
  public static final double kV = 2.283;
  public static final double kA = 0.5;

  // Control system constants (simulation)
  public static final double kSimkP = 2.0;
  public static final double kSimkI = 0.0;
  public static final double kSimkD = 0.0;

  public static final double kSimkS = 0.0;
  public static final double kSimkV = 2.3;
  public static final double kSimkA = 0.24;
}

