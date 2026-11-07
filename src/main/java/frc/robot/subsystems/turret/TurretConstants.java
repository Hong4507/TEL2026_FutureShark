package frc.robot.subsystems.turret;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public class TurretConstants {
  public static final int kUpShooterMotorID = 13;
  public static final int kDownShooterMotorID = 11;
  public static final int kAngleMotorID = 32;
  public static final int kTriggerMotorID = 12;
  public static final int kEncoderID = 15;

  public static final double kMaxAngleRad = 1.45;
  public static final double kMinAngleRad = 0.6981317;

  public static final double kSensorToMechanismRatio = 6.875;

  public static final double kMaxSpeedRadPerSec = 565;

  public static final Translation3d kTurretToRobotCenter = new Translation3d(0.125, -0.04, 0.52);
  public static final Transform3d kTurretToRobotCenterTransform = new Transform3d(kTurretToRobotCenter, new Rotation3d(0, 0, -Math.PI / 2));

  // far 1 block end
  // target = 5,  pitch: 1.3461316999999622
  // target = 8, pitch: 1.4
  // target = 0, pitch: 1.1641316999999822
  // target = 2, 
}