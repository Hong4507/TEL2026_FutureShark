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

  /**
   * CANcoder 磁鐵偏移（圈），讓「砲台在最低點時」絕對位置剛好 = 0。TODO: 上機校正後填入。
   *
   * <p>校正步驟：
   * <ol>
   *   <li>這裡先填 0，部署程式。
   *   <li>把砲台放到最低點（靠在下限位）。
   *   <li>在 AdvantageScope 看 {@code Turret/PitchEncoderAbsRot} 的值，假設是 0.37。
   *   <li>這裡填它的負值 -0.37，重新部署。之後在最低點時 PitchEncoderAbsRot 應該接近 0。
   * </ol>
   * 範圍 -1 ~ 1。
   */
  public static final double kEncoderMagnetOffsetRot = 0.0;

  /**
   * 絕對位置的跳變點（圈）。讀值範圍是 [kEncoderDiscontinuityPointRot - 1, kEncoderDiscontinuityPointRot)。
   * 最低點 = 0、最高點約 0.82 圈（(1.45 - 0.698) × 6.875 / 2π），設 0.9 讓整個行程都不會跨過跳變點，
   * 稍微低於最低點時會讀到小負值，而不是突然變成 0.9。
   */
  public static final double kEncoderDiscontinuityPointRot = 0.9;

  public static final double kMaxSpeedRadPerSec = 565;

  public static final Translation3d kTurretToRobotCenter = new Translation3d(0.125, -0.04, 0.52);
  public static final Transform3d kTurretToRobotCenterTransform = new Transform3d(kTurretToRobotCenter, new Rotation3d(0, 0, -Math.PI / 2));

  // far 1 block end
  // target = 5,  pitch: 1.3461316999999622
  // target = 8, pitch: 1.4
  // target = 0, pitch: 1.1641316999999822
  // target = 2, 
}