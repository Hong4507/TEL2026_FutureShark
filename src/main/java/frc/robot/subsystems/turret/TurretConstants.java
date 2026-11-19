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

  public static final double kSensorToMechanismRatio = 6.875;

  /**
   * 砲台仰角上下限（rad）。setPitch／ShotTable／彈道公式／ManualTurret 全部用這個單位。
   *
   * <p>仰角 = kMinAngleRad + CANcoder 圈數 × 2π ÷ kSensorToMechanismRatio。開機時 CANcoder 設成 0，
   * 所以**開機前砲台要靠在下限位**（= kMinAngleRad = 40°）；上限位時 CANcoder 約 0.82 圈。
   */
  public static final double kMaxAngleRad = 1.45;
  public static final double kMinAngleRad = 0.6981317;

  /** 仰角馬達最大電壓：限制 PID 輸出，避免一切換模式就全速衝。 */
  public static final double kPitchMaxVolts = 6.0;

  /** 仰角最快移動速度（rad/s）與加速度（rad/s²）。太慢就調高，還會衝過頭就調低。 */
  public static final double kPitchMaxVelRadPerSec = 2.0;
  public static final double kPitchMaxAccelRadPerSecSq = 6.0;

  /**
   * 目前仰角用 CANcoder **相對位置**（開機時 CANcoder = 0 = kMinAngleRad，開機前砲台要靠在下限位），
   * 下面兩個常數只影響記錄用的 Turret/PitchEncoderAbsRot；保留給之後改回絕對位置用。
   *
   * <p>CANcoder 磁鐵偏移（圈），讓「砲台在最低點時」絕對位置剛好 = 0。
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
  public static final double kEncoderMagnetOffsetRot = 0.230469;

  /**
   * 絕對位置的跳變點（圈）。讀值範圍是 [kEncoderDiscontinuityPointRot - 1, kEncoderDiscontinuityPointRot)。
   * 最低點 = 0、最高點約 0.82 圈（(1.45 - 0.698) × 6.875 / 2π），設 0.9 讓整個行程都不會跨過跳變點，
   * 稍微低於最低點時會讀到小負值，而不是突然變成 0.9。
   */
  public static final double kEncoderDiscontinuityPointRot = 0.9;

  public static final double kMaxSpeedRadPerSec = 565;

  /**
   * 射出偵測（老方法，改看轉速）：擊發 SparkMax 轉速（|RPM|）從高於這個值掉到低於這個值 = 剛射出一顆。
   * 可以在 NT {@code /Tuning/Turret/ShotDetectRpm} 即時調；看 {@code Turret/TriggerVelocityRpm} 擊發時衝到多少再決定。
   */
  public static final double kShotDetectRpm = 500;

  public static final Translation3d kTurretToRobotCenter = new Translation3d(0.125, -0.04, 0.52);
  public static final Transform3d kTurretToRobotCenterTransform = new Transform3d(kTurretToRobotCenter, new Rotation3d(0, 0, -Math.PI / 2));

  // far 1 block end
  // target = 5,  pitch: 1.3461316999999622
  // target = 8, pitch: 1.4
  // target = 0, pitch: 1.1641316999999822
  // target = 2, 
}