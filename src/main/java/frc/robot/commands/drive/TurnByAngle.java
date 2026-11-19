package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.drive.DifferentialDrive.WheelVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;

import frc.robot.subsystems.drive.Drive;

/**
 * 原地轉指定角度（陀螺儀，逆時針為正）。11/19：起點車頭朝九宮格 → 自動開始先轉 +90° 讓右側（射球方向）朝九宮格。
 * 角度用 {@link Drive#getGyroYawRad()}（不會被 setPose 重設）。P 控制＋最小速度，誤差 &lt; 1.5° 連續 3 個週期結束，3 秒逾時。
 */
public class TurnByAngle extends Command {
  private static final double kP = 0.6;           // 輪速（m/s）/ rad
  private static final double kMaxWheel = 0.35;
  private static final double kMinWheel = 0.04;
  private static final double kToleranceRad = Math.toRadians(1.0);
  private static final double kTimeoutSec = 3.0;

  private final Drive drive;
  private final double angleRad;
  /** 不是 null 時：轉到這個絕對陀螺儀角度（多次轉向時誤差不會累積）。 */
  private final java.util.function.DoubleSupplier absoluteTarget;
  private double target;
  private int settled;
  private final Timer timer = new Timer();

  public TurnByAngle(Drive drive, double angleRad) {
    this.drive = drive;
    this.angleRad = angleRad;
    this.absoluteTarget = null;
    addRequirements(drive);
  }

  private TurnByAngle(Drive drive, java.util.function.DoubleSupplier absoluteTarget) {
    this.drive = drive;
    this.angleRad = 0;
    this.absoluteTarget = absoluteTarget;
    addRequirements(drive);
  }

  /** 轉到絕對陀螺儀角度（{@link Drive#getGyroYawRad()}），在 initialize 時讀。 */
  public static TurnByAngle toHeading(Drive drive, java.util.function.DoubleSupplier gyroYawRad) {
    return new TurnByAngle(drive, gyroYawRad);
  }

  @Override
  public void initialize() {
    target = absoluteTarget != null ? absoluteTarget.getAsDouble() : drive.getGyroYawRad() + angleRad;
    settled = 0;
    timer.restart();
  }

  @Override
  public void execute() {
    double err = MathUtil.angleModulus(target - drive.getGyroYawRad());
    double turn = 0;
    if (Math.abs(err) < kToleranceRad) {
      settled++;
    } else {
      settled = 0;
      turn = Math.clamp(kP * err, -kMaxWheel, kMaxWheel);
      if (Math.abs(turn) < kMinWheel) turn = Math.copySign(kMinWheel, err);
    }
    // arcadeDriveIK：zRotation 正 = 逆時針
    WheelVelocities w = DifferentialDrive.arcadeDriveIK(0, turn, false);
    drive.setVelocity(w.left, w.right);
    Logger.recordOutput("TurnByAngle/ErrorDeg", Math.toDegrees(err));
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
  }

  @Override
  public boolean isFinished() {
    return settled >= 3 || timer.hasElapsed(kTimeoutSec);
  }
}
