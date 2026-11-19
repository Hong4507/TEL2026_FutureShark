package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;

import frc.robot.subsystems.drive.Drive;

/**
 * 直走指定距離（輪子編碼器／里程計，正 = 往車頭）。距離在 initialize 時才讀（可以用前一步量到的值）。
 * P 控制＋最小速度，誤差 &lt; 2 cm 連續 3 週期結束；走不動（0.5 s 前進 &lt; 1 cm）或逾時就停。
 */
public class DriveDistance extends Command {
  private static final double kP = 1.5;
  private static final double kMaxSpeed = 0.6;
  private static final double kMinSpeed = 0.05;
  private static final double kToleranceMeters = 0.02;

  private final Drive drive;
  private final DoubleSupplier meters;
  private double target;
  private Translation2d origin;
  private Translation2d axis;
  private int settled;
  private final Timer timer = new Timer();
  private final Timer stallTimer = new Timer();
  private double stallRef;
  private boolean stalled;

  public DriveDistance(Drive drive, DoubleSupplier meters) {
    this.drive = drive;
    this.meters = meters;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    target = meters.getAsDouble();
    if (!Double.isFinite(target)) target = 0;
    var p = drive.getPose();
    origin = p.getTranslation();
    axis = new Translation2d(p.getRotation().getCos(), p.getRotation().getSin());
    settled = 0;
    stalled = false;
    stallRef = 0;
    timer.restart();
    stallTimer.restart();
    Logger.recordOutput("DriveDistance/TargetMeters", target);
  }

  private double progress() {
    Translation2d d = drive.getPose().getTranslation().minus(origin);
    return d.getX() * axis.getX() + d.getY() * axis.getY();
  }

  @Override
  public void execute() {
    double s = progress();
    double err = target - s;
    double speed = 0;
    if (Math.abs(err) < kToleranceMeters) {
      settled++;
    } else {
      settled = 0;
      speed = Math.clamp(kP * err, -kMaxSpeed, kMaxSpeed);
      if (Math.abs(speed) < kMinSpeed) speed = Math.copySign(kMinSpeed, err);
    }
    if (stallTimer.hasElapsed(0.5)) {
      if (speed != 0 && Math.abs(s - stallRef) < 0.01) stalled = true;
      stallRef = s;
      stallTimer.restart();
    }
    drive.setVelocity(speed, speed);
    Logger.recordOutput("DriveDistance/ErrorMeters", err);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
  }

  @Override
  public boolean isFinished() {
    return settled >= 3 || stalled || timer.hasElapsed(2 + Math.abs(target) / 0.3);
  }
}
