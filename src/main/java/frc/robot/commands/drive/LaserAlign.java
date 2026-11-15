package frc.robot.commands.drive;

import static org.wpilib.units.Units.Radians;

import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.system.Timer;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.drive.DifferentialDrive.WheelVelocities;
import org.wpilib.command2.Command;
import org.littletonrobotics.junction.Logger;
import frc.FSLib.rangefinder.RangeSensorConstants;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.subsystems.drive.Drive;

public class LaserAlign extends Command {
  private final Drive drive;

  double nearestValue = 0;

  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  private final Timer failedTimer = new Timer();
  private final Timer totalTimer = new Timer();

  /** 正常從 30° 歪轉正約 1.2 s；超過這個時間一定有問題，停下來不要一直轉。 */
  private static final double kTimeoutSec = 2.0;
  /** 誤差比剛開始還大這麼多（rad）→ 轉錯方向，通常是 kYawSign 或左右 TF-Luna 接反。 */
  private static final double kWrongWayRad = 0.15;

  /** 角度死區（rad）。 */
  private static final double kDeadbandRad = Math.toRadians(RangeSensorConstants.kAlignDeadbandDeg);
  /** 誤差小於這個值（rad，≈ 5.4°）就算擺正、結束。 */
  private static final double kToleranceRad = 0.095;

  private Angle yaw;
  private double startErrorRad;
  private boolean wrongWay;

  public LaserAlign(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    // 開機就發布，沒跑過也看得到 key（AdvantageKit 只在第一次 recordOutput 後才建立 NT topic）
    Logger.recordOutput("LaserAlign/Active", false);
    Logger.recordOutput("LaserAlign/WrongDirection", false);
    Logger.recordOutput("LaserAlign/YawValid", false);
    Logger.recordOutput("LaserAlign/ErrorRad", Double.NaN);
    Logger.recordOutput("LaserAlign/EndReason", "NotRun");
  }

  @Override
  public void initialize() {
    failedTimer.restart();
    totalTimer.restart();
    startErrorRad = Double.NaN;
    wrongWay = false;
    Logger.recordOutput("LaserAlign/Active", true);
    Logger.recordOutput("LaserAlign/WrongDirection", false);
    Logger.recordOutput("LaserAlign/EndReason", "Running");
  }

  @Override
  public void execute() {
    yaw = rangeSensor.getYaw();
    Logger.recordOutput("LaserAlign/YawValid", yaw != null);
    if (yaw == null) Logger.recordOutput("LaserAlign/ErrorRad", Double.NaN);
    if (yaw != null) {
      double error = Math.abs(Math.PI / 2 - yaw.in(Radians));
      if (Double.isNaN(startErrorRad)) startErrorRad = error;
      if (error > startErrorRad + kWrongWayRad) {
        wrongWay = true;
        Logger.recordOutput("LaserAlign/WrongDirection", true);
      }
      Logger.recordOutput("LaserAlign/ErrorRad", Math.PI / 2 - yaw.in(Radians));
      double signedError = Math.PI / 2 - yaw.in(Radians);
      // §8.3 死區：基線只有 17 cm，讀值差 1 cm ≈ 3.4°，目標附近不要一直修
      double turn = Math.abs(signedError) < kDeadbandRad ? 0 : Math.copySign(0.1, signedError);
      WheelVelocities speeds = DifferentialDrive.arcadeDriveIK(0, turn, false);
      drive.setVelocity(speeds.left, speeds.right);
      failedTimer.reset();
    } else {
      drive.setVelocity(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
    String reason;
    if (interrupted) reason = "Interrupted";
    else if (yaw == null) reason = "NoYaw";            // 測距無效（Luna 讀 0／單邊有效）
    else if (wrongWay) reason = "WrongDirection";
    else if (MathUtil.isNear(Math.PI / 2, yaw.in(Radians), kToleranceRad)) reason = "Aligned";
    else if (totalTimer.hasElapsed(kTimeoutSec)) reason = "Timeout";
    else reason = "Other";
    Logger.recordOutput("LaserAlign/Active", false);
    Logger.recordOutput("LaserAlign/EndReason", reason);
  }

  @Override
  public boolean isFinished() {
    return yaw == null || MathUtil.isNear(Math.PI / 2, yaw.in(Radians), kToleranceRad) || failedTimer.hasElapsed(0.2)
      || wrongWay || totalTimer.hasElapsed(kTimeoutSec);
  }
}
