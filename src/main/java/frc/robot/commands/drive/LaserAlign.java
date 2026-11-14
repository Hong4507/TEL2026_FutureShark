package frc.robot.commands.drive;

import static org.wpilib.units.Units.Radians;

import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.system.Timer;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.drive.DifferentialDrive.WheelVelocities;
import org.wpilib.command2.Command;
import org.littletonrobotics.junction.Logger;
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

  private Angle yaw;
  private double startErrorRad;
  private boolean wrongWay;

  public LaserAlign(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    failedTimer.restart();
    totalTimer.restart();
    startErrorRad = Double.NaN;
    wrongWay = false;
    Logger.recordOutput("LaserAlign/WrongDirection", false);
  }

  @Override
  public void execute() {
    yaw = rangeSensor.getYaw();
    if (yaw != null) {
      double error = Math.abs(Math.PI / 2 - yaw.in(Radians));
      if (Double.isNaN(startErrorRad)) startErrorRad = error;
      if (error > startErrorRad + kWrongWayRad) {
        wrongWay = true;
        Logger.recordOutput("LaserAlign/WrongDirection", true);
      }
      Logger.recordOutput("LaserAlign/ErrorRad", Math.PI / 2 - yaw.in(Radians));
      WheelVelocities speeds = DifferentialDrive.arcadeDriveIK(0, Math.copySign(0.1, 1.57 - yaw.in(Radians)), false);
      drive.setVelocity(speeds.left, speeds.right);
      failedTimer.reset();
    } else {
      drive.setVelocity(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
  }

  @Override
  public boolean isFinished() {
    return yaw == null || MathUtil.isNear(Math.PI / 2, yaw.in(Radians), 0.095) || failedTimer.hasElapsed(0.2)
      || wrongWay || totalTimer.hasElapsed(kTimeoutSec);
  }
}
