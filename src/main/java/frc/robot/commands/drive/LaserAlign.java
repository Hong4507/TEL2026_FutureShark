package frc.robot.commands.drive;

import static org.wpilib.units.Units.Radians;

import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.system.Timer;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.drive.DifferentialDrive.WheelVelocities;
import org.wpilib.command2.Command;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.subsystems.drive.Drive;

public class LaserAlign extends Command {
  private final Drive drive;

  double nearestValue = 0;

  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  private final Timer failedTimer = new Timer();

  private Angle yaw;

  public LaserAlign(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    failedTimer.restart();
  }

  @Override
  public void execute() {
    yaw = rangeSensor.getYaw();
    if (yaw != null) {
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
    return yaw == null || MathUtil.isNear(Math.PI / 2, yaw.in(Radians), 0.095) || failedTimer.hasElapsed(0.2);
  }
}
