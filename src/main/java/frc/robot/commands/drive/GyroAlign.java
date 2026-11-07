package frc.robot.commands.drive;

import org.wpilib.math.controller.PIDController;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.command2.Command;
import frc.robot.subsystems.drive.Drive;

public class GyroAlign extends Command {
  private final Drive drive;

  private final PIDController pid = new PIDController(1, 0, 0);

  public GyroAlign(Drive drive) {
    this.drive = drive;
    addRequirements(drive);

    pid.setSetpoint(1.57);
    pid.setTolerance(0.05);
    pid.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void execute() {
    var speeds = DifferentialDrive.arcadeDriveIK(0, -pid.calculate(drive.getPose().getRotation().getRadians()), false);
    drive.setVelocity(speeds.left, speeds.right);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
