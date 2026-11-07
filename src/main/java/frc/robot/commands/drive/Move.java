package frc.robot.commands.drive;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import frc.robot.subsystems.drive.Drive;

public class Move extends Command {
  private final Drive drive;
  
  private final PIDController pid = new PIDController(2, 0, 0);
  private final SlewRateLimiter limiter = new SlewRateLimiter(1);

  public 
  Move(Drive drive) {
    this.drive = drive;
    addRequirements(drive);

    pid.setSetpoint(-0.235);
    pid.setTolerance(0.02);
  }

  @Override
  public void initialize() {
    drive.setPose(new Pose2d());
  }

  @Override
  public void execute() {
    double speed = pid.calculate(drive.getPose().getX());
    speed = limiter.calculate(speed);
    drive.setVelocity(speed, speed);
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
