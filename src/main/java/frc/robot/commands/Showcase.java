package frc.robot.commands;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.robot.controller.FlySkyController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class Showcase extends Command {
  private final Drive drive;
  private final Turret turret;
  private final FlySkyController controller;

  private final Timer timer = new Timer();

  private final PIDController drivePid = new PIDController(2, 0, 0);
  private final PIDController turnPid = new PIDController(0.01, 0, 0);

  public Showcase(Drive drive, Turret turret, FlySkyController controller) {
    this.drive = drive;
    this.turret = turret;
    this.controller = controller;
    addRequirements(drive, turret);
  }
  
  @Override
  public void initialize() {
    timer.restart();
    drive.setPose(new Pose2d());
  }

  @Override
  public void execute() {
    if (timer.get() < 2) {
      double vel = drivePid.calculate(drive.getPose().getX(), 2.5);
      drive.setVelocity(vel, vel);
    } else if (timer.get() < 2.5) {
      double vel = turnPid.calculate(drive.getPose().getRotation().getDegrees(), 30);
      drive.setVelocity(-vel, vel);
    } else if (timer.get() < 3) {
      double vel = turnPid.calculate(drive.getPose().getRotation().getDegrees(), 0);
      drive.setVelocity(-vel, vel);
    } else if (timer.get() < 3.5) {
      double vel = turnPid.calculate(drive.getPose().getRotation().getDegrees(), -30);
      drive.setVelocity(-vel, vel);
    } else if (timer.get() < 4.5) {
      double vel = turnPid.calculate(drive.getPose().getRotation().getDegrees(), 180);
      drive.setVelocity(-vel, vel);
    } else if (timer.get() < 6) {
      double vel = drivePid.calculate(drive.getPose().getX(), 0);
      drive.setVelocity(-vel, -vel);
    }

    if (timer.get() > 1 && timer.get() < 3.5) {
      turret.setVelocity(TurretConstants.kMaxSpeedRadPerSec);
    } else {
      turret.setVelocity(0);
    }

    if (controller.getRightX() > 0.2) {
      turret.setTrigger(6);
    } else {
      turret.setTrigger(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
    turret.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 6;
  }
}
