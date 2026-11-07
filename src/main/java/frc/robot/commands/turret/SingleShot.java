package frc.robot.commands.turret;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class SingleShot extends Command {

  private final Timer timer = new Timer();
  private final Turret turret;

  public SingleShot (Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    timer.restart();
  }
  
  @Override
  public void execute() {
    turret.setVelocity(TurretConstants.kMaxSpeedRadPerSec, TurretConstants.kMaxSpeedRadPerSec * 0.8);
    if (timer.get() < 0.15) {
      turret.setTrigger(12);
    } else {
      turret.setTrigger(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0, 0);
    turret.setTrigger(0);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.2);
  }
}
