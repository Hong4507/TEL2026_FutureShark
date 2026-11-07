package frc.robot.commands.turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.FSLib.util.Util;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class ManualTurret extends Command {
  private final Turret turret;

  private final DoubleSupplier angleSupplier;
  private final BooleanSupplier triggerSupplier;
  boolean lastValue = false;
  private final Timer timer = new Timer();


  public ManualTurret(Turret turret, DoubleSupplier angleSupplier, BooleanSupplier triggerSupplier) {
    this.turret = turret;
    this.angleSupplier = angleSupplier;
    this.triggerSupplier = triggerSupplier;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    turret.setPitch(Util.mapAxis(angleSupplier.getAsDouble(), TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad));
    turret.setVelocity(TurretConstants.kMaxSpeedRadPerSec, TurretConstants.kMaxSpeedRadPerSec * 0.8);

    if (triggerSupplier.getAsBoolean() && !lastValue) {
      timer.restart();
    }
    lastValue = triggerSupplier.getAsBoolean();

    if (timer.isRunning()) {
      if (timer.get() < 0.15) {
        turret.setTrigger(12);
      } else {
        turret.setTrigger(0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    turret.setTrigger(0);
  }
}
