package frc.robot.commands.turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.FSLib.util.Util;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class AssistTurret extends Command {
  private final Turret turret;
  private final double targetHeightMeters;
  private final DoubleSupplier adjestmentSupplier;
  private final BooleanSupplier fireSupplier;

  private final Timer reloadTimer = new Timer();

  private double targetVelocity = TurretConstants.kMaxSpeedRadPerSec;
  private double pitch = TurretConstants.kMinAngleRad;

  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  public AssistTurret(Turret turret, double targetHeightMeters, DoubleSupplier adjestmentSupplier, BooleanSupplier fireSupplier) {
    this.turret = turret;
    this.targetHeightMeters = targetHeightMeters;
    this.adjestmentSupplier = adjestmentSupplier;
    this.fireSupplier = fireSupplier;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    reloadTimer.reset();
  }

  @Override
  public void execute() {
    double distance = rangeSensor.getTargetDistance();
    pitch = Util.clamp(
      Math.PI / 2
        - Math.atan2(targetHeightMeters - TurretConstants.kTurretToRobotCenter.getZ() + Math.pow(distance, 2) * 0.019358, distance)
        + Util.mapAxis(adjestmentSupplier.getAsDouble(), -0.1, 0.1),
      TurretConstants.kMinAngleRad,
      TurretConstants.kMaxAngleRad
    );
    turret.setVelocity(targetVelocity, targetVelocity * 0.8);
    turret.setPitch(pitch);

    if (turret.isNearTargetSpeeds(targetVelocity, targetVelocity * 0.8) && fireSupplier.getAsBoolean()) {
      reloadTimer.restart();
    }

    if (reloadTimer.isRunning()) {
      if (reloadTimer.get() < 0.2) {
        turret.setTrigger(12);
      } else {
        turret.setTrigger(0);
      };
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    turret.setTrigger(0);
    reloadTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
