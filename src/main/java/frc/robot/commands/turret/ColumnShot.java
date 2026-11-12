package frc.robot.commands.turret;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.FSLib.util.Util;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class ColumnShot extends Command {
  private final Turret turret;
  private final double targetHeightMeters;
  private final int targetFiredCount;

  private final Timer reloadTimer = new Timer();
  private int tureFiredCount = 0;

  private double targetVelocity = TurretConstants.kMaxSpeedRadPerSec;
  private double pitch = TurretConstants.kMinAngleRad;

  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  public ColumnShot(Turret turret, double targetHeightMeters, int targetFireCount) {
    this.turret = turret;
    this.targetHeightMeters = targetHeightMeters;
    this.targetFiredCount = targetFireCount;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    reloadTimer.reset();
    tureFiredCount = 0;
    turret.setAimHeight(targetHeightMeters); // 給記分板判斷這發是哪一列
  }

  @Override
  public void execute() {
    double distance = rangeSensor.getTargetDistance();
    pitch = Util.clamp(
      Math.PI / 2 - Math.atan2(targetHeightMeters - TurretConstants.kTurretToRobotCenter.getZ() + Math.pow(distance, 2) * 0.019358, distance),
      TurretConstants.kMinAngleRad,
      TurretConstants.kMaxAngleRad
    );
    turret.setVelocity(targetVelocity, targetVelocity * 0.8);
    turret.setPitch(pitch);

    if (turret.isNearTargetSpeeds(targetVelocity, targetVelocity * 0.8)) {
      reloadTimer.start();
    }

    if (reloadTimer.isRunning()) {
      if (reloadTimer.get() < 0.2) {
        turret.setTrigger(12);
      } else if (reloadTimer.get() < 1.2) {
        turret.setTrigger(0);
      } else {
        reloadTimer.reset();
        tureFiredCount++;
      };
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    turret.setTrigger(0);
    turret.setAimHeight(Double.NaN);
    reloadTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return tureFiredCount == targetFiredCount;
  }
}
