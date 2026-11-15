package frc.robot.commands.turret;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.TurretConstants;

public class ColumnShot extends Command {
  private final Turret turret;
  private final double targetHeightMeters;
  private final int targetFiredCount;
  /** 結束時要不要停飛輪。自動流程中 false（§8.6：每發轉速一樣，停了再加速沒好處），流程結束才停。 */
  private final boolean stopFlywheelAtEnd;

  private final Timer reloadTimer = new Timer();
  private int tureFiredCount = 0;

  private double targetVelocity = TurretConstants.kMaxSpeedRadPerSec;
  private double pitch = TurretConstants.kMinAngleRad;

  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  public ColumnShot(Turret turret, double targetHeightMeters, int targetFireCount) {
    this(turret, targetHeightMeters, targetFireCount, true);
  }

  public ColumnShot(Turret turret, double targetHeightMeters, int targetFireCount, boolean stopFlywheelAtEnd) {
    this.stopFlywheelAtEnd = stopFlywheelAtEnd;
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
    pitch = ShotTable.pitchRad(targetHeightMeters, distance); // 查表（表空著時 = 原本的彈道公式）
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
    if (stopFlywheelAtEnd) turret.setVelocity(0);
    turret.setTrigger(0);
    turret.setAimHeight(Double.NaN);
    reloadTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return tureFiredCount == targetFiredCount;
  }
}
