package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  /** 目前瞄準的目標高度（公尺）；ColumnShot 設定，手動射擊時為 NaN（記分板無法判斷是哪一列）。 */
  private double aimHeightMeters = Double.NaN;
  /** 扳機從斷電變成通電的次數 = 射出的球數。 */
  private int shotCount = 0;
  private double lastShotAimHeightMeters = Double.NaN;
  private boolean triggerOn = false;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void setVelocity(double upVelocityRadPerSec, double downVelocityRadPerSec) {
    io.setVelocity(upVelocityRadPerSec, downVelocityRadPerSec);
  }

  public void setVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec, velocityRadPerSec);
  }

  public boolean isNearTargetSpeeds(double targetVelocityRadPerSec) {
    return isNearTargetSpeeds(targetVelocityRadPerSec, targetVelocityRadPerSec);
  }

  public boolean isNearTargetSpeeds(double upTargetVelocityRadPerSec, double downTargetVelocityRadPerSec) {
    return Math.abs(inputs.upVelocityRadPerSec - upTargetVelocityRadPerSec) + Math.abs(inputs.downVelocityRadPerSec - downTargetVelocityRadPerSec) < 10;
  }

  public double getPitchPositionRad() {
    return inputs.pitchPositionRad;
  }

  public void setPitch(double pitchPositionRad) {
    io.setPitch(pitchPositionRad);
  }

  public void setTrigger(double volts) {
    if (volts > 0 && !triggerOn) {
      shotCount++;
      lastShotAimHeightMeters = aimHeightMeters;
      Logger.recordOutput("Turret/ShotCount", shotCount);
    }
    triggerOn = volts > 0;
    io.setTrigger(volts);
  }

  public void setAimHeight(double heightMeters) {
    aimHeightMeters = heightMeters;
  }

  public int getShotCount() {
    return shotCount;
  }

  /** 最近一發射出時瞄準的高度；NaN 表示不是 ColumnShot 射的。 */
  public double getLastShotAimHeight() {
    return lastShotAimHeightMeters;
  }

  public Trigger ballFired() {
    return new Trigger(() -> inputs.triggerVolts < 3);
  }

}
