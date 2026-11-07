package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

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
    io.setTrigger(volts);
  }

  public Trigger ballFired() {
    return new Trigger(() -> inputs.triggerVolts < 3);
  }

}
