package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double pitchPositionRad = 0.0;

    public double upVelocityRadPerSec = 0.0;
    public double upAppliedVolts = 0.0;

    public double downVelocityRadPerSec = 0.0;
    public double downAppliedVolts = 0.0;

    public double triggerVolts = 0.0;
    public double triggerCurrentAmps = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setVoltage(double upVolts, double downVolts) {}

  public default void setVelocity(double upVelocityRadPerSec, double downVelocityRadPerSec) {}

  public default void setPitch(double pitchPositionRad) {}

  public default void setTrigger(double volts) {}

}
