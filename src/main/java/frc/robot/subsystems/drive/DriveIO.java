package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double leftPositionMeters = 0.0;
    public double leftVelocityMetersPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;

    public double rightPositionMeters = 0.0;
    public double rightVelocityMetersPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}

  public default void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {}
}
