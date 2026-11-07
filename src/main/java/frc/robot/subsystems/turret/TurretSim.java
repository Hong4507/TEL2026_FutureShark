package frc.robot.subsystems.turret;

import org.wpilib.math.filter.SlewRateLimiter;

public class TurretSim implements TurretIO {
  private final SlewRateLimiter pitchLimiter = new SlewRateLimiter(4);

  private double upSpeed = 0;
  private double downSpeed = 0;
  private double pitch = 0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.pitchPositionRad = pitch;
    inputs.upVelocityRadPerSec = upSpeed;
    inputs.downVelocityRadPerSec = downSpeed;
  }

  @Override
  public void setVelocity(double upVelocityRadPerSec, double downVelocityRadPerSec) {
    upSpeed = upVelocityRadPerSec;
    downSpeed = downVelocityRadPerSec;
  }

  @Override
  public void setPitch(double pitchPositionRad) {
    pitch = pitchLimiter.calculate(pitchPositionRad);
  }
}
