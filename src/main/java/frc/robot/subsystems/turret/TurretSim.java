package frc.robot.subsystems.turret;

import org.wpilib.math.filter.SlewRateLimiter;

public class TurretSim implements TurretIO {
  private final SlewRateLimiter pitchLimiter = new SlewRateLimiter(4, -4, TurretConstants.kMinAngleRad);

  private double upSpeed = 0;
  private double downSpeed = 0;
  private double pitch = TurretConstants.kMinAngleRad;
  /** 擊發馬達轉速（RPM）：一階近似，時間常數約 50 ms，讓模擬也有「轉起來 → 慢下來」可以偵測。 */
  private double triggerRpm = 0;
  private double triggerVolts = 0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.pitchPositionRad = pitch;
    inputs.upConnected = true;
    inputs.downConnected = true;
    inputs.encoderConnected = true;
    inputs.upVelocityRadPerSec = upSpeed;
    inputs.downVelocityRadPerSec = downSpeed;
    triggerRpm += (triggerVolts / 12.0 * 11000 - triggerRpm) * 0.2;
    inputs.triggerVolts = triggerVolts;
    inputs.triggerVelocityRpm = triggerRpm;
  }

  @Override
  public void setTrigger(double volts) {
    triggerVolts = volts;
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
