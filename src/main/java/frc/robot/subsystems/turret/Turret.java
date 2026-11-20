package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  private static final LoggedNetworkNumber shotDetectRpm =
      new LoggedNetworkNumber("/Tuning/Turret/ShotDetectRpm", TurretConstants.kShotDetectRpm);

  /**
   * 11/20 暫時測試用：假擊發。true → 擊發 SparkMax 不轉（一律送 0 V），射出偵測改看「有沒有下擊發指令」：
   * 擊發指令結束的那一刻 = 射出一顆，ShootFeed 等照常反應。⚠️ 比賽前確認是 false（開機預設 false）。
   */
  private static final LoggedNetworkBoolean dryFire = new LoggedNetworkBoolean("/Tuning/Turret/DryFire", false);

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/TriggerStopped", isTriggerStopped());
    Logger.recordOutput("Turret/DryFire", dryFire.get());
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

  public double getUpVelocityRadPerSec() {
    return inputs.upVelocityRadPerSec;
  }

  /** 上下飛輪 TalonFX 與仰角 CANcoder 都有回應。 */
  public boolean isUpConnected() {
    return inputs.upConnected;
  }

  public boolean isDownConnected() {
    return inputs.downConnected;
  }

  public boolean isEncoderConnected() {
    return inputs.encoderConnected;
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
    io.setTrigger(dryFire.get() ? 0 : volts);
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

  /** 擊發 SparkMax 轉速（|RPM|）低於 {@link TurretConstants#kShotDetectRpm} = 停著。 */
  public boolean isTriggerStopped() {
    if (dryFire.get()) return !triggerOn;
    return Math.abs(inputs.triggerVelocityRpm) < shotDetectRpm.get();
  }

  /**
   * 射出偵測（老方法）：看擊發 SparkMax 轉速是否低於門檻。搭配 onTrue／上升緣用：
   * 轉速從高於門檻掉到低於門檻的那一刻 = 剛射出一顆。
   *
   * <p>不用回報電壓（舊版 {@code triggerVolts < 3}）：扳機設了 inverted，電壓是負的，永遠 &lt; 3。
   * 轉速取絕對值就沒有這個問題。
   */
  public Trigger ballFired() {
    return new Trigger(this::isTriggerStopped);
  }

}
