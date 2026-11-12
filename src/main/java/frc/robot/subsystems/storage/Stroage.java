package frc.robot.subsystems.storage;

import static frc.robot.subsystems.storage.StorageConstants.*;

import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.hardware.discrete.PWM;
import org.wpilib.command2.SubsystemBase;

/**
 * Storage / Trigger 兩顆馬達，經 L298N 驅動。
 *
 * <p>IN1~IN4 控制方向（DigitalOutput），ENA / ENB 用 PWM 的「高電位比例」控制速度。
 * roboRIO 時代用 {@code Spark} 類別：速度 1 會送出約 2000 μs 的脈衝，週期 5 ms，等於約 40% 出力。
 * SystemCore 的 PWM 預設週期是 20 ms，同樣的脈衝只剩約 10%，所以改成直接控制 {@link PWM}。
 * 週期設為 5 ms，速度 1 對應 {@link #kMaxPulseMicroseconds}，維持原本約 40% 的出力。
 */
public class Stroage extends SubsystemBase {
  /** PWM 週期（ms）。SystemCore 只支援 5、10、20。 */
  private static final int kPwmPeriodMs = 5;
  /** 速度 1 時的高電位時間（μs）。2000 / 5000 = 40%，與 roboRIO 時代相同。 */
  private static final int kMaxPulseMicroseconds = 2000;

  private final DigitalOutput storageReverse = new DigitalOutput(kStorageForwardPort);
  private final DigitalOutput storageForward = new DigitalOutput(kStorageReversePort);
  private final DigitalOutput triggerReverse = new DigitalOutput(kTriggerForwardPort);
  private final DigitalOutput triggerForward = new DigitalOutput(kTriggerReversePort);

  private final PWM storage = new PWM(kStorageSpeedPort);
  private final PWM trigger = new PWM(kTriggerSpeedPort);

  public Stroage() {
    storage.setOutputPeriod(kPwmPeriodMs);
    trigger.setOutputPeriod(kPwmPeriodMs);
  }

  /** 把 0~1 的速度換算成 L298N EN 腳的高電位時間。 */
  private static int toPulseMicroseconds(double speed) {
    double clamped = Math.max(0, Math.min(1, Math.abs(speed)));
    return (int) Math.round(clamped * kMaxPulseMicroseconds);
  }

  public void setStorageSpeed(double speed) {
    storage.setPulseTimeMicroseconds(toPulseMicroseconds(speed));
  }

  public void setStorageDirection(boolean forward) {
    storageForward.set(forward);
    storageReverse.set(!forward);
  }

  public void disableStorage() {
    storageForward.set(false);
    storageReverse.set(false);
    storage.setDisabled();
  }

  public void setTriggerSpeed(double speed) {
    trigger.setPulseTimeMicroseconds(toPulseMicroseconds(speed));
  }

  public void setTriggerDirection(boolean forward) {
    triggerForward.set(forward);
    triggerReverse.set(!forward);
  }

  public void disableTrigger() {
    triggerForward.set(false);
    triggerReverse.set(false);
    trigger.setDisabled();
  }

  @Override
  public void periodic() {
    // 直接用 PWM 時，機器人 disable 後不一定會自動停止輸出，所以這裡自己確保停止
    if (RobotState.isDisabled()) {
      disableStorage();
      disableTrigger();
    }
  }
}
