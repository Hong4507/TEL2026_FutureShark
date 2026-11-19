package frc.robot.subsystems.storage;

import static frc.robot.subsystems.storage.StorageConstants.*;

import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.command2.SubsystemBase;

/**
 * Storage（左、右兩顆）與 Trigger 共三顆馬達，經 L298N 或繼電器驅動。
 *
 * <p>SystemCore 的 6 個 Smart I/O 全部當 DigitalOutput，只控制開／關與方向（全速，不能調速）。
 * <ul>
 *   <li>L298N：接 IN 腳，ENA / ENB jumper 要插著（常開）。</li>
 *   <li>繼電器（11/19）：每顆馬達兩個 SPDT 繼電器 A、B。馬達兩端各接一個繼電器的 COM；
 *       NC → 電源負（GND），NO → +12V（經保險絲／斷路器）。
 *       都不吸合 = 兩端都接地 = 停（還會煞車）；A 吸合 = 正轉；B 吸合 = 反轉；
 *       <b>兩個都吸合 = 兩端都 +12V = 停，不會短路</b>（這種接法沒有燒掉的組合）。
 *       低電位觸發的模組要把 {@link StorageConstants#kOutputActiveLow} 設 true。</li>
 * </ul>
 */
public class Stroage extends SubsystemBase {
  /** 一個 L298N 通道：forward 腳高 = 正轉，reverse 腳高 = 反轉，兩支都低 = 停止。 */
  private static class Channel {
    private final DigitalOutput forwardPin;
    private final DigitalOutput reversePin;
    private boolean forward = true;

    Channel(int forwardPort, int reversePort) {
      forwardPin = new DigitalOutput(forwardPort);
      reversePin = new DigitalOutput(reversePort);
      stop();
    }

    /** on = 這支腳「動作」（L298N IN 高電位／繼電器吸合）。低電位觸發時反過來輸出。 */
    private static void write(DigitalOutput pin, boolean on) {
      pin.set(on != kOutputActiveLow);
    }

    void setDirection(boolean forward) {
      this.forward = forward;
    }

    /** 速度只看是否為 0：非 0 就以目前方向全速轉，0 就停止。 */
    void setSpeed(double speed) {
      if (speed == 0) {
        stop();
        return;
      }
      // 先把另一支腳拉低，避免兩支腳同時為高
      write(forward ? reversePin : forwardPin, false);
      write(forward ? forwardPin : reversePin, true);
    }

    void stop() {
      write(forwardPin, false);
      write(reversePin, false);
    }
  }

  private final Channel storageLeft = new Channel(kStorageLeftForwardPort, kStorageLeftReversePort);
  private final Channel storageRight = new Channel(kStorageRightForwardPort, kStorageRightReversePort);
  private final Channel trigger = new Channel(kTriggerForwardPort, kTriggerReversePort);

  public void setStorageSpeed(double speed) {
    storageLeft.setSpeed(speed);
    storageRight.setSpeed(speed);
  }

  public void setStorageDirection(boolean forward) {
    storageLeft.setDirection(forward);
    storageRight.setDirection(forward);
  }

  public void disableStorage() {
    storageLeft.stop();
    storageRight.stop();
  }

  public void setTriggerSpeed(double speed) {
    trigger.setSpeed(speed);
  }

  public void setTriggerDirection(boolean forward) {
    trigger.setDirection(forward);
  }

  public void disableTrigger() {
    trigger.stop();
  }

  @Override
  public void periodic() {
    // 機器人 disable 時確保所有輸出都是「關」
    if (RobotState.isDisabled()) {
      disableStorage();
      disableTrigger();
    }
  }
}
