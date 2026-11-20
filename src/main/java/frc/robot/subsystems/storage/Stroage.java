package frc.robot.subsystems.storage;

import static frc.robot.subsystems.storage.StorageConstants.*;

import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.command2.SubsystemBase;

/**
 * Storage（左、右兩顆）與 Trigger 共三顆馬達，經 L298N 驅動。
 *
 * <p>SystemCore 的 6 個 Smart I/O 全部當 DigitalOutput 接 L298N 的 IN 腳，只控制開／關與方向。
 * L298N 的 ENA / ENB jumper 要插著（常開），所以馬達開啟時就是全速。
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
      (forward ? reversePin : forwardPin).set(false);
      (forward ? forwardPin : reversePin).set(true);
      state = forward ? 1 : -1;
    }

    void stop() {
      forwardPin.set(false);
      reversePin.set(false);
      state = 0;
    }

    /**
     * 診斷：最後寫出去的狀態（+1 = 正轉腳高、-1 = 反轉腳高、0 = 停）。
     * ⚠️ 不能用 DigitalOutput.get() 讀回來：SystemCore Smart I/O 不支援，會丟 UncleanStatusException（-3）讓程式當掉。
     */
    private int state = 0;

    int state() {
      return state;
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
    // 機器人 disable 時確保所有 IN 腳都是低電位
    if (RobotState.isDisabled()) {
      disableStorage();
      disableTrigger();
    }
    // 11/20 診斷：L298N IN 腳實際輸出（+1 正轉、-1 反轉、0 停）。程式顯示有轉但這裡是 0 → 程式問題；這裡 ±1 但馬達不轉 → 接線／L298N／電源
    org.littletonrobotics.junction.Logger.recordOutput("Storage/TriggerPin", trigger.state());
    org.littletonrobotics.junction.Logger.recordOutput("Storage/StorageLeftPin", storageLeft.state());
    org.littletonrobotics.junction.Logger.recordOutput("Storage/StorageRightPin", storageRight.state());
  }
}
