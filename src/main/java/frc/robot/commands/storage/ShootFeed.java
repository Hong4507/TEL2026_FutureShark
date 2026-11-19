package frc.robot.commands.storage;

import static frc.robot.subsystems.storage.StorageConstants.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
import frc.robot.subsystems.storage.Stroage;

/**
 * 射擊模式（右開關打開）時的 Storage／L298N Trigger：兩者分開控制。
 *
 * <ol>
 *   <li>等待：兩個都停。</li>
 *   <li>偵測到射出（擊發 SparkMax 轉速掉到門檻以下的那一刻）→ L298N Trigger 轉 {@code kShotFeedTriggerSec} 秒，Storage 不動。</li>
 *   <li>Trigger 停 → Storage 轉 {@code kShotFeedStorageSec} 秒。</li>
 *   <li>回到等待。</li>
 * </ol>
 * 送料途中又射一發 → 從第 2 步重新開始。右開關關掉 → 指令結束，全部停止。
 *
 * @see frc.robot.subsystems.turret.Turret#ballFired()
 */
public class ShootFeed extends Command {
  private final Stroage stroage;
  /** 擊發馬達「停著」（轉速低於門檻）。false → true 的那一刻 = 剛射出。 */
  private final BooleanSupplier triggerStopped;
  private final Timer timer = new Timer();
  private boolean lastStopped;
  private boolean feeding;

  public ShootFeed(Stroage stroage, BooleanSupplier triggerStopped) {
    this.stroage = stroage;
    this.triggerStopped = triggerStopped;
    addRequirements(stroage);
  }

  @Override
  public void initialize() {
    // 進入射擊模式時擊發馬達本來就停著，不能算成一發
    lastStopped = triggerStopped.getAsBoolean();
    feeding = false;
    timer.stop();
    stop();
  }

  @Override
  public void execute() {
    boolean stopped = triggerStopped.getAsBoolean();
    if (stopped && !lastStopped) {
      feeding = true;
      timer.restart();
    }
    lastStopped = stopped;

    String state;
    double t = timer.get();
    if (feeding && t < kShotFeedTriggerSec) {
      // stroage.disableStorage();
      stroage.setTriggerDirection(true);
      stroage.setStorageSpeed(1);
      stroage.setTriggerSpeed(1);
      state = "Trigger";
    } else if (feeding && t < kShotFeedTriggerSec + kShotFeedStorageSec) {
      stroage.disableStorage();
      stroage.setStorageDirection(true);
      stroage.setTriggerSpeed(1);
      state = "Storage";
    } else {
      feeding = false;
      stop();
      state = "Waiting";
    }
    Logger.recordOutput("Storage/ShootFeedState", state);
  }

  private void stop() {
    stroage.disableStorage();
    stroage.disableTrigger();
  }

  @Override
  public void end(boolean interrupted) {
    stop();
    Logger.recordOutput("Storage/ShootFeedState", "Off");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
