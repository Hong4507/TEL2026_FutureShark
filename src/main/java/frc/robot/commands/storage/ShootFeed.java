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
 *   <li>偵測到射出（擊發 SparkMax 轉速掉到門檻以下的那一刻）→ 先只有 L298N Trigger <b>單獨轉</b> {@code kShotFeedPreTriggerSec} 秒（11/21）。</li>
 *   <li>L298N Trigger 和 Storage <b>一起轉</b> {@code kShotFeedTriggerSec} 秒。</li>
 *   <li>Storage 停、L298N Trigger <b>繼續轉</b> {@code kShotFeedStorageSec} 秒。</li>
 *   <li>回到等待。</li>
 * </ol>
 * 送料途中又射一發 → 從第 2 步（Trigger 單獨轉）重新開始。右開關關掉 → 指令結束，全部停止。
 *
 * <p>11/20：{@code manualTrigger} 為 true 時 L298N Trigger 一律正轉（目前沒用到：Manual／Assist 射擊中左搖桿往前推改由 {@link TriggerRun} 中斷這個指令）。
 *
 * @see frc.robot.subsystems.turret.Turret#ballFired()
 */
public class ShootFeed extends Command {
  private final Stroage stroage;
  /** 擊發馬達「停著」（轉速低於門檻）。false → true 的那一刻 = 剛射出。 */
  private final BooleanSupplier triggerStopped;
  /** 手動讓 L298N Trigger 正轉。 */
  private final BooleanSupplier manualTrigger;
  private final boolean startFeeding;
  private final Timer timer = new Timer();
  private boolean lastStopped;
  private boolean feeding;
  /** 診斷：這次右開關打開後偵測到幾發。比 Assist/FireCount 少 = 偵測漏掉，Storage 沒送球。 */
  private int detected;

  public ShootFeed(Stroage stroage, BooleanSupplier triggerStopped) {
    this(stroage, triggerStopped, () -> false);
  }

  public ShootFeed(Stroage stroage, BooleanSupplier triggerStopped, BooleanSupplier manualTrigger) {
    this(stroage, triggerStopped, manualTrigger, false);
  }

  /**
   * @param startFeeding true = 一開始就當作剛射出一顆、直接進送料流程（11/20：TriggerRun 之後再射一發才重新啟動，
   *     那一發本身也要補球）。
   */
  public ShootFeed(Stroage stroage, BooleanSupplier triggerStopped, BooleanSupplier manualTrigger, boolean startFeeding) {
    this.startFeeding = startFeeding;
    this.stroage = stroage;
    this.triggerStopped = triggerStopped;
    this.manualTrigger = manualTrigger;
    addRequirements(stroage);
  }

  @Override
  public void initialize() {
    // 進入射擊模式時擊發馬達本來就停著，不能算成一發
    lastStopped = triggerStopped.getAsBoolean();
    feeding = startFeeding;
    detected = startFeeding ? 1 : 0;
    if (startFeeding) {
      timer.restart();
    } else {
      timer.stop();
      stop();
    }
  }

  @Override
  public void execute() {
    boolean stopped = triggerStopped.getAsBoolean();
    if (stopped && !lastStopped) {
      feeding = true;
      timer.restart();
      detected++;
    }
    Logger.recordOutput("Storage/ShotsDetected", detected);
    lastStopped = stopped;

    boolean manual = manualTrigger.getAsBoolean();
    Logger.recordOutput("Storage/ManualTrigger", manual);
    String state;
    double t = timer.get();
    double together = kShotFeedPreTriggerSec + kShotFeedTriggerSec;
    if (feeding && t < kShotFeedPreTriggerSec) {
      // 第 1 段（11/21）：Trigger 先單獨轉，Storage 停
      stroage.disableStorage();
      stroage.setTriggerDirection(true);
      stroage.setTriggerSpeed(1);
      state = "PreTrigger";
    } else if (feeding && t < together) {
      // 第 2 段：Trigger＋Storage 一起轉
      stroage.setStorageDirection(true);
      stroage.setTriggerDirection(true);
      stroage.setStorageSpeed(1);
      stroage.setTriggerSpeed(1);
      state = "Trigger+Storage";
    } else if (feeding && t < together + kShotFeedStorageSec) {
      // 第 3 段：Storage 停、Trigger 繼續轉
      stroage.disableStorage();
      stroage.setTriggerDirection(true);
      stroage.setTriggerSpeed(1);
      state = "TriggerOnly";
    } else {
      feeding = false;
      stroage.disableStorage();
      if (manual) {
        stroage.setTriggerDirection(true);
        stroage.setTriggerSpeed(1);
      } else {
        stroage.disableTrigger();
      }
      state = manual ? "ManualTrigger" : "Waiting";
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
