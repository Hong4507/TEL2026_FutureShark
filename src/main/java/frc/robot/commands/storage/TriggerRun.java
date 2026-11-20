package frc.robot.commands.storage;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import frc.robot.subsystems.storage.Stroage;

/**
 * 11/20：Manual／Assist 射擊中左搖桿往前推（Y &gt; 0）→ 只讓 L298N Trigger 一直正轉，Storage 停。
 * 要求 stroage，所以會中斷正在跑的 {@link ShootFeed}；放開後 Storage／Trigger 都停；再射一發才重新啟動 ShootFeed（那一發就開始送料），或右開關關掉再打開。
 */
public class TriggerRun extends Command {
  private final Stroage stroage;
  /** 診斷：右開關打開後啟動過幾次（一直增加 = 一直被取消又重新啟動，通常是 FlySky 斷線、搖桿歸零）。 */
  private static int starts = 0;

  public TriggerRun(Stroage stroage) {
    this.stroage = stroage;
    addRequirements(stroage);
  }

  @Override
  public void initialize() {
    starts++;
    Logger.recordOutput("Storage/ShootFeedState", "ManualTrigger");
    Logger.recordOutput("Storage/TriggerRunStarts", starts);
  }

  @Override
  public void execute() {
    stroage.disableStorage();
    stroage.setTriggerDirection(true);
    stroage.setTriggerSpeed(1);
    Logger.recordOutput("Storage/TriggerRunActive", true);
  }

  @Override
  public void end(boolean interrupted) {
    stroage.disableStorage();
    stroage.disableTrigger();
    Logger.recordOutput("Storage/TriggerRunActive", false);
    Logger.recordOutput("Storage/ShootFeedState", interrupted ? "ManualTrigger-Interrupted" : "ManualTrigger-End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
