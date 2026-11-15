package frc.robot.commands.storage;

import org.wpilib.command2.Command;
import frc.robot.subsystems.storage.Stroage;

/**
 * Storage 預設指令：L298N 上的三顆馬達（左、右 Storage 與 L298N Trigger 通道）**一律正轉**，
 * 把球一路送到砲台；真正決定射不射的是砲台的 NEO550 擊發馬達（Turret.setTrigger）。
 * 機器人 disable 時由 Stroage.periodic() 全部關掉。
 */
public class Normal extends Command {
  private final Stroage stroage;

  public Normal(Stroage stroage) {
    this.stroage = stroage;
    addRequirements(stroage);
  }

  @Override
  public void execute() {
    stroage.setStorageDirection(true);
    stroage.setStorageSpeed(1);
    stroage.setTriggerDirection(true);
    stroage.setTriggerSpeed(1);
  }

  @Override
  public void end(boolean interrupted) {
    stroage.disableStorage();
    stroage.disableTrigger();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
