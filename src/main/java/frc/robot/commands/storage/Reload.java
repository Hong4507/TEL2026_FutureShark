package frc.robot.commands.storage;

import java.util.function.BooleanSupplier;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.robot.subsystems.storage.Stroage;

public class Reload extends Command {
  private final Stroage stroage;
  private final BooleanSupplier shooting;
  private final Timer timer = new Timer();

  public Reload(Stroage stroage, BooleanSupplier shooting) {
    this.stroage = stroage;
    this.shooting = shooting;
    addRequirements(stroage);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    if (timer.get() < 0.3) {
      stroage.setStorageDirection(shooting.getAsBoolean()); // 射球中 Storage 不反轉
      stroage.setStorageSpeed(1);
      stroage.setTriggerDirection(shooting.getAsBoolean()); // 射球中 Trigger 也不反轉
      stroage.setTriggerSpeed(1);
    } else if (timer.get() < 1) {
      stroage.setStorageDirection(true);
      stroage.setStorageSpeed(1);
      stroage.setTriggerDirection(true);
      stroage.setTriggerSpeed(1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    stroage.disableStorage();
    stroage.disableTrigger();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }
}
