package frc.robot.commands.storage;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.robot.subsystems.storage.Stroage;

public class Normal extends Command {
  private final Stroage stroage;
  private final Timer timer = new Timer();

  public Normal(Stroage stroage) {
    this.stroage = stroage;
    addRequirements(stroage);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    if (timer.get() < 0.3) {
      stroage.setStorageDirection(false);
      stroage.setStorageSpeed(1);
    } else if (timer.get() < 1.5) {
      stroage.setStorageDirection(true);
      stroage.setStorageSpeed(1);
    } else {
      timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    stroage.disableStorage();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
