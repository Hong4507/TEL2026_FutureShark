package frc.robot.commands.storage;

import org.wpilib.command2.Command;
import frc.robot.subsystems.storage.Stroage;

public class StorageCommands {
  public static Command reload(Stroage stroage) {
    return new Reload(stroage);
  }
}
