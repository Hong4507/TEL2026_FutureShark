package frc.robot.commands.storage;

import java.util.function.BooleanSupplier;

import org.wpilib.command2.Command;
import frc.robot.subsystems.storage.Stroage;

public class StorageCommands {
  public static Command reload(Stroage stroage, BooleanSupplier shooting) {
    return new Reload(stroage, shooting);
  }
}
