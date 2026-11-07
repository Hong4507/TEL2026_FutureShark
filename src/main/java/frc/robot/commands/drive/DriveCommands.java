package frc.robot.commands.drive;

import org.wpilib.command2.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private DriveCommands() {}
  
  public static Command arcadeDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return new ArcadeDrive(drive, xSupplier, zSupplier);
  }

  public static Command align(Drive drive) {
    return new LaserAlign(drive);
  }
}