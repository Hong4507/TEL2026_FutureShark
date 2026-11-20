package frc.robot.commands.drive;

import static frc.robot.subsystems.drive.DriveConstants.kMaxSpeedMetersPerSec;

import java.util.function.DoubleSupplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.command2.Command;
import frc.robot.subsystems.drive.Drive;

public class ArcadeDrive extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier, zSupplier;

  public ArcadeDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.zSupplier = zSupplier;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // 11/21：拿掉加速度限制（原本 SlewRateLimiter(1)），搖桿直接給速度
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.02);
    double z = MathUtil.applyDeadband(zSupplier.getAsDouble(), 0.02);
    var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);
    drive.setVelocity(speeds.left * kMaxSpeedMetersPerSec, speeds.right * kMaxSpeedMetersPerSec);
  }
}
