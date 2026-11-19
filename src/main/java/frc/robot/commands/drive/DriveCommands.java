package frc.robot.commands.drive;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
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

  /**
   * 手動找洞：先往車頭找 {@value #kLunaSearchMeters} m，沒找到就回頭往車尾找 2 倍距離（涵蓋出發點前後）。
   * 欄距 0.9 m → 最近的洞中心一定在 ±0.45 m 內；進洞邊緣比中心近約 0.14 m，所以 0.35 m 就找得到最近那一欄，
   * 不會跑去下一欄。
   */
  public static final double kLunaSearchMeters = 0.35;

  /**
   * 用砲台 TF-Luna 對準最近的一欄（11/19，右搖桿左上）：沿牆找洞，停在進洞和出洞的中間。
   * 砲台 Luna 停用／斷線 → 不動。
   */
  public static Command lunaCenterAlign(Drive drive, frc.robot.subsystems.turretlidar.TurretLidar lidar) {
    LidarHoleAlign forward = new LidarHoleAlign(drive, lidar, 1, 0, kLunaSearchMeters);
    LidarHoleAlign back = new LidarHoleAlign(drive, lidar, -1, 0, 2 * kLunaSearchMeters);
    return Commands.sequence(
      forward,
      Commands.either(Commands.none(), back,
          () -> forward.found() || forward.result() == LidarHoleAlign.Result.UNAVAILABLE)
    ).withName("LunaCenterAlign");
  }
}