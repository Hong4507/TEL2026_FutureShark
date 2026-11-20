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

  /** 沒有牆距時，原地轉每一邊最多轉幾度。 */
  public static final double kLunaTurnFallbackDeg = 8.0;

  /**
   * 原地轉每一邊最多轉幾度：光點沿牆移動 {@value #kLunaSearchMeters} m 的角度（和平移版同樣的理由，
   * 只找得到最近那一欄的進洞邊緣，不會先掃到另一邊比較遠的那一欄）。中列約 3.1 m → 6.4°、低列 1.7 m → 11.6°。
   */
  static double lunaTurnMaxDeg(frc.robot.subsystems.turretlidar.TurretLidar lidar) {
    double d = lidar.getSquareHorizontalMeters();
    if (!(d > 0.5)) return kLunaTurnFallbackDeg;
    return Math.toDegrees(Math.atan(kLunaSearchMeters / d));
  }

  /**
   * 11/20：用砲台 TF-Luna 對準最近的一欄，<b>原地轉</b>（不沿牆走，不會撞到場地 Frame）。
   * 先逆時針掃（{@link #lunaTurnMaxDeg}），沒找到轉回原角度再順時針掃；找到就停在進洞和出洞角度的中間。
   * 砲台 Luna 停用／斷線 → 不動。
   */
  public static Command lunaTurnAlign(Drive drive, frc.robot.subsystems.turretlidar.TurretLidar lidar) {
    LidarTurnScan ccw = new LidarTurnScan(drive, lidar, 1, () -> lunaTurnMaxDeg(lidar));
    LidarTurnScan cw = new LidarTurnScan(drive, lidar, -1, () -> lunaTurnMaxDeg(lidar));
    return Commands.sequence(
      ccw,
      Commands.either(Commands.none(), cw, () -> ccw.found() || !lidar.isUsable())
    ).withName("LunaTurnAlign");
  }

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