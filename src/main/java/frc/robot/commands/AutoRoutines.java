package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.FSLib.rangefinder.RangeSensors;
import frc.robot.commands.drive.VisionAlign;
import frc.robot.commands.turret.ColumnShot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

/**
 * 前 60 秒自動流程（策略見 Upgrade.md「自動策略」）。
 *
 * <p>原則：自動只做手動做不到的事 —— 加分格只在前 60 秒算分，所以一定要在自動打。
 * 對準亮燈那一欄後不再換欄（避免和對手相撞），順便把同一欄的洞打完，換欄交給手動。
 * 每一發前都重新用視覺對準一次（已對準時約 0.1 秒），被撞歪也能修回來。
 */
public final class AutoRoutines {
  private AutoRoutines() {}

  /** 等亮燈加分格出現的最長時間（秒）。超過就當作燈沒亮 / 辨識不到，改打中欄、不打加分格。 */
  public static final double kLitWaitSeconds = 2.0;

  /**
   * 一場比賽的規則差異。發數可在 NT {@code /Tuning/Auto/<FIRST|SECOND>/*} 調整（11/14 依實測命中率，
   * 用 {@code jetson_tools/strategy_calc.py} 算出建議值）。自動流程每次啟動時才讀取，所以比賽前改了就會生效。
   */
  public enum Half {
    /** 上半場：規則上限加分格 3 顆、每格 3 顆。 */
    FIRST(3, 3),
    /** 下半場：規則上限加分格 2 顆、每格 2 顆（連線另計，換欄交給手動）。 */
    SECOND(2, 2);

    /** 規則上限（算分的最多顆數），也是預設發數。 */
    public final int bonusCap;
    public final int holeCap;

    private final LoggedNetworkNumber bonusBalls;
    private final LoggedNetworkNumber ballsPerHole;

    Half(int bonusCap, int holeCap) {
      this.bonusCap = bonusCap;
      this.holeCap = holeCap;
      this.bonusBalls = new LoggedNetworkNumber("/Tuning/Auto/" + name() + "/BonusBalls", bonusCap);
      this.ballsPerHole = new LoggedNetworkNumber("/Tuning/Auto/" + name() + "/BallsPerHole", holeCap);
    }

    /** 加分格要射幾發。命中率不高時可以超過上限（多射幾發確保進 bonusCap 顆）。 */
    public int bonusBalls() {
      return clampCount(bonusBalls.get());
    }

    /** 同一欄每個洞射幾發。 */
    public int ballsPerHole() {
      return clampCount(ballsPerHole.get());
    }

    private static int clampCount(double value) {
      return (int) Math.max(0, Math.min(10, Math.round(value)));
    }
  }

  /**
   * @param startAuto PathPlanner 的 auto 名稱（從起始框開到中欄並 LaserAlign），例如 "RS_C2_Auto"
   */
  public static Command bonusThenColumn(Drive drive, Vision vision, Turret turret, Half half, String startAuto) {
    int bonusBalls = half.bonusBalls();
    int ballsPerHole = half.ballsPerHole();
    Logger.recordOutput("Auto/BonusBalls", bonusBalls);
    Logger.recordOutput("Auto/BallsPerHole", ballsPerHole);
    return Commands.sequence(
      log("Path"),
      new PathPlannerAuto(startAuto),
      log("WaitLit"),
      Commands.waitUntil(() -> litVisible(vision)).withTimeout(kLitWaitSeconds),
      Commands.either(
        Commands.sequence(
          log("AlignLit"),
          new VisionAlign(drive, vision, VisionAlign.Target.LIT),
          log("Bonus"),
          alignedShots(drive, vision, turret, kSpecialTargetHeight, bonusBalls)
        ),
        Commands.sequence(
          log("NoLit-AlignMid"),
          new VisionAlign(drive, vision, VisionAlign.Target.MID)
        ),
        () -> litVisible(vision)
      ),
      log("Column"),
      alignedShots(drive, vision, turret, kLowTargetHeight, ballsPerHole),
      alignedShots(drive, vision, turret, kMidTargetHeight, ballsPerHole),
      alignedShots(drive, vision, turret, kHighTargetHeight, ballsPerHole),
      log("Done")
    ).withName("Auto-" + half + "-" + startAuto);
  }

  /** 每一發前先用視覺對準最近的一欄（已鎖定的那欄），再射一發。 */
  private static Command alignedShots(Drive drive, Vision vision, Turret turret, double height, int count) {
    Command[] shots = new Command[count];
    for (int i = 0; i < count; i++) {
      shots[i] = Commands.sequence(
        new VisionAlign(drive, vision, VisionAlign.Target.NEAREST),
        new ColumnShot(turret, height, 1)
      );
    }
    return Commands.sequence(shots);
  }

  private static boolean litVisible(Vision vision) {
    return vision.getLitError(RangeSensors.get().getTargetDistance()).isPresent();
  }

  private static Command log(String stage) {
    return Commands.runOnce(() -> Logger.recordOutput("Auto/Stage", stage)).ignoringDisable(true);
  }
}
