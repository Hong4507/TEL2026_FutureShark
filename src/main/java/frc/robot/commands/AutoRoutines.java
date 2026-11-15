package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.FSLib.rangefinder.RangeSensors;
import frc.robot.commands.drive.LidarHoleAlign;
import frc.robot.commands.drive.LidarTurnScan;
import frc.robot.commands.drive.VisionAlign;
import frc.robot.commands.turret.ColumnShot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turretlidar.TurretLidar;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Column;

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

  /** 起點在九宮格的哪一側（看向九宮格）。 */
  public enum StartSide {
    LEFT("左起點", "LS_C2_Auto", Column.LEFT),
    RIGHT("右起點", "RS_C2_Auto", Column.RIGHT);

    public final String label;
    /** PathPlanner auto：從起點開到中欄並 LaserAlign。 */
    public final String pathAuto;
    /** 己方欄 = 起點那一側的欄（§8.8 定案）。 */
    public final Column ownColumn;

    StartSide(String label, String pathAuto, Column ownColumn) {
      this.label = label;
      this.pathAuto = pathAuto;
      this.ownColumn = ownColumn;
    }
  }

  /** 沒看到亮燈時要打哪一欄（§8.8）。亮燈加分洞永遠優先。 */
  public enum TargetColumn {
    MID("中欄"),
    OWN_SIDE("己方欄");

    public final String label;

    TargetColumn(String label) {
      this.label = label;
    }
  }

  /**
   * 一次自動流程的設定。
   *
   * @param runPath false = 不跑路徑（§8.7）：機器人直接擺在目標欄前面，只做 VisionAlign＋射擊。
   */
  /** 自動流程用哪一種方式對準欄。 */
  public enum AutoMethod {
    /** 往前走、用砲台 TF-Luna 找洞（不用視覺）；被擋住就原地轉找洞。 */
    LUNA("砲台Luna找洞"),
    /** 原本的視覺流程（加分洞＋VisionAlign）。 */
    VISION("視覺");

    public final String label;

    AutoMethod(String label) {
      this.label = label;
    }
  }

  public record Plan(Half half, StartSide side, boolean runPath, TargetColumn column, AutoMethod method) {
    public Plan(Half half, StartSide side, boolean runPath, TargetColumn column) {
      this(half, side, runPath, column, AutoMethod.VISION);
    }

    /** 砲台 Luna 找洞的方向：右起點往車頭（+1）、左起點往車尾（-1），兩邊都是往中欄走。 */
    public double lunaDirection() {
      return side == StartSide.RIGHT ? 1 : -1;
    }

    public Column fallbackColumn() {
      return column == TargetColumn.MID ? Column.MID : side.ownColumn;
    }

    public String halfLabel() {
      return half == Half.FIRST ? "上半場" : "下半場";
    }

    public String describe() {
      if (method == AutoMethod.LUNA) return halfLabel() + "・" + side.label + "・" + method.label;
      return halfLabel() + "・" + side.label + "・" + (runPath ? "跑路徑" : "不跑路徑") + "・" + column.label;
    }
  }

  /** 依 {@link Plan#method()} 產生自動流程。 */
  public static Command build(Drive drive, Vision vision, Turret turret, TurretLidar turretLidar, Plan plan) {
    return plan.method() == AutoMethod.LUNA
        ? lunaColumn(drive, turret, turretLidar, plan)
        : bonusThenColumn(drive, vision, turret, plan);
  }

  /**
   * 不用視覺的自動流程（11/15 隊員指定）：
   * <ol>
   *   <li>飛輪開始轉＋往前走 {@code /Tuning/LunaAuto/ApproachMeters}，再慢慢走直到砲台 Luna 穿過洞、停在洞中間（{@link LidarHoleAlign}）。</li>
   *   <li>找到 → 低／中／高各射 {@link Half#ballsPerHole()} 發（上半場預設 3）。</li>
   *   <li>走不動（被對手擋住）或走完沒找到 → 原地轉找洞（{@link LidarTurnScan}），找到才射，距離用 正對距離 / cos(轉角)。</li>
   *   <li>砲台 Luna 停用或斷線 → 只走 ApproachMeters，原地直接射（等於人工擺位）。</li>
   * </ol>
   * 方向由起點決定（{@link Plan#lunaDirection()}）。
   */
  public static Command lunaColumn(Drive drive, Turret turret, TurretLidar turretLidar, Plan plan) {
    int perHole = plan.half().ballsPerHole();
    double dir = plan.lunaDirection();
    Logger.recordOutput("Auto/BallsPerHole", perHole);
    Logger.recordOutput("Auto/Plan", plan.describe());
    LidarHoleAlign align = new LidarHoleAlign(drive, turretLidar, dir);
    LidarTurnScan scan = new LidarTurnScan(drive, turretLidar, dir);
    double[] squareDistance = {Double.NaN};
    DoubleSupplier slanted = () -> squareDistance[0] / Math.max(Math.cos(scan.turnedRad()), 0.5);
    return Commands.sequence(
      log("Luna-Approach"),
      Commands.parallel(Commands.runOnce(() -> spinUp(turret)), align),
      Commands.either(
        Commands.sequence(log("Luna-Shoot"), columnShots(turret, perHole, null)),
        Commands.either(
          Commands.sequence(log("Luna-Unavailable-Shoot"), columnShots(turret, perHole, null)),
          Commands.sequence(
            log("Luna-TurnScan"),
            Commands.runOnce(() -> squareDistance[0] = RangeSensors.get().getTargetDistance()),
            scan,
            Commands.either(
              Commands.sequence(log("Luna-ScanShoot"), columnShots(turret, perHole, slanted)),
              log("Luna-NoHole"),
              scan::found)
          ),
          () -> align.result() == LidarHoleAlign.Result.UNAVAILABLE),
        align::found),
      log("Done")
    ).finallyDo(() -> {
      turret.setVelocity(0);
      turret.setTrigger(0);
    }).withName("Auto-" + plan.describe());
  }

  /** 低、中、高各 count 發，飛輪不停。 */
  private static Command columnShots(Turret turret, int count, DoubleSupplier distance) {
    return Commands.sequence(
      new ColumnShot(turret, kLowTargetHeight, count, false, distance),
      new ColumnShot(turret, kMidTargetHeight, count, false, distance),
      new ColumnShot(turret, kHighTargetHeight, count, false, distance)
    );
  }

  /** 舊介面：跑 PathPlanner 路徑、沒亮燈打中欄。 */
  public static Command bonusThenColumn(Drive drive, Vision vision, Turret turret, Half half, String startAuto) {
    StartSide side = startAuto.startsWith("LS") ? StartSide.LEFT : StartSide.RIGHT;
    return bonusThenColumn(drive, vision, turret, new Plan(half, side, true, TargetColumn.MID));
  }

  /**
   * 前 60 秒自動流程：
   * <ol>
   *   <li>飛輪開始轉（§8.6，整段不停）＋（選了跑路徑時）PathPlanner 開到中欄。</li>
   *   <li>等最多 {@link #kLitWaitSeconds} 秒找亮燈加分洞 → 對準亮燈那欄、射加分洞；找不到就對 {@link Plan#fallbackColumn()}。</li>
   *   <li>同一欄低／中／高，每發前重新 VisionAlign NEAREST，對準失敗重試 1 次（§8.8）。</li>
   *   <li>結束或取消時停飛輪。</li>
   * </ol>
   */
  public static Command bonusThenColumn(Drive drive, Vision vision, Turret turret, Plan plan) {
    Half half = plan.half();
    int bonusBalls = half.bonusBalls();
    int ballsPerHole = half.ballsPerHole();
    Logger.recordOutput("Auto/BonusBalls", bonusBalls);
    Logger.recordOutput("Auto/BallsPerHole", ballsPerHole);
    Logger.recordOutput("Auto/Plan", plan.describe());
    VisionAlign.Target fallback = switch (plan.fallbackColumn()) {
      case LEFT -> VisionAlign.Target.LEFT;
      case MID -> VisionAlign.Target.MID;
      case RIGHT -> VisionAlign.Target.RIGHT;
    };
    return Commands.sequence(
      log(plan.runPath() ? "Path" : "NoPath"),
      Commands.parallel(
        Commands.runOnce(() -> spinUp(turret)),
        plan.runPath() ? new PathPlannerAuto(plan.side().pathAuto) : Commands.none()
      ),
      log("WaitLit"),
      Commands.waitUntil(() -> litVisible(vision)).withTimeout(kLitWaitSeconds),
      Commands.either(
        Commands.sequence(
          log("AlignLit"),
          alignWithRetry(drive, vision, VisionAlign.Target.LIT),
          log("Bonus"),
          alignedShots(drive, vision, turret, kSpecialTargetHeight, bonusBalls)
        ),
        Commands.sequence(
          log("NoLit-Align" + fallback.name()),
          alignWithRetry(drive, vision, fallback)
        ),
        () -> litVisible(vision)
      ),
      log("Column"),
      alignedShots(drive, vision, turret, kLowTargetHeight, ballsPerHole),
      alignedShots(drive, vision, turret, kMidTargetHeight, ballsPerHole),
      alignedShots(drive, vision, turret, kHighTargetHeight, ballsPerHole),
      log("Done")
    ).finallyDo(() -> {
      turret.setVelocity(0);
      turret.setTrigger(0);
    }).withName("Auto-" + plan.describe());
  }

  /** 和 ColumnShot 同一個轉速（下飛輪 80%）。 */
  private static void spinUp(Turret turret) {
    double v = TurretConstants.kMaxSpeedRadPerSec;
    turret.setVelocity(v, v * 0.8);
  }

  /** VisionAlign；失敗（逾時／找不到）就再試一次（§8.8）。 */
  private static Command alignWithRetry(Drive drive, Vision vision, VisionAlign.Target target) {
    VisionAlign first = new VisionAlign(drive, vision, target);
    return Commands.sequence(
      first,
      Commands.either(
        Commands.sequence(
          Commands.runOnce(() -> Logger.recordOutput("Auto/AlignRetry", target.name())),
          new VisionAlign(drive, vision, target)
        ),
        Commands.none(),
        () -> !first.succeeded()
      )
    );
  }

  /** 每一發前先用視覺對準最近的一欄（已鎖定的那欄），再射一發。飛輪不停（§8.6）。 */
  private static Command alignedShots(Drive drive, Vision vision, Turret turret, double height, int count) {
    Command[] shots = new Command[count];
    for (int i = 0; i < count; i++) {
      shots[i] = Commands.sequence(
        alignWithRetry(drive, vision, VisionAlign.Target.NEAREST),
        new ColumnShot(turret, height, 1, false)
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
