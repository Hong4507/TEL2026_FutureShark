package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.system.Timer;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.FSLib.rangefinder.RangeSensors;
import frc.robot.commands.drive.DriveDistance;
import frc.robot.commands.drive.LidarTurnScan;
import frc.robot.commands.drive.TurnByAngle;
import frc.robot.commands.drive.VisionAlign;
import frc.robot.commands.drive.VisionTurnAlign;
import frc.robot.commands.storage.ShootFeed;
import frc.robot.commands.turret.ColumnShot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.storage.Stroage;
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

  /**
   * 11/19：起點離九宮格 5.8–7.0 m（TargetDistance）。先朝九宮格盲走這麼遠，再逆時針轉 90° 讓右側朝九宮格。
   * 11/20：轉完<b>不再前後走到固定射程</b>，原地用融合距離（Luna×2／Luna單邊+IMU／Limelight）查射擊表。
   * 砲台 Luna（ 23°）有效 TargetDistance 約 2.92–3.62 m → 起點超過約 6.2 m 時可能找不到洞（找不到就不射）。
   */
  private static final LoggedNetworkNumber kBlindApproach = new LoggedNetworkNumber("/Tuning/Auto/BlindApproachMeters", 2.55);
  /** 11/20：原地轉找到洞後先停穩再射（剛停下來車身還會轉一點，第一發會偏）。 */
  private static final double kScanSettleSec = 0.3;
  /** 11/20：每發之間至少隔幾秒（擊發到擊發），讓 ShootFeed 送下一顆球上來。 */
  private static final LoggedNetworkNumber kShotInterval = new LoggedNetworkNumber("/Tuning/Auto/ShotIntervalSec", 5.0);
  /** 每發前確認砲台 Luna 還在洞裡：最多等這麼久。 */
  private static final double kAimConfirmSec = 0.3;
  /** 確認時不在洞裡 → 原地左右各小掃這麼多度重新找洞；還是找不到就不射了。 */
  private static final double kReacquireDeg = 6;

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

    /** 規則上限（算分的最多顆數）。holeCap 也是每洞預設發數；加分格預設 0 發（不射）。 */
    public final int bonusCap;
    public final int holeCap;

    private final LoggedNetworkNumber bonusBalls;
    private final LoggedNetworkNumber ballsPerHole;

    Half(int bonusCap, int holeCap) {
      this.bonusCap = bonusCap;
      this.holeCap = holeCap;
      // 11/19：我們不射加分格 → 預設 0 發（還是會對準亮燈那欄，接著射那欄的低／中／高）。要射再到 Elastic 改
      this.bonusBalls = new LoggedNetworkNumber("/Tuning/Auto/" + name() + "/BonusBalls", 0);
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
    VISION("視覺"),
    /** 11/20：只盲走＋轉向（右側朝九宮格）就停，不瞄準、不射，飛輪不轉。測試起點擺法／轉向用。 */
    APPROACH_ONLY("只盲走＋轉向"),
    /**
     * 11/21：假自瞄。起點<b>右 Frame 朝九宮格</b>（不是車頭朝牆）→ 往中間走 50 cm → 順時針轉 90°（車頭朝九宮格）
     * → 朝九宮格走 2.55 m → 逆時針轉回 90°（右 Frame 朝九宮格）→ 原地慢慢左右來回轉 30 s。不射、飛輪不轉、Storage 不轉。
     * 牆面基準固定用 0°（右 Frame 朝牆），不管 StartYawToWallDeg，見 {@link AutoRoutines#startYawToWallDeg}。
     */
    FAKE_AIM("假自瞄（不射）"),
    /**
     * 11/21：起點<b>車頭朝九宮格</b>（同其他流程，牆面基準用 StartYawToWallDeg）→ 往前走 3 m
     * → 逆時針轉 90°（右 Frame 朝九宮格）→ 往中間走 1 m。不射、飛輪不轉、Storage 不轉。
     * 左右起點都轉逆時針（砲台在右側，一定要右 Frame 朝九宮格）；往中間的方向用 {@link Plan#lunaDirection()}：
     * 右起點往車頭（前進）、左起點往車尾（後退）。
     */
    DRIVE_TURN_DRIVE("走3m＋逆轉90°＋走1m（不射）");

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
      if (method != AutoMethod.VISION) return halfLabel() + "・" + side.label + "・" + method.label;
      return halfLabel() + "・" + side.label + "・" + (runPath ? "跑路徑" : "不跑路徑") + "・" + column.label;
    }
  }

  /**
   * 11/21：Auto 開始時設定 IMU 牆面基準要用的「起點車身相對牆面偏角」。
   * 假自瞄起點是右 Frame 朝九宮格 → 0°；其他流程是車頭朝九宮格 → 用 Elastic 的 StartYawToWallDeg（−90）。
   * 基準錯 90° 的話 IMU 偏角永遠 &gt; 45°，兩顆 Luna／單 Luna＋IMU 都會被丟掉，只剩 Limelight／預設距離。
   */
  public static double startYawToWallDeg(Plan plan, double configuredDeg) {
    return plan.method() == AutoMethod.FAKE_AIM ? 0.0 : configuredDeg;
  }

  /** 假自瞄：往中間走多遠（m）。 */
  private static final LoggedNetworkNumber kFakeAimSideMeters = new LoggedNetworkNumber("/Tuning/Auto/FakeAim/SideMeters", 0.5);
  /** 假自瞄：轉完朝九宮格走多遠（m）。 */
  private static final LoggedNetworkNumber kFakeAimApproachMeters = new LoggedNetworkNumber("/Tuning/Auto/FakeAim/ApproachMeters", 2.55);
  /** 假自瞄：原地來回轉多久（s）。 */
  private static final LoggedNetworkNumber kFakeAimSweepSec = new LoggedNetworkNumber("/Tuning/Auto/FakeAim/SweepSec", 30.0);
  /** 假自瞄：左右各轉幾度。 */
  private static final LoggedNetworkNumber kFakeAimAmplitudeDeg = new LoggedNetworkNumber("/Tuning/Auto/FakeAim/AmplitudeDeg", 15.0);
  /** 假自瞄：來回一次幾秒（15° / 6 s → 最快約 16°/s）。 */
  private static final LoggedNetworkNumber kFakeAimPeriodSec = new LoggedNetworkNumber("/Tuning/Auto/FakeAim/PeriodSec", 6.0);
  private static final double kFakeAimTurnP = 0.6;      // 輪速（m/s）/ rad，同 TurnByAngle
  private static final double kFakeAimMaxWheel = 0.2;

  /**
   * 假自瞄流程（{@link AutoMethod#FAKE_AIM}）。只要求底盤；飛輪、Storage 沒有指令 → 不轉。
   * 往中間的方向用 {@link Plan#lunaDirection()}（右起點、右 Frame 朝牆 → 往車頭）。
   */
  private static Command fakeAim(Drive drive, Plan plan) {
    double[] start = {0};
    double[] center = {0};
    Timer sweepTimer = new Timer();
    Command sweep = Commands.run(() -> {
      double t = sweepTimer.get();
      double period = Math.max(1.0, kFakeAimPeriodSec.get());
      // sin 從 0 開始、整數個週期時回到中間
      double target = center[0] + Math.toRadians(kFakeAimAmplitudeDeg.get()) * Math.sin(2 * Math.PI * t / period);
      double err = org.wpilib.math.util.MathUtil.angleModulus(target - drive.getGyroYawRad());
      double turn = Math.clamp(kFakeAimTurnP * err, -kFakeAimMaxWheel, kFakeAimMaxWheel);
      // arcadeDriveIK：zRotation 正 = 逆時針
      var w = org.wpilib.drive.DifferentialDrive.arcadeDriveIK(0, turn, false);
      drive.setVelocity(w.left, w.right);
      Logger.recordOutput("Auto/FakeAim/TargetDeg", Math.toDegrees(target));
    }, drive)
        .beforeStarting(() -> {
          // 以起點朝向（右 Frame 朝九宮格）為中心來回轉，轉回來沒轉準也不會偏
          center[0] = start[0];
          sweepTimer.restart();
        })
        .until(() -> sweepTimer.hasElapsed(kFakeAimSweepSec.get()))
        .finallyDo(() -> drive.setVelocity(0, 0));
    return Commands.sequence(
      Commands.runOnce(() -> start[0] = drive.getGyroYawRad()),
      log("FakeAim-Side"),
      new DriveDistance(drive, () -> plan.lunaDirection() * kFakeAimSideMeters.get()),
      log("FakeAim-TurnCW"),
      // 順時針 90°（逆時針為正 → −90°），用起點絕對角度，走直線時的偏差不會累積
      TurnByAngle.toHeading(drive, () -> start[0] - Math.PI / 2),
      log("FakeAim-Approach"),
      new DriveDistance(drive, kFakeAimApproachMeters::get),
      log("FakeAim-TurnBack"),
      // 11/21：逆時針轉回起點朝向 → 右 Frame（砲台）再朝九宮格，然後才原地來回轉
      TurnByAngle.toHeading(drive, () -> start[0]),
      log("FakeAim-Sweep"),
      sweep,
      log("Done")
    ).withName("Auto-" + plan.describe());
  }

  /** 走3m＋逆轉90°＋走1m：第一段往前走多遠（m）。 */
  private static final LoggedNetworkNumber kDtdFirstMeters = new LoggedNetworkNumber("/Tuning/Auto/DriveTurnDrive/FirstMeters", 3.0);
  /** 走3m＋逆轉90°＋走1m：轉完往中間走多遠（m，右起點前進、左起點後退）。 */
  private static final LoggedNetworkNumber kDtdSecondMeters = new LoggedNetworkNumber("/Tuning/Auto/DriveTurnDrive/SecondMeters", 1.0);

  /** {@link AutoMethod#DRIVE_TURN_DRIVE}：只要求底盤，飛輪、Storage 沒有指令 → 不轉。 */
  private static Command driveTurnDrive(Drive drive, Plan plan) {
    double[] start = {0};
    return Commands.sequence(
      Commands.runOnce(() -> start[0] = drive.getGyroYawRad()),
      log("DTD-Forward"),
      new DriveDistance(drive, kDtdFirstMeters::get),
      log("DTD-TurnCCW"),
      // 逆時針 90°（逆時針為正），用起點絕對角度
      TurnByAngle.toHeading(drive, () -> start[0] + Math.PI / 2),
      log("DTD-Forward2"),
      // 右 Frame 朝九宮格時：右起點往車頭是中間、左起點往車尾是中間
      new DriveDistance(drive, () -> plan.lunaDirection() * kDtdSecondMeters.get()),
      log("Done")
    ).withName("Auto-" + plan.describe());
  }

  /** 依 {@link Plan#method()} 產生自動流程。 */
  public static Command build(Drive drive, Vision vision, Turret turret, TurretLidar turretLidar, Plan plan) {
    return build(drive, vision, turret, turretLidar, null, plan, 0);
  }

  public static Command build(Drive drive, Vision vision, Turret turret, TurretLidar turretLidar, Plan plan,
      double startYawToWallDeg) {
    return build(drive, vision, turret, turretLidar, null, plan, startYawToWallDeg);
  }

  /**
   * @param stroage 不是 null 時，整段自動流程一起跑 {@link ShootFeed}（偵測到射出 → Storage／L298N Trigger 送下一顆）。
   * @param startYawToWallDeg 起點擺放時車身相對「右側正對九宮格」的偏角（度，逆時針為正；同 {@code /Tuning/RangeSensor/StartYawToWallDeg}）。
   *     11/19 起車頭朝九宮格 = −90。不是 0 時：先朝九宮格盲走 {@link #kBlindApproach}，再原地轉 −startYawToWallDeg
   *     （−90 → 逆時針 90°）讓右側朝九宮格，然後直接接主流程（11/20：不再前後走到固定射程，也不 LaserAlign）。
   *     Luna 找洞和「不跑路徑」的視覺流程都假設右側朝九宮格；跑 PathPlanner 路徑時由路徑決定，不額外轉。
   */
  public static Command build(Drive drive, Vision vision, Turret turret, TurretLidar turretLidar, Stroage stroage,
      Plan plan, double startYawToWallDeg) {
    // 11/21：假自瞄自己處理走位／轉向（起點擺法不同），不接盲走＋轉向，也不跑 ShootFeed
    if (plan.method() == AutoMethod.FAKE_AIM) return fakeAim(drive, plan);
    if (plan.method() == AutoMethod.DRIVE_TURN_DRIVE) return driveTurnDrive(drive, plan);
    boolean approachOnly = plan.method() == AutoMethod.APPROACH_ONLY;
    Command main = switch (plan.method()) {
      case LUNA -> lunaColumn(drive, turret, turretLidar, plan);
      case VISION -> bonusThenColumn(drive, vision, turret, plan);
      case APPROACH_ONLY, FAKE_AIM, DRIVE_TURN_DRIVE -> log("Done").withName("Auto-" + plan.describe());
    };
    String name = main.getName();
    boolean needsTurn = Math.abs(startYawToWallDeg) > 1 && (plan.method() != AutoMethod.VISION || !plan.runPath());
    if (needsTurn) {
      double yaw = Math.toRadians(startYawToWallDeg);
      // 車頭方向往牆的分量（−90 → 1：車頭朝牆；+90 → −1：車尾朝牆）
      double towardWall = Math.cos(-Math.PI / 2 - yaw);
      // 起點朝向（陀螺儀）；右側朝九宮格 = start − yaw（yaw = −90° → 逆時針 90°）
      double[] start = {0};
      main = Commands.sequence(
        Commands.runOnce(() -> start[0] = drive.getGyroYawRad()),
        // 起點離九宮格 5.8–7.0 m，車身 Luna 在黑板上量不到那麼遠 → 盲走（編碼器）
        log("BlindApproach"),
        // 11/20：盲走時飛輪不轉，瞄準完才加速
        Math.abs(towardWall) > 0.5 ? new DriveDistance(drive, () -> kBlindApproach.get() / towardWall) : Commands.none(),
        log("TurnToWall"),
        TurnByAngle.toHeading(drive, () -> start[0] - yaw),
        main
      );
    }
    if (stroage != null && !approachOnly) {
      main = Commands.deadline(main, new ShootFeed(stroage, turret::isTriggerStopped));
    }
    return main.withName(name);
  }

  /**
   * 不用視覺的自動流程（11/15 隊員指定）：
   * <ol>
   *   <li>11/20：<b>不沿牆走</b>（會撞到場地 Frame）。原地轉找洞（{@link LidarTurnScan}，飛輪不轉）：
   *       先往起點決定的方向掃（{@link Plan#lunaDirection()}，往中欄那側），沒找到轉回來再掃另一側。</li>
   *   <li>找到 → 低／中／高各射 {@link Half#ballsPerHole()} 發（上半場預設 3）。</li>
   *   <li>11/20：每發前確認砲台 Luna 還在洞裡（= 對到 Column）才射；不在 → 左右小掃 {@link #kReacquireDeg}° 重找，
   *       還是找不到就停止，剩下的不射。每發之間至少隔 {@link #kShotInterval} 秒（ShootFeed 送球）。</li>
   *   <li>距離：融合距離（Luna×2／Luna單邊+IMU／Limelight，即時讀）/ cos(相對正對牆面的轉角)。</li>
   *   <li>兩邊都沒找到 → 不射。</li>
   *   <li>砲台 Luna 停用或斷線 → 原地直接射（等於人工擺位），照樣每發隔 {@link #kShotInterval} 秒。</li>
   * </ol>
   */
  public static Command lunaColumn(Drive drive, Turret turret, TurretLidar turretLidar, Plan plan) {
    int perHole = plan.half().ballsPerHole();
    double dir = plan.lunaDirection();
    Logger.recordOutput("Auto/BallsPerHole", perHole);
    Logger.recordOutput("Auto/Plan", plan.describe());
    LidarTurnScan scan = new LidarTurnScan(drive, turretLidar, dir);
    LidarTurnScan scanBack = new LidarTurnScan(drive, turretLidar, -dir);
    // 正對牆面時的朝向（用車身 Luna／IMU 算出的牆面偏角修正；沒有就當作掃描前是正對的）
    double[] squareHeading = {0};
    DoubleSupplier slanted = slantedDistance(drive, () -> squareHeading[0]);
    ShotState shots = new ShotState();
    return Commands.sequence(
      // 11/20：找洞時飛輪不轉；找到洞（瞄準完畢）才加速。Luna 不能用時由 ColumnShot 自己加速
      // 判斷 Luna 能不能用之前先等一下（剛開始可能還沒讀到第一筆、或短暫斷線），不要直接當成斷線原地射
      Commands.waitUntil(turretLidar::isUsable).withTimeout(0.5),
      Commands.either(
        Commands.sequence(log("Luna-Unavailable-Shoot"), columnShots(turret, perHole, null, shots, null)),
        Commands.sequence(
          log("Luna-TurnScan"),
          Commands.runOnce(() -> squareHeading[0] = VisionTurnAlign.squareHeadingNow(drive)),
          scan,
          Commands.either(Commands.none(), Commands.sequence(log("Luna-TurnScanBack"), scanBack), scan::found),
          Commands.either(
            Commands.sequence(log("Luna-ScanShoot"),
                // 停在洞裡原地射，每發隔 5 s：不要讓砲台 Luna 把洞後面的箱子學成板子
                Commands.runOnce(() -> turretLidar.setHoldInHole(true)),
                Commands.runOnce(() -> spinUp(turret)),
                Commands.waitSeconds(kScanSettleSec),
                columnShots(turret, perHole, slanted, shots, () -> confirmAim(drive, turretLidar, dir, shots))),
            log("Luna-NoHole"),
            () -> scan.found() || scanBack.found())
        ),
        () -> !turretLidar.isUsable()),
      Commands.runOnce(() -> Logger.recordOutput("Auto/Stage", shots.lost ? "Done-LostHole" : "Done")).ignoringDisable(true)
    ).finallyDo(() -> {
      turret.setVelocity(0);
      turret.setTrigger(0);
      turretLidar.setHoldInHole(false);
    }).withName("Auto-" + plan.describe());
  }

  /** 一次自動流程裡的射擊狀態（每發間隔、是不是對不到洞了）。 */
  private static final class ShotState {
    /** 上一發 ColumnShot 結束的時間；NaN = 還沒射過。 */
    double lastShotEnd = Double.NaN;
    /** 確認時對不到洞、重找也找不到 → 剩下的都不射。 */
    boolean lost = false;
  }

  /** 融合距離（即時）/ cos(現在朝向 − 正對朝向)。 */
  private static DoubleSupplier slantedDistance(Drive drive, DoubleSupplier squareHeading) {
    return () -> {
      double d = RangeSensors.get().getTargetDistance()
          / Math.max(Math.cos(drive.getPose().getRotation().getRadians() - squareHeading.getAsDouble()), 0.5);
      Logger.recordOutput("Auto/ShotDistanceMeters", d);
      return d;
    };
  }

  /**
   * 射前確認砲台 Luna 在洞裡：等最多 {@link #kAimConfirmSec}；不在 → 往 dir 再往 −dir 各小掃 {@link #kReacquireDeg}°，
   * 都找不到 → {@link ShotState#lost} = true。
   */
  private static Command confirmAim(Drive drive, TurretLidar lidar, double dir, ShotState shots) {
    LidarTurnScan a = new LidarTurnScan(drive, lidar, dir, kReacquireDeg);
    LidarTurnScan b = new LidarTurnScan(drive, lidar, -dir, kReacquireDeg);
    return Commands.sequence(
      Commands.waitUntil(lidar::isRawThroughHole).withTimeout(kAimConfirmSec),
      Commands.either(
        Commands.none(),
        Commands.sequence(
          log("Luna-Reacquire"),
          a,
          Commands.either(Commands.none(), b, a::found),
          Commands.waitSeconds(kScanSettleSec)
        ),
        lidar::isRawThroughHole),
      Commands.runOnce(() -> {
        boolean ok = lidar.isRawThroughHole();
        Logger.recordOutput("Auto/AimConfirmed", ok);
        if (!ok) {
          shots.lost = true;
          Logger.recordOutput("Auto/Stage", "Luna-LostHole");
        }
      })
    );
  }

  /** 低、中、高各 count 發，飛輪不停；每發一顆一顆射（見 {@link #spacedShot}）。 */
  private static Command columnShots(Turret turret, int count, DoubleSupplier distance, ShotState shots,
      java.util.function.Supplier<Command> aim) {
    java.util.List<Command> list = new java.util.ArrayList<>();
    for (double height : new double[] {kLowTargetHeight, kMidTargetHeight, kHighTargetHeight}) {
      for (int i = 0; i < count; i++) {
        list.add(spacedShot(turret, height, distance, shots, aim == null ? null : aim.get()));
      }
    }
    return Commands.sequence(list.toArray(Command[]::new));
  }

  /**
   * 一發：等到距離上一發擊發滿 {@link #kShotInterval} 秒 → （有給 aim 時）確認對準 → 對準了才射 1 發。
   * {@link ShotState#lost} 之後的都跳過。
   */
  private static Command spacedShot(Turret turret, double height, DoubleSupplier distance, ShotState shots, Command aim) {
    Command shot = Commands.sequence(
      Commands.waitUntil(() -> Double.isNaN(shots.lastShotEnd)
          || Timer.getTimestamp() - shots.lastShotEnd >= kShotInterval.get() - ColumnShot.kCycleSec),
      aim == null ? Commands.none() : aim,
      Commands.either(
        Commands.none(),
        new ColumnShot(turret, height, 1, false, distance)
            .finallyDo(() -> shots.lastShotEnd = Timer.getTimestamp()),
        () -> shots.lost)
    );
    return Commands.either(Commands.none(), shot, () -> shots.lost);
  }

  /** 舊介面：跑 PathPlanner 路徑、沒亮燈打中欄。 */
  public static Command bonusThenColumn(Drive drive, Vision vision, Turret turret, Half half, String startAuto) {
    StartSide side = startAuto.startsWith("LS") ? StartSide.LEFT : StartSide.RIGHT;
    return bonusThenColumn(drive, vision, turret, new Plan(half, side, true, TargetColumn.MID));
  }

  /**
   * 前 60 秒自動流程：
   * <ol>
   *   <li>（選了跑路徑時）PathPlanner 開到中欄。11/20：飛輪等第一次對準完才加速（之後整段不停，§8.6）。</li>
   *   <li>等最多 {@link #kLitWaitSeconds} 秒找亮燈加分洞 → 對準亮燈那欄、射加分洞；找不到就對 {@link Plan#fallbackColumn()}。</li>
   *   <li>同一欄低／中／高，每發前重新對準 NEAREST，對準失敗重試 1 次（§8.8）。</li>
   *   <li>11/20：對準改成<b>原地轉</b>（{@link VisionTurnAlign}），不沿牆走；射擊距離用 正對距離 / cos(轉角)。</li>
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
    // 車身正對牆面時的朝向（路徑跑完後用車身 Luna 的牆面偏角算；沒有偏角就當作那時是正對的）；之後原地轉對準都相對這個角度
    double[] square = {0};
    DoubleSupplier squareHeading = () -> square[0];
    ShotState shots = new ShotState();
    return Commands.sequence(
      log(plan.runPath() ? "Path" : "NoPath"),
      // 11/20：跑路徑、找欄時飛輪不轉；第一次對準完才加速
      plan.runPath() ? new PathPlannerAuto(plan.side().pathAuto) : Commands.none(),
      Commands.runOnce(() -> square[0] = VisionTurnAlign.squareHeadingNow(drive)),
      log("WaitLit"),
      Commands.waitUntil(() -> litVisible(vision)).withTimeout(kLitWaitSeconds),
      Commands.either(
        Commands.sequence(
          log("AlignLit"),
          alignWithRetry(drive, vision, VisionAlign.Target.LIT, squareHeading),
          Commands.runOnce(() -> spinUp(turret)),
          log("Bonus"),
          alignedShots(drive, vision, turret, kSpecialTargetHeight, bonusBalls, squareHeading, shots)
        ),
        Commands.sequence(
          log("NoLit-Align" + fallback.name()),
          alignWithRetry(drive, vision, fallback, squareHeading),
          Commands.runOnce(() -> spinUp(turret))
        ),
        () -> litVisible(vision)
      ),
      log("Column"),
      alignedShots(drive, vision, turret, kLowTargetHeight, ballsPerHole, squareHeading, shots),
      alignedShots(drive, vision, turret, kMidTargetHeight, ballsPerHole, squareHeading, shots),
      alignedShots(drive, vision, turret, kHighTargetHeight, ballsPerHole, squareHeading, shots),
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

  /** 原地轉視覺對準；失敗（逾時／找不到）就再試一次（§8.8）。 */
  private static Command alignWithRetry(Drive drive, Vision vision, VisionAlign.Target target, DoubleSupplier squareHeading) {
    VisionTurnAlign first = new VisionTurnAlign(drive, vision, target, squareHeading);
    return Commands.sequence(
      first,
      Commands.either(
        Commands.sequence(
          Commands.runOnce(() -> Logger.recordOutput("Auto/AlignRetry", target.name())),
          new VisionTurnAlign(drive, vision, target, squareHeading)
        ),
        Commands.none(),
        () -> !first.succeeded()
      )
    );
  }

  /**
   * 每一發前先用視覺對準最近的一欄（原地轉），再射一發。飛輪不停（§8.6）。距離 = 正對距離 / cos(轉角)。
   * 11/20：每發之間至少隔 {@link #kShotInterval} 秒（ShootFeed 送球）。
   */
  private static Command alignedShots(Drive drive, Vision vision, Turret turret, double height, int count,
      DoubleSupplier squareHeading, ShotState state) {
    DoubleSupplier slanted = slantedDistance(drive, squareHeading);
    Command[] shots = new Command[count];
    for (int i = 0; i < count; i++) {
      shots[i] = spacedShot(turret, height, slanted, state,
          alignWithRetry(drive, vision, VisionAlign.Target.NEAREST, squareHeading));
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
