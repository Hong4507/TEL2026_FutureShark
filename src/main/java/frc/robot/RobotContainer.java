// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import org.wpilib.math.util.MathUtil;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.filter.Debouncer;
import frc.FSLib.rangefinder.RangeSensors;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotMode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.AutoRoutines.AutoMethod;
import frc.robot.commands.AutoRoutines.Half;
import frc.robot.commands.AutoRoutines.Plan;
import frc.robot.commands.AutoRoutines.StartSide;
import frc.robot.commands.AutoRoutines.TargetColumn;
import frc.robot.commands.drive.LaserAlign;
import frc.robot.commands.drive.VisionAlign;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.storage.Normal;
import frc.robot.commands.storage.ShootFeed;
import frc.robot.commands.storage.TriggerRun;
import frc.robot.commands.turret.AssistTurret;
import frc.robot.commands.turret.ColumnShot;
import frc.robot.commands.turret.ManualTurret;
import frc.robot.commands.turret.RealManualTurret;
import frc.robot.controller.CommandFlySkyController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOOnboardIMU;
import frc.robot.subsystems.scoreboard.Scoreboard;
import frc.robot.subsystems.status.StatusBoard;
import frc.robot.subsystems.storage.Stroage;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretReal;
import frc.robot.subsystems.turret.TurretSim;
import frc.robot.subsystems.turretlidar.TurretLidar;
import frc.robot.subsystems.turretlidar.TurretLidarIO;
import frc.robot.subsystems.turretlidar.TurretLidarIONT;
import frc.robot.subsystems.turretlidar.TurretLidarIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIONT;
import frc.robot.subsystems.vision.VisionIOSim;

public class RobotContainer {
  private final Drive drive;
  private final Turret turret;
  private final Vision vision;
  /** 砲台正前方的 TF-Luna（接 Jetson，經 NT）。 */
  private final TurretLidar turretLidar;
  private final Stroage stroage = new Stroage();

  private CommandFlySkyController controller;

  private final MockDS mockDS = new MockDS();

  /** 前 60 秒自動流程選單（筆電 Dashboard 選，比賽前 90 秒準備時間設定）。 */
  private final LoggedNetworkChooser<AutoOption> autoChooser = new LoggedNetworkChooser<>("Auto Choices");

  /** §8.7：跑 PathPlanner 路徑，或機器人直接擺在目標欄前面、只做 VisionAlign＋射擊。 */
  private final LoggedNetworkChooser<Boolean> autoPathChooser = new LoggedNetworkChooser<>("Auto Path");
  /** §8.8：沒亮燈時打哪一欄（己方欄 = 起點那一側）。 */
  private final LoggedNetworkChooser<TargetColumn> autoColumnChooser = new LoggedNetworkChooser<>("Auto Column");
  /** 11/15：砲台 Luna 找洞（不用視覺，預設）或原本的視覺流程。 */
  private final LoggedNetworkChooser<AutoMethod> autoMethodChooser = new LoggedNetworkChooser<>("Auto Method");
  /** 11/19：右搖桿左上的對齊方式（Elastic 賽前頁「對齊方式」）。 */
  private enum AlignMethod { LUNA_TURN, LUNA, VISION, LASER }
  private final LoggedNetworkChooser<AlignMethod> alignChooser = new LoggedNetworkChooser<>("Align Method");

  /** 自動選項：哪個半場（記分板用來決定每格上限）+ 起點。half == null 代表還沒選（READY 燈不會亮）。 */
  private record AutoOption(Half half, StartSide side) {
    static final AutoOption NONE = new AutoOption(null, null);

    boolean selected() {
      return half != null;
    }
  }

  private Scoreboard scoreboard;

  /** 撥桿啟動的自動流程正在跑（它是在 kDisable 模式下跑的，Storage 要照常送球）。 */
  private boolean autoRunning = false;

  /**
   * 11/19：開機時兩顆 Luna 打不到牆 → 自動開始時用「起點擺放角度」設定 IMU 的牆面基準。
   * 車身相對九宮格牆面的偏角（度，逆時針為正，同 IMU；0 = 右 frame 正對牆）。
   * 11/19 起 init 時車頭（Front frame）朝九宮格牆 = 從「右 frame 朝牆」順時針轉 90° → −90。
   * 若實測轉成右 frame 朝牆後 RangeSensor/ImuYawErrorDeg 是 ±180 附近，改成 +90。
   */
  private final LoggedNetworkNumber startYawToWallDeg = new LoggedNetworkNumber("/Tuning/RangeSensor/StartYawToWallDeg", -90.0);
  /** Elastic 按鈕：設 true → 以「現在車身相對牆面 = StartYawToWallDeg」設定基準，然後自動跳回 false。 */
  /** 開機後等多久（陀螺儀有讀值）再用開機擺放角度設定牆面基準。 */
  private static final double kBootSeedDelaySec = 0.5;
  private final LoggedNetworkBoolean seedWallHeadingNow = new LoggedNetworkBoolean("/Tuning/RangeSensor/SeedWallHeadingNow", false);

  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case kReal:
        drive = new Drive(new DriveIOSpark(), new GyroIOOnboardIMU());
        turret = new Turret(new TurretReal());
        vision = new Vision(new VisionIONT());
        turretLidar = new TurretLidar(new TurretLidarIONT(), () -> RangeSensors.get().getWallDistanceMeters(), () -> {
          var y = RangeSensors.get().getYaw();
          return y == null ? 0.0 : y.in(org.wpilib.units.Units.Radians) - Math.PI / 2;
        });
        break;
      
      case kSim:
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        turret = new Turret(new TurretSim());
        // 環境變數 VISION_IO=nt → 改讀 NT（測 Jetson / vision_fake_publisher.py），否則用模擬相機
        vision = "nt".equalsIgnoreCase(System.getenv("VISION_IO"))
            ? new Vision(new VisionIONT())
            : new Vision(new VisionIOSim(drive::getPose));
        // 預設用模擬光束（依位置算有沒有穿過洞）；環境變數 TURRET_LIDAR_IO=nt → 讀 NT（Jetson／假 publisher）
        turretLidar = new TurretLidar(
            "nt".equalsIgnoreCase(System.getenv("TURRET_LIDAR_IO")) ? new TurretLidarIONT() : new TurretLidarIOSim(drive::getPose),
            () -> RangeSensors.get().getWallDistanceMeters(), () -> {
          var y = RangeSensors.get().getYaw();
          return y == null ? 0.0 : y.in(org.wpilib.units.Units.Radians) - Math.PI / 2;
        });
        break;

      default:
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        turret = new Turret(new TurretIO() {});
        vision = new Vision(new VisionIO() {});
        turretLidar = new TurretLidar(new TurretLidarIO() {}, () -> RangeSensors.get().getWallDistanceMeters());
        break;
    }

    // 11/17 距離融合：車身 Luna ＋ IMU ＋ Limelight（一顆 Luna 被擋 → 另一顆＋IMU；兩顆都被擋 → Limelight）。
    // 要在建立任何 command 之前換上，command 建構時會抓 RangeSensors.get()。
    RangeSensors.installFusion(drive::getGyroYawRad, vision::getDistanceEstimate);

    NamedCommands.registerCommand("ColumnShot", Commands.sequence(
      new ColumnShot(turret, FieldConstants.kLowTargetHeight, 1),
      new ColumnShot(turret, FieldConstants.kMidTargetHeight, 1),
      new ColumnShot(turret, FieldConstants.kHighTargetHeight, 1)
    ));
    NamedCommands.registerCommand("Align", new LaserAlign(drive));
    for (VisionAlign.Target target : VisionAlign.Target.values()) {
      // VisionAlignLIT、VisionAlignLEFT、VisionAlignMID、VisionAlignRIGHT、VisionAlignNEAREST
      NamedCommands.registerCommand("VisionAlign" + target.name(), new VisionAlign(drive, vision, target));
    }

    // 預設「未選」：READY 燈不亮，逼操作手每場都選一次（§8.9）
    autoChooser.addDefault("（未選）", AutoOption.NONE);
    autoChooser.add("上半場-右起點", new AutoOption(Half.FIRST, StartSide.RIGHT));
    autoChooser.add("上半場-左起點", new AutoOption(Half.FIRST, StartSide.LEFT));
    autoChooser.add("下半場-右起點", new AutoOption(Half.SECOND, StartSide.RIGHT));
    autoChooser.add("下半場-左起點", new AutoOption(Half.SECOND, StartSide.LEFT));
    autoPathChooser.addDefault("跑路徑", true);
    autoPathChooser.add("不跑路徑（擺在目標欄前）", false);
    autoColumnChooser.addDefault("中欄", TargetColumn.MID);
    autoColumnChooser.add("己方欄（起點那一側）", TargetColumn.OWN_SIDE);
    alignChooser.addDefault("砲台Luna找洞（原地轉）", AlignMethod.LUNA_TURN);
    alignChooser.add("砲台Luna找洞（沿牆前後找）", AlignMethod.LUNA);
    alignChooser.add("視覺（最近一欄，原地轉）", AlignMethod.VISION);
    alignChooser.add("LaserAlign（擺正牆面）", AlignMethod.LASER);
    autoMethodChooser.addDefault("砲台Luna找洞（不用視覺）", AutoMethod.LUNA);
    autoMethodChooser.add("視覺（加分洞＋VisionAlign）", AutoMethod.VISION);
    autoMethodChooser.add("只盲走＋轉向（不瞄準、不射）", AutoMethod.APPROACH_ONLY);
    scoreboard = new Scoreboard(turret, vision, () -> currentPlan().half());
    // 開機就給 Auto/Stage 一個值，Elastic 的「Auto 階段」才不會是空的
    Logger.recordOutput("Auto/Stage", "Idle");

    controller = new CommandFlySkyController();
    new StatusBoard(drive, turret, vision, turretLidar, () -> controller.getHID().isConnected(),
        () -> autoChooser.get() != null && autoChooser.get().selected() ? currentPlan() : null);
    Logger.recordOutput("Mode", "disable");
    // Manual／Assist 射擊中（右開關開）左搖桿 X 是擊發、Y 往前是手動 Trigger → 底盤鎖住（速度 0），避免一擊發就轉向
    drive.setDefaultCommand(DriveCommands.arcadeDrive(drive,
        () -> holdDrive() ? 0 : controller.getLeftY(),
        () -> holdDrive() ? 0 : -controller.getLeftX()));

    // 右開關關著：RealManual／Feed 模式一律正轉；Manual、Assist 和 kDisable（未選模式、左開關剛關掉）都停。
    // 11/19 修 BUG：左開關關掉 → 模式變 kDisable，但 MockDS 停掉後機器人還會 enable 一小段時間，
    // 舊條件（只排除 Manual）會讓 Storage 全部轉起來。自動流程是在 kDisable 模式下跑的，所以跑自動時照轉。
    stroage.setDefaultCommand(new Normal(stroage, this::storageRunsByDefault));
    // 11/19：右開關打開（射擊模式）→ Storage 和 L298N Trigger 分開控制：
    // 偵測到射出（擊發 SparkMax 轉速掉到門檻以下）→ Trigger 單獨轉 kShotFeedPreTriggerSec（0.7 s）
    // → Trigger＋Storage 一起轉 kShotFeedTriggerSec（1.7 s）→ Storage 停、Trigger 繼續轉 kShotFeedStorageSec（0.7 s）→ 停
    // 11/20：自動流程自己會跑 ShootFeed（要求 stroage）→ 自動中撥右開關不能排這個，不然會把整個自動流程中斷
    // 11/20：Manual／Assist 射擊中左搖桿往前推（Y > 0；這支 FlySky 往上推是正值）→ 中斷 ShootFeed，改跑 TriggerRun（只有 L298N Trigger 一直轉）；
    // 放開後 Storage／Trigger 都停，等再射一發才重新啟動 ShootFeed（見下面 shotJustFired），或右開關關掉再打開
    controller.rightSwitch().and(isFeed().negate()).and(() -> !autoRunning)
        .whileTrue(new ShootFeed(stroage, turret::isTriggerStopped));
    // 搖桿放開後多撐 0.3 s 才停：L298N 一啟動電壓掉、FlySky 斷線一下時搖桿會歸零，不要因此把 TriggerRun 取消
    new Trigger(this::manualTriggerPushed).debounce(kManualTriggerReleaseSec, Debouncer.DebounceType.FALLING)
        .and(() -> !autoRunning).whileTrue(new TriggerRun(stroage));
    // 11/20：TriggerRun 放開後 ShootFeed 停著；之後再射一發（擊發馬達轉速掉下來那一刻）→ 重新啟動 ShootFeed，
    // 而且這一發就開始送料。shotJustFired 放最前面，每個週期都會被呼叫（and 會短路），上升緣才判斷得對。
    new Trigger(this::shotJustFired).and(this::manualOrAssistShooting).and(() -> !autoRunning)
        .and(() -> !manualTriggerPushed()).and(this::storageIdle)
        .onTrue(new ShootFeed(stroage, turret::isTriggerStopped, () -> false, true)
            .onlyWhile(() -> controller.getHID().getRightSwitch()));

    configureBindings();
  }

  private void configureBindings() {
    controller.leftSwitch().onTrue(mockDS.startMockDS()).onFalse(mockDS.stopMockDS());

    new Trigger(seedWallHeadingNow::get).onTrue(Commands.runOnce(() -> {
      seedWallHeading();
      seedWallHeadingNow.set(false);
    }).ignoringDisable(true));

    // 11/20：開機擺法 = 砲台朝右、車頭（Front frame）朝九宮格 → 車身 Luna 打到的是側邊，不是九宮格。
    // 不先設定基準的話，兩顆 Luna 讀到側牆（平的）會被學成「九宮格牆」，之後右側轉向九宮格時 IMU 偏角錯 90° 而且不會自己修正。
    // 開機等陀螺儀有讀值後，用 StartYawToWallDeg（開機擺放角度）設定基準；之後 IMU 偏角 > 45° 就不會拿側牆學習。
    CommandScheduler.getInstance().schedule(
        Commands.waitSeconds(kBootSeedDelaySec).andThen(Commands.runOnce(() -> {
          seedWallHeading();
          Logger.recordOutput("RangeSensor/BootSeeded", true);
        })).ignoringDisable(true).withName("BootSeedWallHeading"));

    // 自動流程：撥一下就啟動（onTrue）。開始時先把陀螺儀歸零（11/19），再用起點角度設定牆面基準。規則規定啟動後不能再碰遙控器，所以不能用 whileTrue（搖桿放開就會取消）。
    // 左開關關掉（MockDS 停止 → disable）時一併取消。
    // 11/20：觸發改成右搖桿右上（原本左上；送球模式改到左上）
    // 11/20 修 BUG：左開關打開那一刻 MockDS 才剛啟動、機器人還沒 enable，DeferredCommand 預設 runsWhenDisabled = false
    // → scheduler 直接丟掉，Auto 永遠不會開始（Auto/Stage 一直 Idle）。外層 ignoringDisable(true) 讓它排得進去，
    // 開頭先等真的 enable 了才歸零陀螺儀、開始動。
    controller.leftSwitch().and(righttop()).and(isDisable())
      .onTrue(Commands.defer(
        () -> Commands.runOnce(() -> Logger.recordOutput("Auto/Stage", "WaitEnable"))
            .andThen(Commands.waitUntil(org.wpilib.driverstation.RobotState::isEnabled))
            .andThen(Commands.runOnce(scoreboard::reset)).andThen(Commands.runOnce(drive::resetGyro)).andThen(Commands.runOnce(this::seedWallHeading)).andThen(buildAuto()).until(controller.leftSwitch().negate()).ignoringDisable(true)
            .beforeStarting(() -> autoRunning = true).finallyDo(() -> autoRunning = false),
        java.util.Set.of(drive, turret, stroage)
      ).ignoringDisable(true));

    controller.leftSwitch().and(midtop()).and(isDisable())
      .onTrue(Commands.runOnce(() -> {
        Constants.kRobotMode = RobotMode.kManual;
        Logger.recordOutput("Mode", "Manual");
      }).ignoringDisable(true)
    );
    // 要用真手動（仰角、轉速固定，只擊發）時，把上面的 kManual／"Manual" 換成 kRealManual／"RealManual"
    controller.leftSwitch().and(midmid()).and(isDisable())
      .onTrue(Commands.runOnce(() -> {
        Constants.kRobotMode = RobotMode.kAssist;
        Logger.recordOutput("Mode", "Assist");
      }).ignoringDisable(true)
    );
    // 11/20：右搖桿左上 → 送球模式（Storage＋L298N Trigger 不停正轉；左開關關掉就停）。原本右上，右上改給自動流程。
    controller.leftSwitch().and(lefttop()).and(isDisable())
      .onTrue(Commands.runOnce(() -> {
        Constants.kRobotMode = RobotMode.kFeed;
        Logger.recordOutput("Mode", "Feed");
      }).ignoringDisable(true)
    );
    // 11/20：kAuto 模式（右搖桿正下）已拿掉

    // isDisable 時 lefttop 是選送球模式用的，不觸發對齊
    // 11/19：右搖桿左上 = 對齊（Elastic「對齊方式」選：砲台Luna找洞／VisionAlign／LaserAlign）。
    // 只要求底盤，所以 Assist 模式射擊中按也不會中斷射擊。
    isManual().negate().and(isRealManual().negate()).and(isDisable().negate()).and(isFeed().negate()).and(lefttop())
      .onTrue(Commands.defer(this::buildAlign, java.util.Set.of(drive)).until(this::leftStickOverride));

    controller.rightSwitch().and(isAssist())
      .whileTrue(
        Commands.parallel(
          // 11/19：射大洞。右搖桿 左下=低、右下=中、右上=高（記住）；右搖桿 Y 微調仰角；左搖桿往右推擊發（同 Manual）
          new AssistTurret(turret, () -> controller.getRightY(), () -> controller.getLeftX() > 0.3,
            righttop(), rightdown(), leftdown())
          // 11/19：不再佔用底盤（Commands.idle(drive)），對齊才不會把射擊打斷；底盤預設指令在 Assist 射擊時不吃搖桿
        )
      );

    // 真手動：仰角、轉速固定，左搖桿 X 往右推 = 擊發
    controller.rightSwitch().and(isRealManual())
      .whileTrue(
        Commands.parallel(
          new RealManualTurret(turret, () -> controller.getLeftX() > 0.3),
          Commands.idle(drive)
        )
      );

    // 11/20：底盤不再用 Commands.idle（不送指令，馬達會維持最後的速度）→ 交給底盤預設指令，holdDrive() 時送速度 0 鎖住
    controller.rightSwitch().and(isManual())
      .whileTrue(
        new ManualTurret(turret, () -> controller.getRightY(), () -> controller.getLeftX() > 0.3,
          () -> turretLidar.getDistanceMeters().orElse(Double.NaN))
      );
    
    
  }

  /** 右開關關著時 Storage／L298N Trigger 要不要一直正轉。 */
  /** 車子擺在已知角度（起點）時呼叫：IMU 牆面基準 = 現在角度 − StartYawToWallDeg。之後 Luna×2 一致時會再慢慢修正。 */
  private void seedWallHeading() {
    RangeSensors.seedWallHeading(startYawToWallDeg.get());
  }

  /** 11/20：Manual／Assist 射擊中左搖桿 Y 推過這個值（往前推，Y 為正：FlySky 讀值 = (µs − 1500)/500，往上推 µs 變大）→ L298N Trigger 手動正轉（留一點死區，避免搖桿回中時抖動）。 */
  private static final double kManualTriggerThreshold = 0.3;
  /** 手動 Trigger 放開後多撐多久才停（FlySky 短暫斷線時搖桿會歸零）。 */
  private static final double kManualTriggerReleaseSec = 0.3;

  /**
   * 11/20：搖桿對齊時左搖桿有輸入 → 中斷對齊。
   * Assist 射擊中左搖桿 X 是擊發、往前推（Y > 0）是手動 Trigger，都不算；只有往後拉（Y < 0）會中斷。
   */
  private static final double kAlignOverrideThreshold = 0.15;

  private boolean leftStickOverride() {
    boolean y = assistShooting()
        ? controller.getLeftY() < -kAlignOverrideThreshold
        : Math.abs(controller.getLeftY()) > kAlignOverrideThreshold;
    boolean x = !assistShooting() && Math.abs(controller.getLeftX()) > kAlignOverrideThreshold;
    return x || y;
  }

  private boolean assistShooting() {
    return Constants.kRobotMode == RobotMode.kAssist && controller.getHID().getRightSwitch();
  }

  /** Manual 或 Assist 模式、右開關打開（射擊狀態）。 */
  private boolean manualOrAssistShooting() {
    return (Constants.kRobotMode == RobotMode.kManual || Constants.kRobotMode == RobotMode.kAssist)
        && controller.getHID().getRightSwitch();
  }

  /** Manual／Assist 射擊中左搖桿往前推 → TriggerRun。 */
  private boolean manualTriggerPushed() {
    return manualOrAssistShooting() && controller.getLeftY() > kManualTriggerThreshold;
  }

  private boolean lastTriggerStopped = true;

  /** 擊發馬達轉速剛從高於門檻掉到以下（= 剛射出一顆）。每個週期只能呼叫一次。 */
  private boolean shotJustFired() {
    boolean stopped = turret.isTriggerStopped();
    boolean fired = stopped && !lastTriggerStopped;
    lastTriggerStopped = stopped;
    return fired;
  }

  /** Storage 沒有 ShootFeed／TriggerRun 在跑（只有預設指令）。 */
  private boolean storageIdle() {
    Command current = stroage.getCurrentCommand();
    return current == null || current == stroage.getDefaultCommand();
  }

  /** 底盤鎖住（速度 0、不吃搖桿）：Manual／Assist 射擊中、送球模式。 */
  private boolean holdDrive() {
    return manualOrAssistShooting() || Constants.kRobotMode == RobotMode.kFeed;
  }

  private boolean storageRunsByDefault() {
    return switch (Constants.kRobotMode) {
      case kRealManual, kFeed -> true;
      case kManual, kAssist -> false; // 11/19：Assist 和 Manual 一樣，右開關關著就停
      case kDisable -> autoRunning || org.wpilib.driverstation.RobotState.isAutonomous();
    };
  }

  public Trigger isDisable() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kDisable);
  }

  public Trigger isManual() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kManual);
  }

  public Trigger isFeed() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kFeed);
  }

  public Trigger isRealManual() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kRealManual);
  }

  public Trigger isAssist() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kAssist);
  }

  public Trigger midmid() {
    return new Trigger(() -> MathUtil.isNear(0, controller.getRightX(), 0.3) &&  MathUtil.isNear(0, controller.getRightY(), 0.3) );
  }

  public Trigger midtop() {
    return new Trigger(() -> MathUtil.isNear(0, controller.getRightX(), 0.3) && controller.getRightY() > 0.7);
  }

  public Trigger middown() {
    return new Trigger(() -> MathUtil.isNear(0, controller.getRightX(), 0.3) && controller.getRightY() < -0.7);
  }

  public Trigger righttop() {
    return new Trigger(() -> controller.getRightX() > 0.5 && controller.getRightY() > 0.5);
  }
  
  public Trigger rightdown() {
    return new Trigger(() -> controller.getRightX() > 0.5 && controller.getRightY() < -0.5);
  }

  public Trigger lefttop() {
    return new Trigger(() -> controller.getRightX() < -0.5 && controller.getRightY() > 0.5);
  }
  
  public Trigger leftdown() {
    return new Trigger(() -> controller.getRightX() < -0.5 && controller.getRightY() < -0.5);
  }

  private Command buildAlign() {
    AlignMethod m = alignChooser.get() == null ? AlignMethod.LUNA_TURN : alignChooser.get();
    Logger.recordOutput("Align/Method", m.name());
    return switch (m) {
      case LUNA_TURN -> DriveCommands.lunaTurnAlign(drive, turretLidar);
      case LUNA -> DriveCommands.lunaCenterAlign(drive, turretLidar);
      case VISION -> new frc.robot.commands.drive.VisionTurnAlign(drive, vision, VisionAlign.Target.NEAREST);
      case LASER -> new LaserAlign(drive);
    };
  }

  /** 依選單產生新的自動流程指令（同一個指令物件不能重複組合，所以每次都重建）。 */
  private Command buildAuto() {
    return AutoRoutines.build(drive, vision, turret, turretLidar, stroage, currentPlan(), startYawToWallDeg.get());
  }

  /**
   * 目前選單組成的自動流程設定。沒選半場／起點時用最安全的設定：上半場、不跑路徑、中欄
   * （不知道起點在哪一側時不能跑路徑）。
   */
  private Plan currentPlan() {
    AutoOption option = autoChooser.get();
    AutoMethod method = autoMethodChooser.get() == null ? AutoMethod.LUNA : autoMethodChooser.get();
    if (option == null || !option.selected()) {
      return new Plan(Half.FIRST, StartSide.RIGHT, false, TargetColumn.MID, method);
    }
    Boolean runPath = autoPathChooser.get();
    TargetColumn column = autoColumnChooser.get();
    return new Plan(option.half(), option.side(), runPath == null || runPath,
        column == null ? TargetColumn.MID : column, method);
  }

  /** 只有在 DS 進 autonomous 模式時才會用到（目前用 FlySky 開關 + MockDS 的 teleop 啟動，不會進這裡）。 */
  public Command getAutonomousCommand() {
    return buildAuto();
  }
}
