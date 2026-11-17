// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;

import org.wpilib.math.util.MathUtil;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
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

  /** 自動選項：哪個半場（記分板用來決定每格上限）+ 起點。half == null 代表還沒選（READY 燈不會亮）。 */
  private record AutoOption(Half half, StartSide side) {
    static final AutoOption NONE = new AutoOption(null, null);

    boolean selected() {
      return half != null;
    }
  }

  private Scoreboard scoreboard;

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
    autoMethodChooser.addDefault("砲台Luna找洞（不用視覺）", AutoMethod.LUNA);
    autoMethodChooser.add("視覺（加分洞＋VisionAlign）", AutoMethod.VISION);
    scoreboard = new Scoreboard(turret, vision, () -> currentPlan().half());
    // 開機就給 Auto/Stage 一個值，Elastic 的「Auto 階段」才不會是空的
    Logger.recordOutput("Auto/Stage", "Idle");

    controller = new CommandFlySkyController();
    new StatusBoard(drive, turret, vision, turretLidar, () -> controller.getHID().isConnected(),
        () -> autoChooser.get() != null && autoChooser.get().selected() ? currentPlan() : null);
    Logger.recordOutput("Mode", "disable");
    drive.setDefaultCommand(DriveCommands.arcadeDrive(drive, () -> controller.getLeftY(), () -> -controller.getLeftX()));

    // L298N 上的馬達（Storage 左右 + L298N Trigger 通道）一律正轉；只靠砲台 NEO550 擊發
    stroage.setDefaultCommand(new Normal(stroage));

    configureBindings();
  }

  private void configureBindings() {
    controller.leftSwitch().onTrue(mockDS.startMockDS()).onFalse(mockDS.stopMockDS());

    // 自動流程：撥一下就啟動（onTrue）。規則規定啟動後不能再碰遙控器，所以不能用 whileTrue（搖桿放開就會取消）。
    // 左開關關掉（MockDS 停止 → disable）時一併取消。
    controller.leftSwitch().and(lefttop()).and(isDisable())
      .onTrue(Commands.defer(
        () -> Commands.runOnce(scoreboard::reset).andThen(buildAuto()).until(controller.leftSwitch().negate()).ignoringDisable(true),
        java.util.Set.of(drive, turret)
      ));

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
    controller.leftSwitch().and(middown()).and(isDisable())
      .onTrue(Commands.runOnce(() -> {
        Constants.kRobotMode = RobotMode.kAuto;
        Logger.recordOutput("Mode", "Auto");
      }).ignoringDisable(true)
    );
    
    // isDisable 時 lefttop 是啟動自動流程用的，不能同時觸發 LaserAlign（兩個都要 drive，會互相打斷）
    isManual().negate().and(isRealManual().negate()).and(isDisable().negate()).and(lefttop())
      .onTrue(new LaserAlign(drive));

    controller.rightSwitch().and(isAuto()).and(righttop())
      .whileTrue(
        Commands.sequence(
          new ColumnShot(turret, FieldConstants.kLowTargetHeight, 2),
          new ColumnShot(turret, FieldConstants.kMidTargetHeight, 2),
          new ColumnShot(turret, FieldConstants.kHighTargetHeight, 2)
        )
      );

    controller.rightSwitch().and(isAuto()).and(leftdown())
      .whileTrue(new ColumnShot(turret, FieldConstants.kLowTargetHeight, 1));
    
    controller.rightSwitch().and(isAuto()).and(middown())
      .whileTrue(new ColumnShot(turret, FieldConstants.kMidTargetHeight, 1));
      
    controller.rightSwitch().and(isAuto()).and(rightdown())
      .whileTrue(new ColumnShot(turret, FieldConstants.kHighTargetHeight, 1));

    controller.rightSwitch().and(isAssist())
      .whileTrue(
        Commands.parallel(
          new AssistTurret(turret, FieldConstants.kSpecialTargetHeight, () -> controller.getRightY(), () -> controller.getLeftX() > 0.3),
          Commands.idle(drive)
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

    controller.rightSwitch().and(isManual())
      .whileTrue(
        Commands.parallel(
          new ManualTurret(turret, () -> controller.getRightY(), () -> controller.getLeftX() > 0.3,
            () -> turretLidar.getDistanceMeters().orElse(Double.NaN)),
          Commands.idle(drive)
        )
      );
    
    
  }

  public Trigger isDisable() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kDisable);
  }

  public Trigger isManual() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kManual);
  }

  public Trigger isRealManual() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kRealManual);
  }

  public Trigger isAssist() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kAssist);
  }

  public Trigger isAuto() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kAuto);
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

  /** 依選單產生新的自動流程指令（同一個指令物件不能重複組合，所以每次都重建）。 */
  private Command buildAuto() {
    return AutoRoutines.build(drive, vision, turret, turretLidar, currentPlan());
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
