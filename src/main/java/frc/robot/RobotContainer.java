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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotMode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.AutoRoutines.Half;
import frc.robot.commands.drive.LaserAlign;
import frc.robot.commands.drive.VisionAlign;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.storage.Normal;
import frc.robot.commands.storage.StorageCommands;
import frc.robot.commands.turret.AssistTurret;
import frc.robot.commands.turret.ColumnShot;
import frc.robot.commands.turret.ManualTurret;
import frc.robot.controller.CommandFlySkyController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOOnboardIMU;
import frc.robot.subsystems.scoreboard.Scoreboard;
import frc.robot.subsystems.storage.Stroage;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretReal;
import frc.robot.subsystems.turret.TurretSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIONT;
import frc.robot.subsystems.vision.VisionIOSim;

public class RobotContainer {
  private final Drive drive;
  private final Turret turret;
  private final Vision vision;
  private final Stroage stroage = new Stroage();

  private CommandFlySkyController controller;

  private final MockDS mockDS = new MockDS();

  /** 前 60 秒自動流程選單（筆電 Dashboard 選，比賽前 90 秒準備時間設定）。 */
  private final LoggedNetworkChooser<AutoOption> autoChooser = new LoggedNetworkChooser<>("Auto Choices");

  /** 自動選項：哪個半場（記分板用來決定每格上限）+ 產生指令的方法。 */
  private record AutoOption(Half half, String startAuto) {}

  private Scoreboard scoreboard;

  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case kReal:
        drive = new Drive(new DriveIOSpark(), new GyroIOOnboardIMU());
        turret = new Turret(new TurretReal());
        vision = new Vision(new VisionIONT());
        break;
      
      case kSim:
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        turret = new Turret(new TurretSim());
        // 環境變數 VISION_IO=nt → 改讀 NT（測 Jetson / vision_fake_publisher.py），否則用模擬相機
        vision = "nt".equalsIgnoreCase(System.getenv("VISION_IO"))
            ? new Vision(new VisionIONT())
            : new Vision(new VisionIOSim(drive::getPose));
        break;

      default:
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        turret = new Turret(new TurretIO() {});
        vision = new Vision(new VisionIO() {});
        break;
    }

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

    autoChooser.addDefault("上半場-右起點", new AutoOption(Half.FIRST, "RS_C2_Auto"));
    autoChooser.add("上半場-左起點", new AutoOption(Half.FIRST, "LS_C2_Auto"));
    autoChooser.add("下半場-右起點", new AutoOption(Half.SECOND, "RS_C2_Auto"));
    autoChooser.add("下半場-左起點", new AutoOption(Half.SECOND, "LS_C2_Auto"));
    scoreboard = new Scoreboard(turret, vision, () -> autoChooser.get().half());
    // 開機就給 Auto/Stage 一個值，Elastic 的「Auto 階段」才不會是空的
    Logger.recordOutput("Auto/Stage", "Idle");

    controller = new CommandFlySkyController();
    drive.setDefaultCommand(DriveCommands.arcadeDrive(drive, () -> controller.getLeftY(), () -> -controller.getLeftX()));

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
    isManual().negate().and(isDisable().negate()).and(lefttop())
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

    controller.rightSwitch().and(isManual())
      .whileTrue(
        Commands.parallel(
          new ManualTurret(turret, () -> controller.getRightY(), () -> controller.getLeftX() > 0.3),
          Commands.idle(drive)
        )
      );
    
    
    turret.ballFired().onTrue(StorageCommands.reload(stroage));
  }

  public Trigger isDisable() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kDisable);
  }

  public Trigger isManual() {
    return new Trigger(() -> Constants.kRobotMode == RobotMode.kManual);
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
    AutoOption option = autoChooser.get();
    return AutoRoutines.bonusThenColumn(drive, vision, turret, option.half(), option.startAuto());
  }

  /** 只有在 DS 進 autonomous 模式時才會用到（目前用 FlySky 開關 + MockDS 的 teleop 啟動，不會進這裡）。 */
  public Command getAutonomousCommand() {
    return buildAuto();
  }
}
