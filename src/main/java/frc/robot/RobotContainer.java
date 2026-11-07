// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import org.wpilib.math.util.MathUtil;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotMode;
import frc.robot.commands.drive.LaserAlign;
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
import frc.robot.subsystems.storage.Stroage;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretReal;
import frc.robot.subsystems.turret.TurretSim;

public class RobotContainer {
  private final Drive drive;
  private final Turret turret;
  private final Stroage stroage = new Stroage();

  private CommandFlySkyController controller;

  private final MockDS mockDS = new MockDS();

  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case kReal:
        drive = new Drive(new DriveIOSpark(), new GyroIOOnboardIMU());
        turret = new Turret(new TurretReal());
        break;
      
      case kSim:
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        turret = new Turret(new TurretSim());
        break;

      default:
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        turret = new Turret(new TurretIO() {});
        break;
    }

    NamedCommands.registerCommand("ColumnShot", Commands.sequence(
      new ColumnShot(turret, FieldConstants.kLowTargetHeight, 1),
      new ColumnShot(turret, FieldConstants.kMidTargetHeight, 1),
      new ColumnShot(turret, FieldConstants.kHighTargetHeight, 1)
    ));
    NamedCommands.registerCommand("Align", new LaserAlign(drive));

    controller = new CommandFlySkyController();
    drive.setDefaultCommand(DriveCommands.arcadeDrive(drive, () -> controller.getLeftY(), () -> -controller.getLeftX()));

    stroage.setDefaultCommand(new Normal(stroage));

    configureBindings();
  }

  private void configureBindings() {
    controller.leftSwitch().onTrue(mockDS.startMockDS()).onFalse(mockDS.stopMockDS());

    controller.leftSwitch().and(lefttop()).and(isDisable())
      .whileTrue(new PathPlannerAuto("RS_C2_Auto").ignoringDisable(true));

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
    
    isManual().negate().and(lefttop())
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

  public Command getAutonomousCommand() {
    return null;
  }
}
