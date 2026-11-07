// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.FieldConstants.*;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import org.wpilib.math.geometry.Translation3d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  
  private final RobotContainer m_robotContainer;

  public Robot() {
    super(Constants.kDefaultPeriod);
    Logger.recordMetadata("ProjectName", "TEL_2025"); 
    switch (Constants.kCurrentMode) {
      case kReal:
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case kSim:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case kReplay:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // URCL has no 2027 / SystemCore build yet; re-add Logger.registerURCL(...) once available.
    Logger.start();
    
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void simulationInit() {
    NetworkTableInstance.getDefault()
      .getTable("Field")
      .getStructArrayTopic("NineGrid", Translation3d.struct)
      .publish()
      .set(new Translation3d[] {
        new Translation3d(kTargetX, kLeftTargetY, kLowTargetHeight),
        new Translation3d(kTargetX, kLeftTargetY, kMidTargetHeight),
        new Translation3d(kTargetX, kLeftTargetY, kHighTargetHeight),
        new Translation3d(kTargetX, kMidTargetY, kLowTargetHeight),
        new Translation3d(kTargetX, kMidTargetY, kMidTargetHeight),
        new Translation3d(kTargetX, kMidTargetY, kHighTargetHeight),
        new Translation3d(kTargetX, kRightTargetY, kLowTargetHeight),
        new Translation3d(kTargetX, kRightTargetY, kMidTargetHeight),
        new Translation3d(kTargetX, kRightTargetY, kHighTargetHeight),
        new Translation3d(kTargetX, kMidTargetY, kSpecialTargetHeight)
      });
    NetworkTableInstance.getDefault()
      .getTable("Field")
      .getStructArrayTopic("RobotOperationArea", Translation3d.struct)
      .publish()
      .set(new Translation3d[] {
        new Translation3d(0, 0, 0),
        new Translation3d(5, 0, 0),
        new Translation3d(5, 4, 0),
        new Translation3d(0, 4, 0),
        new Translation3d(0, 0, 0)
      });
  }

  @Override
  public void simulationPeriodic() {
    // MapleSim (SimulatedArena) has no 2027 / SystemCore build yet; game-piece sim disabled.
  }
}
