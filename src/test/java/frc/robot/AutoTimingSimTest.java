package frc.robot;

import static frc.robot.Constants.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.drive.VisionAlign;
import frc.robot.commands.turret.ColumnShot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOSim;

/**
 * 量自動流程每一段在模擬裡花多久，用來估 60 秒內能做完多少事。
 *
 * <p>注意：TurretSim 的飛輪是瞬間到速，實機每次 ColumnShot 還要加上飛輪加速時間。
 * 執行：./gradlew test --tests "*AutoTimingSimTest" -i
 */
class AutoTimingSimTest {
  private static final double kDt = Constants.kDefaultPeriod;

  private Drive drive;
  private Turret turret;
  private Vision vision;
  private final List<String> report = new ArrayList<>();

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void teardown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  /** 跑一個指令直到結束（最多 timeout 秒），回傳花的秒數。 */
  private double runTimed(String name, Command cmd, double timeout) {
    CommandScheduler.getInstance().schedule(cmd);
    double t = 0;
    do {
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
      t += kDt;
    } while (cmd.isScheduled() && t < timeout);
    Pose2d p = drive.getPose();
    String line = String.format("%-28s %5.2f s%s  pose=(%.2f, %.2f, %.0f°)", name, t,
        t >= timeout ? "（逾時）" : "", p.getX(), p.getY(), p.getRotation().getDegrees());
    report.add(line);
    return t;
  }

  @Test
  void measureSegments() {
    drive = new Drive(new DriveIOSim(), new GyroIO() {});
    turret = new Turret(new TurretSim());
    vision = new Vision(new VisionIOSim(drive::getPose, 0, 7130)); // 亮左欄

    double total = 0;
    total += runTimed("RS_C2_Auto（RS→中欄+LaserAlign）", new PathPlannerAuto("RS_C2_Auto"), 15);
    total += runTimed("VisionAlign LIT（中→左欄）", new VisionAlign(drive, vision, VisionAlign.Target.LIT), 7);
    total += runTimed("ColumnShot 低 x1", new ColumnShot(turret, kLowTargetHeight, 1), 10);
    total += runTimed("ColumnShot 中 x1", new ColumnShot(turret, kMidTargetHeight, 1), 10);
    total += runTimed("ColumnShot 高 x1", new ColumnShot(turret, kHighTargetHeight, 1), 10);
    total += runTimed("ColumnShot 加分洞 x3", new ColumnShot(turret, kSpecialTargetHeight, 3), 15);
    total += runTimed("VisionAlign MID（左→中欄）", new VisionAlign(drive, vision, VisionAlign.Target.MID), 7);
    total += runTimed("VisionAlign RIGHT（中→右欄）", new VisionAlign(drive, vision, VisionAlign.Target.RIGHT), 7);

    System.out.println("===== 自動流程分段時間（模擬，不含飛輪加速） =====");
    report.forEach(System.out::println);
    System.out.printf("合計 %.2f s%n", total);
  }
}
