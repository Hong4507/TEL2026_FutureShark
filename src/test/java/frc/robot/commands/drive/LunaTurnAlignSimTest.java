package frc.robot.commands.drive;

import static frc.robot.Constants.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.OptionalDouble;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.turretlidar.TurretLidar;
import frc.robot.subsystems.turretlidar.TurretLidarIOSim;

/** 11/20 右搖桿左上「砲台Luna找洞（原地轉）」：不沿牆走，轉到光束打在最近那一欄的洞中間。 */
class LunaTurnAlignSimTest {
  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
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

  @ParameterizedTest(name = "從 y={0} 原地轉 → 光束打在欄 y={1}（洞後箱子 {2} m）")
  @CsvSource({
    "0.80, 1.1, NaN",   // 洞在逆時針那側
    "1.45, 1.1, NaN",   // 洞在順時針那側：逆時針掃不到 → 轉回來再掃另一邊
    "1.05, 1.1, NaN",   // 一開始就在洞裡
    "2.30, 2.0, NaN",
    "0.80, 1.1, 0.15",  // 洞後面有箱子
    "1.45, 1.1, 0.15",
  })
  void turnsBeamOntoNearestHole(double startY, double expectedY, double boxBehind) {
    Drive drive = new Drive(new DriveIOSim(), new GyroIO() {});
    drive.setPose(new Pose2d(kTargetX - TurretLidar.idealHorizontalMeters(1), startY, Rotation2d.fromDegrees(90)));
    TurretLidar lidar = new TurretLidar(new TurretLidarIOSim(drive::getPose, boxBehind),
        () -> OptionalDouble.of(kTargetX - drive.getPose().getX() - TurretLidar.kInboardMeters),
        () -> drive.getPose().getRotation().getRadians() - Math.PI / 2);
    for (int i = 0; i < 20; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(Constants.kDefaultPeriod);
    }
    Pose2d start = drive.getPose();
    Command cmd = DriveCommands.lunaTurnAlign(drive, lidar);
    CommandScheduler.getInstance().schedule(cmd);
    double t = 0;
    while (cmd.isScheduled() && t < 30) {
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(Constants.kDefaultPeriod);
      t += Constants.kDefaultPeriod;
    }
    for (int i = 0; i < 30; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(Constants.kDefaultPeriod);
    }
    Pose2d p = drive.getPose();
    // 光束（機器人右側方向）打到板面時的 y
    double h = p.getRotation().getRadians();
    double horizontal = (kTargetX - p.getX()) / Math.sin(h);
    double hitY = p.getY() - Math.cos(h) * horizontal;
    double moved = p.getTranslation().getDistance(start.getTranslation());
    System.out.printf("LunaTurnAlign y0=%.2f → 光點 %.3f、轉 %.1f°、位移 %.3f m（%.1f s）%n",
        startY, hitY, Math.toDegrees(h) - 90, moved, t);
    assertTrue(t < 30);
    assertTrue(moved < 0.05, "原地轉不應該沿牆走");
    assertEquals(expectedY, hitY, 0.06);
  }
}
