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

/** 右搖桿左上「砲台Luna找洞」：先往車頭找 0.6 m，沒找到回頭往車尾找；停在洞中間。 */
class LunaCenterAlignSimTest {
  private Drive drive;

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

  @ParameterizedTest(name = "從 y={0} 出發 → 停在欄 y={1}（洞後箱子 {2} m）")
  @CsvSource({
    "0.80, 1.1, NaN",   // 洞在車頭方向
    "1.45, 1.1, NaN",   // 剛過洞：往前找不到 → 回頭
    "1.05, 1.1, NaN",   // 一開始就在洞裡
    "2.30, 2.0, NaN",   // 剛過中欄
    "0.80, 1.1, 0.15",  // 洞後面有箱子
    "1.45, 1.1, 0.15",
  })
  void centersOnNearestHole(double startY, double expectedY, double boxBehind) {
    drive = new Drive(new DriveIOSim(), new GyroIO() {});
    drive.setPose(new Pose2d(kTargetX - TurretLidar.idealHorizontalMeters(1), startY, Rotation2d.fromDegrees(90)));
    TurretLidar lidar = new TurretLidar(new TurretLidarIOSim(() -> drive.getPose(), boxBehind),
        () -> OptionalDouble.of(kTargetX - drive.getPose().getX() - TurretLidar.kInboardMeters));
    for (int i = 0; i < 20; i++) { // 實機 TurretLidar 一直在跑；測試先讓它更新（超過 debounce 0.06 s）
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(Constants.kDefaultPeriod);
    }
    Command cmd = DriveCommands.lunaCenterAlign(drive, lidar);
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
    double y = drive.getPose().getY();
    System.out.printf("LunaCenterAlign y0=%.2f → %.3f（%.1f s）%n", startY, y, t);
    assertTrue(t < 30);
    assertEquals(expectedY, y, 0.03);
  }
}
