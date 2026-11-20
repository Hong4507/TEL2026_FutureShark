package frc.robot.commands.drive;

import static frc.robot.Constants.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

import frc.FSLib.rangefinder.RangeSensorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOSim;

/**
 * 11/20 VisionTurnAlign 閉迴路模擬：原地轉對準，檢查「從射球點沿射擊方向（機器人右側）畫線」打到九宮格的位置，
 * 而且機器人沒有沿牆移動。設定同 {@link VisionAlignSimTest}（攝影機在機器人中心、距離用 fallback 3.0 m）。
 */
class VisionTurnAlignSimTest {
  private static final double kDt = frc.robot.Constants.kDefaultPeriod;
  private static final double kFinalTolerance = 0.05;

  private Drive drive;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // 幾何測試：模擬是理想的，不加實機瞄準修正
    VisionTurnAlign.aimTrimDeg = () -> 0;
  }

  @AfterEach
  void teardown() {
    VisionTurnAlign.aimTrimDeg = () -> 0;
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  private static double columnY(VisionAlign.Target target, int litColumn) {
    return switch (target) {
      case LEFT -> kLeftTargetY;
      case MID -> kMidTargetY;
      case RIGHT -> kRightTargetY;
      case LIT -> new double[] {kLeftTargetY, kMidTargetY, kRightTargetY}[litColumn];
      case NEAREST -> Double.NaN;
    };
  }

  /** 射擊線打到九宮格平面時和目標欄中心差多少。 */
  private double hitError(double targetY) {
    Pose2d pose = drive.getPose();
    double h = pose.getRotation().getRadians();
    double fwd = VisionConstants.kShooterForwardOfCameraMeters;
    double sx = pose.getX() + fwd * Math.cos(h), sy = pose.getY() + fwd * Math.sin(h);
    double dx = Math.sin(h), dy = -Math.cos(h);
    double t = (kTargetX - sx) / dx;
    return sy + dy * t - targetY;
  }

  @ParameterizedTest(name = "{0} 從 y={1} 原地轉")
  @CsvSource({
    "LEFT, 2.4", "LEFT, 2.9", "LEFT, 3.3",
    "MID, 1.5", "MID, 2.0", "MID, 2.4",
    "RIGHT, 0.6", "RIGHT, 1.0", "RIGHT, 1.4",
    "LIT, 1.6",
  })
  void turnsOntoColumn(VisionAlign.Target target, double startY) {
    double err = alignAndHitError(target, startY);
    assertEquals(0, err, kFinalTolerance);
  }

  /** 11/20：AimTrimDeg = 正 → 對準完再順時針多轉 → 落點往車尾（−y）移 射擊距離 × tan(修正角)。 */
  @ParameterizedTest(name = "{0} 從 y={1}，修正 {2}°")
  @CsvSource({
    "MID, 2.0, 1.0",
    "MID, 2.4, 2.0",
    "RIGHT, 1.0, 1.0",
  })
  void aimTrimTurnsClockwise(VisionAlign.Target target, double startY, double trimDeg) {
    double untrimmed = alignAndHitError(target, startY);
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    VisionTurnAlign.aimTrimDeg = () -> trimDeg;
    double trimmed = alignAndHitError(target, startY);
    double expected = -RangeSensorConstants.kFallbackDistanceMeters * Math.tan(Math.toRadians(trimDeg));
    assertEquals(expected, trimmed - untrimmed, 0.02, "落點應該往車尾移");
  }

  private double alignAndHitError(VisionAlign.Target target, double startY) {
    int litColumn = 0;
    double distance = RangeSensorConstants.kFallbackDistanceMeters;
    drive = new Drive(new DriveIOSim(), new GyroIO() {});
    drive.setPose(new Pose2d(kTargetX - distance, startY, Rotation2d.fromDegrees(90)));
    Vision vision = new Vision(new VisionIOSim(drive::getPose, litColumn, 7130));
    for (int i = 0; i < 5; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
    }
    Pose2d start = drive.getPose();
    VisionTurnAlign cmd = new VisionTurnAlign(drive, vision, target);
    CommandScheduler.getInstance().schedule(cmd);
    double t = 0;
    while (cmd.isScheduled() && t < 9.0) {
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
      t += kDt;
    }
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
    }
    double err = hitError(columnY(target, litColumn));
    double moved = drive.getPose().getTranslation().getDistance(start.getTranslation());
    System.out.printf("%-6s start=%.2f  落點誤差=%+.3f m  轉 %.1f°  位移 %.3f m  time=%.2f s  success=%b%n",
        target, startY, err, drive.getPose().getRotation().getDegrees() - 90, moved, t, cmd.succeeded());
    assertTrue(cmd.succeeded(), "應該對準成功");
    assertTrue(moved < 0.05, "原地轉不應該沿牆走");
    return err;
  }
}
