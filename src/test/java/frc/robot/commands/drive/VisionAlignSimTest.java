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
 * VisionAlign 閉迴路模擬：DriveIOSim + VisionIOSim，從不同起點跑到指定欄，檢查射球點最後有沒有對準。
 *
 * <p>機器人朝 +y（車頭朝左欄）、攝影機朝右（+x）看九宮格。模擬讀不到 TF-Luna，距離一律是
 * {@link RangeSensorConstants#kFallbackDistanceMeters}，所以預設把攝影機放在離九宮格這個距離的地方。
 */
class VisionAlignSimTest {
  private static final double kDt = frc.robot.Constants.kDefaultPeriod; // DriveIOSim 每次 update 走這麼久，要一致
  /** 最後射球點與目標欄中心的容許誤差。VisionAlign 本身容許 2 cm，加上量測雜訊留一點空間。 */
  private static final double kFinalTolerance = 0.04;

  private Drive drive;
  private Vision vision;

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

  private static double columnY(VisionAlign.Target target, int litColumn) {
    return switch (target) {
      case LEFT -> kLeftTargetY;
      case MID -> kMidTargetY;
      case RIGHT -> kRightTargetY;
      case LIT -> new double[] {kLeftTargetY, kMidTargetY, kRightTargetY}[litColumn];
      case NEAREST -> Double.NaN;
    };
  }

  /** 射球點的 y：攝影機在機器人中心，射球點在前方（+y）kShooterForwardOfCameraMeters。 */
  private double shooterY() {
    Pose2d pose = drive.getPose();
    return pose.getY() + VisionConstants.kShooterForwardOfCameraMeters * pose.getRotation().getSin();
  }

  /** finished：指令自己結束（沒被測試的 7 秒上限切掉）。 */
  /**
   * 球的落點誤差：砲台只有俯仰、固定朝機器人右側射，從射球點沿這個方向畫線，
   * 看它打到九宮格平面（x = kTargetX）時的 y 和目標欄中心差多少。車身擺正時等於 shooterY() 的誤差。
   */
  private double hitError(double targetY) {
    Pose2d pose = drive.getPose();
    double h = pose.getRotation().getRadians();
    double fwd = VisionConstants.kShooterForwardOfCameraMeters;
    double sx = pose.getX() + fwd * Math.cos(h), sy = pose.getY() + fwd * Math.sin(h);
    double dx = Math.sin(h), dy = -Math.cos(h); // 機器人右側
    double t = (kTargetX - sx) / dx;
    return sy + dy * t - targetY;
  }

  private record Result(boolean finished, double finalError, double hitError, double seconds) {}

  private Result run(VisionAlign.Target target, double startY, double distance, double headingDeg, int litColumn) {
    drive = new Drive(new DriveIOSim(), new GyroIO() {});
    drive.setPose(new Pose2d(kTargetX - distance, startY, Rotation2d.fromDegrees(headingDeg)));
    vision = new Vision(new VisionIOSim(drive::getPose, litColumn, 7130));

    // 先讓視覺跑幾個週期拿到第一張畫面
    for (int i = 0; i < 5; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
    }

    VisionAlign cmd = new VisionAlign(drive, vision, target);
    CommandScheduler.getInstance().schedule(cmd);
    double t = 0;
    while (cmd.isScheduled() && t < 7.0) {
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
      t += kDt;
      if (System.getenv("VA_TRACE") != null && Math.round(t / kDt) % 50 == 0) {
        System.out.printf("  trace t=%.1f shooterY=%.3f v=%.3f%n", t, shooterY(), drive.getTwist().vx);
      }
    }
    // 讓車子停下來後再量
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
    }
    double error = shooterY() - columnY(target, litColumn);
    return new Result(t < 7.0, error, hitError(columnY(target, litColumn)), t);
  }

  @ParameterizedTest(name = "{0} 從 y={1} 出發")
  @CsvSource({
    "LEFT, 2.0", "LEFT, 2.5", "LEFT, 3.2",
    "MID, 1.5", "MID, 2.0", "MID, 2.5",
    "RIGHT, 0.9", "RIGHT, 1.3", "RIGHT, 1.8",
  })
  void alignsToColumn(VisionAlign.Target target, double startY) {
    Result r = run(target, startY, RangeSensorConstants.kFallbackDistanceMeters, 90, 1);
    System.out.printf("%-6s start=%.2f  final error=%+.3f m  time=%.2f s%n", target, startY, r.finalError, r.seconds);
    assertEquals(0, r.hitError, kFinalTolerance);
  }

  @ParameterizedTest(name = "LIT 亮第 {0} 欄")
  @CsvSource({"0", "1", "2"})
  void alignsToLitColumn(int litColumn) {
    Result r = run(VisionAlign.Target.LIT, 2.0, RangeSensorConstants.kFallbackDistanceMeters, 90, litColumn);
    System.out.printf("LIT col=%d  final error=%+.3f m  time=%.2f s%n", litColumn, r.finalError, r.seconds);
    assertEquals(0, r.hitError, kFinalTolerance);
  }

  /**
   * 敏感度掃描（只印結果、不判斷成敗）：真實距離和 TF-Luna 給的距離不同、車身沒擺正、起點很遠。
   * 用 VA_SWEEP=1 開啟：./gradlew test --tests "*VisionAlignSimTest.sweep" -i
   */
  @org.junit.jupiter.api.Test
  void sweep() {
    org.junit.jupiter.api.Assumptions.assumeTrue(System.getenv("VA_SWEEP") != null);
    double fallback = RangeSensorConstants.kFallbackDistanceMeters;
    System.out.println("== 真實距離 vs 假設距離 " + fallback + " m（MID，從 y=2.3 出發）==");
    for (double d : new double[] {2.0, 2.5, 3.0, 3.5, 4.0}) {
      Result r = run(VisionAlign.Target.MID, 2.3, d, 90, 1);
      System.out.printf("  distance=%.1f  error=%+.3f  time=%.2f  finished=%b%n", d, r.finalError, r.seconds, r.finished);
    }
    System.out.println("== 車身偏角（MID，從 y=2.3 出發）==");
    for (double h : new double[] {80, 85, 88, 92, 95, 100}) {
      Result r = run(VisionAlign.Target.MID, 2.3, fallback, h, 1);
      System.out.printf("  heading=%.0f  沿牆位置誤差=%+.3f  球落點誤差=%+.3f  time=%.2f%n", h, r.finalError, r.hitError, r.seconds);
    }
    System.out.println("== 起點距離（RIGHT 欄 y=" + kRightTargetY + "）==");
    for (double y : new double[] {1.6, 2.0, 2.4, 2.8, 3.2, 3.6}) {
      Result r = run(VisionAlign.Target.RIGHT, y, fallback, 90, 1);
      System.out.printf("  startY=%.1f (要移動 %.2f m)  error=%+.3f  time=%.2f  finished=%b%n",
          y, y + VisionConstants.kShooterForwardOfCameraMeters - kRightTargetY, r.finalError, r.seconds, r.finished);
    }
  }

  /** 改 VisionAlign 的 NT 調整參數（private static LoggedNetworkNumber），測試專用。 */
  private static void setTuning(String field, double value) {
    try {
      var f = VisionAlign.class.getDeclaredField(field);
      f.setAccessible(true);
      var n = (org.littletonrobotics.junction.networktables.LoggedNetworkNumber) f.get(null);
      n.set(value);
      n.periodic();
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * 速度 × 延遲掃描（只印結果）：VA_SPEED=1 ./gradlew test --tests "*VisionAlignSimTest.speedSweep" -i
   * MID 欄，分別從要移動 0.3 m 與 1.0 m 的地方出發。
   */
  @org.junit.jupiter.api.Test
  void speedSweep() {
    org.junit.jupiter.api.Assumptions.assumeTrue(System.getenv("VA_SPEED") != null);
    double fallback = RangeSensorConstants.kFallbackDistanceMeters;
    double fwd = VisionConstants.kShooterForwardOfCameraMeters;
    try {
      for (double latency : new double[] {0.06, 0.12}) {
        VisionIOSim.latencySeconds = latency;
        System.out.printf("== 延遲 %.0f ms ==%n", latency * 1000);
        for (double maxSpeed : new double[] {0.25, 0.4, 0.5, 0.75, 1.0}) {
          setTuning("kMaxSpeed", maxSpeed);
          for (double travel : new double[] {0.3, 1.0}) {
            Result r = run(VisionAlign.Target.MID, kMidTargetY - fwd - travel, fallback, 90, 1);
            System.out.printf("  maxSpeed=%.2f travel=%.1f  落點誤差=%+.3f  time=%.2f%s%n",
                maxSpeed, travel, r.hitError, r.seconds, r.seconds >= 4.99 ? " 逾時" : "");
          }
        }
      }
    } finally {
      VisionIOSim.latencySeconds = VisionIOSim.kDefaultLatencySeconds;
      setTuning("kMaxSpeed", 0.5);
    }
  }
}
