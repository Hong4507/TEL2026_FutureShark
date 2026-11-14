package frc.robot.commands.drive;

import static frc.robot.Constants.FieldConstants.kTargetX;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Radians;

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
import org.wpilib.units.measure.Angle;

import frc.FSLib.rangefinder.DualTFLunaRangeSensor;
import frc.FSLib.rangefinder.RangeSensorConstants;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.GyroIO;

/**
 * LaserAlign（TF-Luna 對正牆面）閉迴路模擬。
 *
 * <p>假設兩顆 TF-Luna 和攝影機一樣朝機器人右側、對著九宮格平面（x = kTargetX），沿車身前後相距
 * {@link RangeSensorConstants#kBaselineMeters}。距離由模擬位置算光束打到牆的長度，再交給正式程式的
 * {@link DualTFLunaRangeSensor#yawRad} 算偏航角。
 *
 * <p>{@code leftIsFront}：程式裡的「Left」那顆是不是車頭那顆。從 TF-Luna 後面往牆看，左手邊是車頭，
 * 所以照字面接線是 true。兩種接法剛好一種方向對、一種方向反，實機哪一種要 11/14 確認
 * （檢查清單 1.5）。方向反的那一種必須很快停下來並記錄 WrongDirection，而不是一直轉。
 */
class LaserAlignSimTest {
  private static final double kDt = frc.robot.Constants.kDefaultPeriod;
  private static final double kDistance = 2.5;

  private Drive drive;

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
    RangeSensors.setForTesting(null);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  /** 依模擬位置算兩顆 TF-Luna 的距離。 */
  private final class SimTFLunas implements TargetRangeSensor {
    final boolean leftIsFront;

    SimTFLunas(boolean leftIsFront) {
      this.leftIsFront = leftIsFront;
    }

    /** 車身前後 offset（+ = 車頭方向）的那顆，光束朝機器人右側打到牆的長度。 */
    double beam(double forwardOffset) {
      Pose2d p = drive.getPose();
      double h = p.getRotation().getRadians();
      double sx = p.getX() + forwardOffset * Math.cos(h);
      double dx = Math.sin(h); // 機器人右側方向的 x 分量
      return (kTargetX - sx) / dx;
    }

    @Override
    public double getTargetDistance() {
      return RangeSensorConstants.kFallbackDistanceMeters;
    }

    @Override
    public Angle getYaw() {
      double half = RangeSensorConstants.kBaselineMeters / 2;
      double front = beam(half), rear = beam(-half);
      // 光束太斜（> 60°）時 TF-Luna 收不到反射
      if (Math.abs(Math.PI / 2 - drive.getPose().getRotation().getRadians()) > Math.toRadians(60)) return null;
      double left = leftIsFront ? front : rear;
      double right = leftIsFront ? rear : front;
      return Radians.of(DualTFLunaRangeSensor.yawRad(left, right));
    }
  }

  private record Result(double startDeg, double endDeg, double seconds, boolean timedOut) {}

  private Result run(double headingDeg, boolean leftIsFront) {
    drive = new Drive(new DriveIOSim(), new GyroIO() {});
    drive.setPose(new Pose2d(kTargetX - kDistance, 2.0, Rotation2d.fromDegrees(headingDeg)));
    RangeSensors.setForTesting(new SimTFLunas(leftIsFront));

    LaserAlign cmd = new LaserAlign(drive);
    CommandScheduler.getInstance().schedule(cmd);
    double t = 0;
    while (cmd.isScheduled() && t < 5.0) {
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
      t += kDt;
    }
    boolean timedOut = cmd.isScheduled();
    CommandScheduler.getInstance().cancelAll();
    for (int i = 0; i < 25; i++) { // 停下來再量
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
    }
    double end = drive.getPose().getRotation().getDegrees();
    System.out.printf("leftIsFront=%b  %5.1f° → %6.1f°  %.2f s%n", leftIsFront, headingDeg, end, t);
    return new Result(headingDeg, end, t, timedOut);
  }

  /** 方向對的接法：kYawSign = +1 時，Left 是車尾那顆。 */
  @ParameterizedTest(name = "方向對：從 {0}° 轉正")
  @CsvSource({"60", "75", "85", "95", "105", "120"})
  void squaresUp(double headingDeg) {
    Result r = run(headingDeg, false);
    assertFalse(r.timedOut);
    // LaserAlign 容許 0.095 rad ≈ 5.4°，加上煞停多轉一點
    assertEquals(90, r.endDeg, 7);
    assertTrue(r.seconds < 2.0, "應該在 2 秒內轉正，實際 " + r.seconds);
  }

  @ParameterizedTest(name = "方向對：已經正了 {0}° 不動")
  @CsvSource({"88", "90", "93"})
  void alreadySquareFinishesImmediately(double headingDeg) {
    Result r = run(headingDeg, false);
    assertTrue(r.seconds <= 3 * kDt);
    assertEquals(headingDeg, r.endDeg, 1);
  }

  /** 接反或 kYawSign 設反：要很快發現並停下，不能一直轉。 */
  @ParameterizedTest(name = "方向反：從 {0}° 要停下")
  @CsvSource({"75", "85", "105"})
  void wrongDirectionStopsQuickly(double headingDeg) {
    Result r = run(headingDeg, true);
    assertFalse(r.timedOut);
    assertTrue(r.seconds < 1.0, "方向反應該 1 秒內停，實際 " + r.seconds);
    double worse = Math.abs(r.endDeg - 90) - Math.abs(headingDeg - 90);
    assertTrue(worse < 15, "方向反時最多再歪 15°，實際多歪 " + worse + "°");
  }
}
