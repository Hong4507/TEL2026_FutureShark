package frc.robot;

import static frc.robot.Constants.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
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
import org.wpilib.units.measure.Angle;

import frc.FSLib.rangefinder.RangeSensorConstants;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.AutoRoutines.AutoMethod;
import frc.robot.commands.AutoRoutines.Half;
import frc.robot.commands.AutoRoutines.Plan;
import frc.robot.commands.AutoRoutines.StartSide;
import frc.robot.commands.AutoRoutines.TargetColumn;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSim;
import frc.robot.subsystems.turretlidar.TurretLidar;
import frc.robot.subsystems.turretlidar.TurretLidarIO;
import frc.robot.subsystems.turretlidar.TurretLidarIOSim;

/**
 * 不用視覺的自動流程（{@link AutoRoutines#lunaColumn}）閉迴路模擬：砲台 TF-Luna 用 {@link TurretLidarIOSim} 光束模擬，
 * 每一發擊發時算射擊線（機器人右側）打到九宮格平面的 y，檢查是不是打在洞中間。
 *
 * <p>機器人離牆的距離放在「光束剛好打中列中心」的位置（23° 時鏡頭到板面 3.109 m）。
 */
class AutoLunaSimTest {
  private static final double kDt = Constants.kDefaultPeriod;
  /** 落點容許誤差：洞半徑 20 cm − 球半徑 3.5 cm 的一半。 */
  private static final double kTolerance = 0.08;

  private Drive drive;
  /** 這個 y 之後（往 +y）被對手擋住；NaN = 沒擋。 */
  private double blockY = Double.NaN;
  private double blockMinY = Double.NaN;

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

  /** 射擊線（機器人右側）打到 x = kTargetX 時的 y。射球點當成在機器人中心（和模擬 Luna 同一點）。 */
  private double hitY() {
    Pose2d p = drive.getPose();
    double h = p.getRotation().getRadians();
    double t = (kTargetX - p.getX()) / Math.sin(h);
    return p.getY() - Math.cos(h) * t;
  }

  private double wallDistance() {
    return kTargetX - drive.getPose().getX() - TurretLidar.kInboardMeters;
  }

  private record Run(List<Double> shots, double seconds, double endHeadingDeg) {}

  private Run run(StartSide side, double startY, TurretLidarIO lidarIO) {
    drive = new Drive(new DriveIOSim(), new GyroIO() {}) {
      @Override
      public void setVelocity(double left, double right) {
        // 對手擋住：往被擋的方向直走時只留下轉向的部分
        double y = getPose().getY();
        double forward = (left + right) / 2;
        boolean blocked = (!Double.isNaN(blockY) && y >= blockY && forward > 0)
            || (!Double.isNaN(blockMinY) && y <= blockMinY && forward < 0);
        if (blocked) {
          double turn = (right - left) / 2;
          left = -turn;
          right = turn;
        }
        super.setVelocity(left, right);
      }
    };
    double x = kTargetX - TurretLidar.idealHorizontalMeters(1);
    drive.setPose(new Pose2d(x, startY, Rotation2d.fromDegrees(90)));
    RangeSensors.setForTesting(new TargetRangeSensor() {
      @Override
      public double getTargetDistance() {
        return wallDistance() + RangeSensorConstants.kShooterInboardMeters;
      }

      @Override
      public Angle getYaw() {
        return null;
      }

      @Override
      public OptionalDouble getWallDistanceMeters() {
        return OptionalDouble.of(wallDistance());
      }
    });
    TurretLidar lidar = new TurretLidar(lidarIO, () -> OptionalDouble.of(wallDistance()));

    List<Double> shots = new ArrayList<>();
    Turret turret = new Turret(new TurretSim()) {
      private boolean firing = false;

      @Override
      public void setTrigger(double volts) {
        if (volts > 0 && !firing) shots.add(hitY());
        firing = volts > 0;
        super.setTrigger(volts);
      }
    };

    Plan plan = new Plan(Half.FIRST, side, false, TargetColumn.MID, AutoMethod.LUNA);
    Command auto = AutoRoutines.build(drive, null, turret, lidar, plan);
    CommandScheduler.getInstance().schedule(auto);
    double t = 0;
    while (auto.isScheduled() && t < 60) {
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
      t += kDt;
    }
    double heading = drive.getPose().getRotation().getDegrees();
    System.out.printf("== %s y0=%.2f 擋=%.2f：%.1f s，%d 發，結束朝向 %.1f°，落點 %s%n", side, startY, blockY, t,
        shots.size(), heading, shots.stream().map(v -> String.format("%.3f", v)).toList());
    assertTrue(t < 60, "60 秒內沒跑完");
    return new Run(shots, t, heading);
  }

  private static void assertAllAt(List<Double> shots, int count, double y) {
    assertEquals(count, shots.size());
    for (int i = 0; i < shots.size(); i++) {
      assertEquals(y, shots.get(i), kTolerance, "第 " + (i + 1) + " 發");
    }
  }

  /** 右起點往車頭找到右欄、左起點往車尾找到左欄，都停在洞中間射 9 發。 */
  @ParameterizedTest(name = "{0} 從 y={1} 出發 → 欄 y={2}")
  @CsvSource({
    "RIGHT, 0.40, 1.1",
    "RIGHT, 0.70, 1.1",
    "RIGHT, 1.30, 2.0",   // 已經過了右欄 → 往前找到中欄
    "LEFT,  3.60, 2.9",
    "LEFT,  3.20, 2.9",
  })
  void findsHoleAndShoots(StartSide side, double startY, double expectedY) {
    Run r = run(side, startY, new TurretLidarIOSim(() -> drive.getPose()));
    assertAllAt(r.shots, 9, expectedY);
    for (double s : r.shots) assertEquals(expectedY, s, 0.03, "停在洞中間");
  }

  /** 前進被對手擋住（還沒到洞）→ 原地轉找洞，找到才射。 */
  @Test
  void blockedThenTurnScan() {
    blockY = 0.80;
    Run r = run(StartSide.RIGHT, 0.40, new TurretLidarIOSim(() -> drive.getPose()));
    assertAllAt(r.shots, 9, kRightTargetY);
    assertTrue(r.endHeadingDeg > 91, "應該往逆時針轉了一點，實際 " + r.endHeadingDeg);
  }

  @Test
  void blockedLeftSideTurnsClockwise() {
    blockMinY = 3.20;
    Run r = run(StartSide.LEFT, 3.60, new TurretLidarIOSim(() -> drive.getPose()));
    assertAllAt(r.shots, 9, kLeftTargetY);
    assertTrue(r.endHeadingDeg < 89, "應該往順時針轉了一點，實際 " + r.endHeadingDeg);
  }

  /** 光束永遠打在板上（找不到洞）→ 轉完掃描也找不到 → 不射，朝向回到原來。 */
  @Test
  void noHoleNoShots() {
    Run r = run(StartSide.RIGHT, 0.40, new TurretLidarIO() {
      @Override
      public void updateInputs(TurretLidarIOInputs inputs) {
        inputs.connected = true;
        inputs.distanceMeters = TurretLidar.idealHorizontalMeters(1) / Math.cos(Math.toRadians(TurretLidar.kMountPitchDeg));
        inputs.strength = 1000;
      }
    });
    assertEquals(0, r.shots.size());
    assertEquals(90, r.endHeadingDeg, 2);
  }

  /** 砲台 Luna 斷線 → 只走 ApproachMeters，原地射 9 發（等於人工擺位）。 */
  @Test
  void lidarUnavailableShootsInPlace() {
    Run r = run(StartSide.RIGHT, 0.80, new TurretLidarIO() {});
    assertEquals(9, r.shots.size());
    assertEquals(0.80 + 0.3, r.shots.get(0), 0.1);
  }
}
