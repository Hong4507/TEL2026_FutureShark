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
import org.wpilib.system.Timer;
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
    return run(side, startY, lidarIO, 90, 0);
  }

  private Run run(StartSide side, double startY, TurretLidarIO lidarIO, double startHeadingDeg, double startYawToWallDeg) {
    return run(side, startY, lidarIO, startHeadingDeg, startYawToWallDeg, TurretLidar.idealHorizontalMeters(1));
  }

  private double endTargetDistance = Double.NaN;
  private AutoMethod method = AutoMethod.LUNA;
  /** 飛輪第一次被要求轉的時候機器人在哪（NaN = 沒轉過）。 */
  private double spinUpY = Double.NaN;
  private double spinUpHeadingDeg = Double.NaN;
  /** Stroage 佔用 DIO，同一個 JVM 只能建一次 → 共用。 */
  private static frc.robot.subsystems.storage.Stroage sharedStorage;

  private static frc.robot.subsystems.storage.Stroage storage() {
    if (sharedStorage == null) sharedStorage = new frc.robot.subsystems.storage.Stroage();
    else CommandScheduler.getInstance().registerSubsystem(sharedStorage); // teardown 會 unregisterAllSubsystems
    return sharedStorage;
  }

  /** true = 和實機一樣帶 Storage（整段跑 ShootFeed）。 */
  private boolean useStorage = false;
  /** 每一發擊發的時間（秒）。 */
  private final List<Double> shotTimes = new ArrayList<>();
  /** 模擬車身 Luna 在黑板上的最遠量程。 */
  private double kLunaMaxRangeMeters = 4.6;

  private Run run(StartSide side, double startY, TurretLidarIO lidarIO, double startHeadingDeg, double startYawToWallDeg,
      double startHorizontal) {
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
    double x = kTargetX - startHorizontal;
    drive.setPose(new Pose2d(x, startY, Rotation2d.fromDegrees(startHeadingDeg)));
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
        // 黑板：太遠量不到（模擬 4.6 m 以外沒讀值）
        return wallDistance() > kLunaMaxRangeMeters ? OptionalDouble.empty() : OptionalDouble.of(wallDistance());
      }
    });
    TurretLidar lidar = new TurretLidar(lidarIO, () -> OptionalDouble.of(wallDistance()));

    List<Double> shots = new ArrayList<>();
    Turret turret = new Turret(new TurretSim()) {
      private boolean firing = false;

      @Override
      public void setVelocity(double up, double down) {
        if ((up > 0 || down > 0) && Double.isNaN(spinUpY)) {
          spinUpY = drive.getPose().getY();
          spinUpHeadingDeg = drive.getPose().getRotation().getDegrees();
        }
        super.setVelocity(up, down);
      }

      @Override
      public void setTrigger(double volts) {
        if (volts > 0 && !firing) {
          shots.add(hitY());
          shotTimes.add(Timer.getTimestamp());
        }
        firing = volts > 0;
        super.setTrigger(volts);
      }
    };

    Plan plan = new Plan(Half.FIRST, side, false, TargetColumn.MID, method);
    Command auto = AutoRoutines.build(drive, null, turret, lidar, useStorage ? storage() : null,
        plan, startYawToWallDeg);
    CommandScheduler.getInstance().schedule(auto);
    double t = 0;
    while (auto.isScheduled() && t < 60) {
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
      t += kDt;
    }
    double heading = drive.getPose().getRotation().getDegrees();
    endTargetDistance = wallDistance() + RangeSensorConstants.kShooterInboardMeters;
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

  /** 砲台 Luna 斷線 → 不動，原地射 9 發（等於人工擺位）。 */
  @Test
  void lidarUnavailableShootsInPlace() {
    Run r = run(StartSide.RIGHT, 0.80, new TurretLidarIO() {});
    assertEquals(9, r.shots.size());
    assertEquals(0.80, r.shots.get(0), 0.05);
  }

  /** 11/20：Auto 改原地轉 → 整段都不沿牆走（不會撞場地 Frame）。洞在反方向時，掃完一邊轉回來掃另一邊。 */
  @ParameterizedTest(name = "{0} 從 y={1} → 欄 y={2}，不沿牆走")
  @CsvSource({
    "RIGHT, 0.80, 1.1",
    "RIGHT, 1.50, 1.1",   // 往車頭那邊（逆時針）掃不到近的，掃到中欄或回頭找到右欄都可以，但不能走
    "LEFT,  3.20, 2.9",
  })
  void doesNotDriveAlongWall(StartSide side, double startY, double expectedY) {
    Run r = run(side, startY, new TurretLidarIOSim(() -> drive.getPose()));
    assertEquals(startY, drive.getPose().getY(), 0.05, "不應該沿牆走");
    assertEquals(9, r.shots.size());
  }

  /** 11/19：起點車頭朝九宮格（StartYawToWallDeg = −90）→ 盲走 2.55 m（起點 TargetDistance 5.8 m）→ 逆時針轉 90° → 找洞射 9 發。 */
  @ParameterizedTest(name = "車頭朝九宮格 {0} 從 y={1} 出發 → 欄 y={2}")
  @CsvSource({
    "RIGHT, 0.40, 1.1",
    "LEFT,  3.60, 2.9",
  })
  void frontToGridTurnsFirst(StartSide side, double startY, double expectedY) {
    double startHorizontal = 5.80 - RangeSensorConstants.kShooterInboardMeters + TurretLidar.kInboardMeters;
    Run r = run(side, startY, new TurretLidarIOSim(() -> drive.getPose()), 0, -90, startHorizontal);
    assertAllAt(r.shots, 9, expectedY);
    assertEquals(90, r.endHeadingDeg, 26, "轉完右側朝九宮格（原地轉找洞最多 25°）");
  }

  /** 11/19：洞後面放實心黑箱子（離板面 boxBehind m）→ 穿過洞讀到的是箱子，不是 0。靠「板子基準」照樣找得到。 */
  @ParameterizedTest(name = "洞後箱子 {0} m：{1} 從 y={2} → 欄 y={3}")
  @CsvSource({
    "0.15, RIGHT, 0.40, 1.1",
    "0.25, RIGHT, 0.70, 1.1",
    "0.40, LEFT,  3.60, 2.9",
    "0.15, RIGHT, 1.30, 2.0",
  })
  void findsHoleWithBoxBehind(double boxBehind, StartSide side, double startY, double expectedY) {
    Run r = run(side, startY, new TurretLidarIOSim(() -> drive.getPose(), boxBehind));
    assertAllAt(r.shots, 9, expectedY);
    for (double v : r.shots) assertEquals(expectedY, v, 0.03, "停在洞中間");
  }

  /**
   * 11/20：車頭朝九宮格 → 盲走 2.55 m → 逆時針轉 90° → 不再前後走，原地用融合距離找洞射 9 發。
   * 模擬起點 5.5–6.0 m 都找得到洞；6.1 m 以上（射擊時 > 3.55 m，再加上原地轉斜打）光束打不到洞的高度 → 不射。
   */
  @ParameterizedTest(name = "車頭朝九宮格、TargetDistance {0} m 出發 → 盲走 2.55 m 後原地射")
  @CsvSource({
    "5.50", "5.60", "5.70", "5.80", "5.90", "6.00",   // 模擬：6.10 以上砲台 Luna 找不到洞（射擊時 > 3.55 m）
  })
  void blindApproachThenShootInPlace(double startTarget) {
    double startHorizontal = startTarget - RangeSensorConstants.kShooterInboardMeters + TurretLidar.kInboardMeters;
    Run r = run(StartSide.RIGHT, 0.40, new TurretLidarIOSim(() -> drive.getPose()), 0, -90, startHorizontal);
    assertEquals(startTarget - 2.55, endTargetDistance, 0.05, "轉完不再前後走");
    assertAllAt(r.shots, 9, kRightTargetY);
    assertEquals(90, r.endHeadingDeg, 26);
  }

  /** 11/20：實機版本（帶 Storage、整段 ShootFeed）：車頭朝九宮格 5.8 m 起跑，照樣射 9 發。 */
  @Test
  void withStorageShootFeed() {
    useStorage = true;
    double startHorizontal = 5.80 - RangeSensorConstants.kShooterInboardMeters + TurretLidar.kInboardMeters;
    Run r = run(StartSide.RIGHT, 0.40, new TurretLidarIOSim(() -> drive.getPose()), 0, -90, startHorizontal);
    assertAllAt(r.shots, 9, kRightTargetY);
    // 11/20：盲走、轉向、找洞時飛輪都不轉；轉完找到洞（右側朝九宮格附近、停在洞的角度）才加速
    assertEquals(r.endHeadingDeg, spinUpHeadingDeg, 0.5, "找到洞、停在射擊角度才開始加速");
  }

  /** 11/20：「只盲走＋轉向」：走 2.55 m、逆時針轉 90° 就結束，不射、不再轉。 */
  @Test
  void approachOnlyStopsAfterTurn() {
    method = AutoMethod.APPROACH_ONLY;
    useStorage = true;
    double startHorizontal = 5.80 - RangeSensorConstants.kShooterInboardMeters + TurretLidar.kInboardMeters;
    Run r = run(StartSide.RIGHT, 0.40, new TurretLidarIOSim(() -> drive.getPose()), 0, -90, startHorizontal);
    assertEquals(0, r.shots.size(), "不射");
    assertTrue(Double.isNaN(spinUpY), "只盲走＋轉向：飛輪不轉");
    assertEquals(90, r.endHeadingDeg, 2, "右側朝九宮格");
    assertEquals(5.80 - 2.55, endTargetDistance, 0.05, "盲走 2.55 m");
    assertTrue(r.seconds < 10, "轉完就結束，實際 " + r.seconds);
  }

  /** 11/20：每發之間（擊發到擊發）至少隔 5 秒，讓 ShootFeed 送球。 */
  @Test
  void shotsAreSpacedFiveSeconds() {
    run(StartSide.RIGHT, 0.40, new TurretLidarIOSim(() -> drive.getPose()));
    assertEquals(9, shotTimes.size());
    for (int i = 1; i < shotTimes.size(); i++) {
      double gap = shotTimes.get(i) - shotTimes.get(i - 1);
      assertTrue(gap >= 5.0 - 0.03 && gap < 5.5, "第 " + i + "→" + (i + 1) + " 發間隔 " + gap);
    }
  }

  /** 11/20：射到一半砲台 Luna 不在洞裡了（例如被撞歪）→ 小掃重找也找不到 → 剩下的不射。 */
  @Test
  void stopsShootingWhenHoleLost() {
    TurretLidarIOSim sim = new TurretLidarIOSim(() -> drive.getPose());
    Run r = run(StartSide.RIGHT, 0.40, new TurretLidarIO() {
      @Override
      public void updateInputs(TurretLidarIOInputs inputs) {
        sim.updateInputs(inputs);
        if (shotTimes.size() >= 3) {
          // 光束一直打在板上
          inputs.distanceMeters = TurretLidar.idealHorizontalMeters(1) / Math.cos(Math.toRadians(TurretLidar.kMountPitchDeg));
          inputs.strength = 1000;
        }
      }
    });
    assertEquals(3, r.shots.size(), "對不到洞之後不能再射");
  }
}
