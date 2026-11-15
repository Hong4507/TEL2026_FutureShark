package frc.robot;

import static frc.robot.Constants.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.units.measure.Angle;

import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.AutoRoutines.Half;
import frc.robot.commands.AutoRoutines.Plan;
import frc.robot.commands.AutoRoutines.StartSide;
import frc.robot.commands.AutoRoutines.TargetColumn;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOSim;

/**
 * 完整自動流程模擬：每一發 ColumnShot 開始時，記錄射擊線打到九宮格平面的落點，
 * 檢查是不是打在預期那一欄（亮燈欄，沒亮燈時中欄）。
 *
 * <p>撞擊模擬：在第 bumpAtShot 發開始時，把機器人沿牆推 bumpMeters（直接改 pose；模擬裡視覺依 pose 產生，
 * 等同機器人真的被推走）。
 *
 * <p>注意：TurretSim 飛輪瞬間到速，時間不含飛輪加速。
 */
class AutoRoutineSimTest {
  private static final double kDt = Constants.kDefaultPeriod;
  /** 落點容許誤差：大洞半徑 20 cm − 球半徑 3.5 cm，留一半當安全範圍。加分洞另外看。 */
  private static final double kHoleTolerance = 0.08;
  private static final double kBonusTolerance = 0.05;

  private Drive drive;
  private record Shot(double time, double height, double hitY) {}

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // 模擬沒有 TF-Luna（DualTFLuna 會一直回預設 3.0 m，和實際 2.5 m 不符，VisionAlign 換算會偏）。
    // 改用模擬位置算：VisionIOSim 的攝影機在機器人中心，所以 cameraDistance(TargetDistance) 要等於 kTargetX − x。
    RangeSensors.setForTesting(new TargetRangeSensor() {
      @Override
      public double getTargetDistance() {
        return kTargetX - drive.getPose().getX() - VisionConstants.kCameraBehindShooterMeters;
      }

      @Override
      public Angle getYaw() {
        return null; // 和原本一樣：沒有 yaw，LaserAlign 直接結束
      }
    });
  }

  @AfterEach
  void teardown() {
    RangeSensors.setForTesting(null);
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  /** 射擊線（砲台固定朝機器人右側）打到 x = kTargetX 時的 y。 */
  private double hitY() {
    Pose2d pose = drive.getPose();
    double h = pose.getRotation().getRadians();
    double fwd = VisionConstants.kShooterForwardOfCameraMeters;
    double sx = pose.getX() + fwd * Math.cos(h), sy = pose.getY() + fwd * Math.sin(h);
    double t = (kTargetX - sx) / Math.sin(h);
    return sy - Math.cos(h) * t;
  }

  @ParameterizedTest(name = "{0} 亮燈欄={1} 撞擊於第{2}發 {3}m")
  @CsvSource({
    "FIRST, 0, -1, 0",
    "FIRST, 1, -1, 0",
    "FIRST, 2, -1, 0",
    "FIRST, -1, -1, 0",   // 沒亮燈
    "SECOND, 0, -1, 0",
    "SECOND, 2, -1, 0",
    "FIRST, 1, 5, 0.3",   // 射到一半被推 30 cm
    "SECOND, 0, 3, -0.3",
  })
  void runAuto(Half half, int litColumn, int bumpAtShot, double bumpMeters) {
    run(new Plan(half, StartSide.RIGHT, true, TargetColumn.MID), litColumn, bumpAtShot, bumpMeters, Double.NaN);
  }

  /**
   * §8.7 不跑路徑：機器人直接擺在目標欄前（射擊線偏 15 cm 模擬擺放誤差）。
   * §8.8 己方欄：沒亮燈時打起點那一側的欄；有亮燈時仍然打亮燈那欄。
   */
  @ParameterizedTest(name = "{0} {1} 路徑={2} {3} 亮燈欄={4} 擺在 y={5}")
  @CsvSource({
    "FIRST,  RIGHT, false, MID,      -1, 2.0",
    "FIRST,  RIGHT, false, MID,       1, 2.0",
    "SECOND, LEFT,  false, OWN_SIDE, -1, 2.9",
    "SECOND, RIGHT, false, OWN_SIDE, -1, 1.1",
    "SECOND, RIGHT, false, OWN_SIDE,  2, 1.1",
    "FIRST,  RIGHT, true,  OWN_SIDE, -1, NaN",  // 跑路徑到中欄，再沿牆到右欄
  })
  void runPlan(Half half, StartSide side, boolean runPath, TargetColumn column, int litColumn, double startY) {
    run(new Plan(half, side, runPath, column), litColumn, -1, 0, startY);
  }

  private void run(Plan plan, int litColumn, int bumpAtShot, double bumpMeters, double startY) {
    Half half = plan.half();
    drive = new Drive(new DriveIOSim(), new GyroIO() {});
    if (!Double.isNaN(startY)) {
      // 射擊線朝 +x（機器人右側），射球點在鏡頭前方 kShooterForwardOfCameraMeters
      drive.setPose(new Pose2d(kTargetX - 2.5, startY - VisionConstants.kShooterForwardOfCameraMeters + 0.15,
          org.wpilib.math.geometry.Rotation2d.fromDegrees(90)));
    }
    List<Shot> shots = new ArrayList<>();
    double[] now = {0};
    // 指令在流程（sequence）裡時不會觸發排程器回呼，所以改成攔截砲台的擊發：
    // 扳機從 0 變成通電 = 射出一發；扳機斷電 = 這發射完，撞擊在這時候發生。
    int[] flywheelStops = {0};
    Turret turret = new Turret(new TurretSim()) {
      private boolean firing = false;

      @Override
      public void setVelocity(double up, double down) {
        if (up == 0 && !shots.isEmpty()) flywheelStops[0]++;
        super.setVelocity(up, down);
      }

      @Override
      public void setVelocity(double v) {
        if (v == 0 && !shots.isEmpty()) flywheelStops[0]++;
        super.setVelocity(v);
      }

      @Override
      public void setTrigger(double volts) {
        if (volts > 0 && !firing) {
          shots.add(new Shot(now[0], Double.NaN, hitY()));
        } else if (volts <= 0 && firing && shots.size() == bumpAtShot) {
          Pose2d p = drive.getPose();
          drive.setPose(new Pose2d(p.getTranslation().plus(new Translation2d(0, bumpMeters)), p.getRotation()));
        }
        firing = volts > 0;
        super.setTrigger(volts);
      }
    };
    Vision vision = new Vision(new VisionIOSim(drive::getPose, litColumn, 7130));

    Command auto = AutoRoutines.bonusThenColumn(drive, vision, turret, plan);
    CommandScheduler.getInstance().schedule(auto);
    while (auto.isScheduled() && now[0] < 60) {
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);
      now[0] += kDt;
    }

    double[] columns = {kLeftTargetY, kMidTargetY, kRightTargetY};
    double fallbackY = switch (plan.fallbackColumn()) {
      case LEFT -> kLeftTargetY;
      case MID -> kMidTargetY;
      case RIGHT -> kRightTargetY;
    };
    double expectedY = litColumn >= 0 ? columns[litColumn] : fallbackY;
    int bonus = litColumn >= 0 ? half.bonusBalls() : 0;
    int expectedShots = bonus + 3 * half.ballsPerHole();

    System.out.printf("== %s 亮燈欄=%d 撞擊=%d/%.1fm：完成 %.1f s，%d 發 ==%n",
        plan.describe(), litColumn, bumpAtShot, bumpMeters, now[0], shots.size());
    double worst = 0;
    for (int i = 0; i < shots.size(); i++) {
      Shot s = shots.get(i);
      double err = s.hitY - expectedY;
      worst = Math.max(worst, Math.abs(err));
      System.out.printf("  #%d t=%5.1f %s 落點誤差=%+.3f%s%n", i + 1, s.time, i < bonus ? "加分格" : "大洞  ",
          err, i == bumpAtShot ? "  ← 被推之後第一發" : "");
    }

    assertTrue(now[0] < 60, "自動流程沒在 60 秒內跑完");
    // §8.6：流程中飛輪不停（只有最後結束時停一次）
    assertEquals(1, flywheelStops[0], "自動流程中飛輪不能停，結束時要停一次");
    assertEquals(expectedShots, shots.size());
    for (int i = 0; i < shots.size(); i++) {
      assertEquals(expectedY, shots.get(i).hitY, i < bonus ? kBonusTolerance : kHoleTolerance, "第 " + (i + 1) + " 發");
    }
  }
}
