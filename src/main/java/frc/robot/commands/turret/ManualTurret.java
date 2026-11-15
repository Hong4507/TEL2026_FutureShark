package frc.robot.commands.turret;

import java.util.Locale;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.util.Util;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

/**
 * 手動射擊：右搖桿 Y 控制仰角，左搖桿 X 往右推擊發。
 *
 * <p>量插值表（ShotTable）用：
 * <ul>
 *   <li>Elastic 「賽前」分頁的 {@code /Tuning/Manual/PitchRad} 填一個值（例如 1.20）→ 仰角固定在這個值，不看搖桿，
 *       方便一次只改一點點。填 0 → 回到搖桿控制。</li>
 *   <li>每射一發，console 和 log（{@code ShotTable/Sample}）會記一行「距離、仰角」，命中的那一行貼進 ShotTable。</li>
 * </ul>
 */
public class ManualTurret extends Command {
  private static final double kTriggerSeconds = 0.15;

  /** 所有 ManualTurret 共用同一個 NT 值。 */
  private static final LoggedNetworkNumber pitchOverrideRad = new LoggedNetworkNumber("/Tuning/Manual/PitchRad", 0.0);

  private final Turret turret;

  private final DoubleSupplier angleSupplier;
  private final BooleanSupplier triggerSupplier;
  /** 砲台 TF-Luna 的距離（公尺），沒資料 = NaN。只用來記樣本。 */
  private final DoubleSupplier turretLidarMeters;
  boolean lastValue = false;
  private final Timer timer = new Timer();
  private int sampleCount = 0;

  public ManualTurret(Turret turret, DoubleSupplier angleSupplier, BooleanSupplier triggerSupplier, DoubleSupplier turretLidarMeters) {
    this.turret = turret;
    this.angleSupplier = angleSupplier;
    this.triggerSupplier = triggerSupplier;
    this.turretLidarMeters = turretLidarMeters;
    addRequirements(turret);
  }

  public ManualTurret(Turret turret, DoubleSupplier angleSupplier, BooleanSupplier triggerSupplier) {
    this(turret, angleSupplier, triggerSupplier, () -> Double.NaN);
  }

  @Override
  public void execute() {
    double pitch = commandedPitchRad();
    turret.setPitch(pitch);
    turret.setVelocity(TurretConstants.kMaxSpeedRadPerSec, TurretConstants.kMaxSpeedRadPerSec * 0.8);
    Logger.recordOutput("ManualTurret/CommandedPitchRad", pitch);

    if (triggerSupplier.getAsBoolean() && !lastValue) {
      timer.restart();
      recordSample(pitch);
    }
    lastValue = triggerSupplier.getAsBoolean();

    if (timer.isRunning()) {
      if (timer.get() < kTriggerSeconds) {
        turret.setTrigger(12);
      } else {
        turret.setTrigger(0);
      }
    }
  }

  private double commandedPitchRad() {
    double override = pitchOverrideRad.get();
    if (override > 0) {
      return Util.clamp(override, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
    }
    return Util.mapAxis(angleSupplier.getAsDouble(), TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
  }

  /** 擊發瞬間記一筆：可直接貼進 ShotTable 的 {距離, 仰角}，後面附實際仰角、飛輪是否到速、砲台 TF-Luna。 */
  private void recordSample(double commandedPitch) {
    sampleCount++;
    double distance = RangeSensors.get().getTargetDistance();
    boolean atSpeed = turret.isNearTargetSpeeds(TurretConstants.kMaxSpeedRadPerSec, TurretConstants.kMaxSpeedRadPerSec * 0.8);
    double slant = turretLidarMeters.getAsDouble();
    double lunaDistance = frc.robot.subsystems.turretlidar.TurretLidar.targetDistanceFromSlant(slant);
    // 前面 {距離, 仰角} 的距離 = 車身 Luna 的 TargetDistance（比賽時 ColumnShot 查表用的就是它）。
    // 「砲台Luna換算」= 砲台 Luna 斜距 × cos(23°) + 0.1588，兩者應該差不多；差很多代表車身 Luna 或砲台 Luna 有問題。
    String line = String.format(Locale.ROOT,
        "{%.3f, %.4f}, // #%d 實際仰角 %.4f, 飛輪%s, 砲台TF-Luna斜距 %.3f m → 換算距離 %.3f m（和車身Luna差 %+.3f）",
        distance, commandedPitch, sampleCount, turret.getPitchPositionRad(),
        atSpeed ? "到速" : "未到速(這發不準)", slant, lunaDistance, lunaDistance - distance);
    System.out.println("ShotTable 樣本 " + line);
    Logger.recordOutput("ShotTable/Sample", line);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    turret.setTrigger(0);
  }
}
