package frc.robot.commands.drive;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Timer;

import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Column;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * 用 YOLO 偵測結果水平對準九宮格的某一欄（沿牆直走）。牆面角度請先用 {@link LaserAlign} 對正。
 *
 * <p>流程：先依 {@link Target} 找到要對的那一欄並「鎖定」，之後每張新畫面只追蹤離鎖定位置最近的欄，
 * 中間用里程計補償機器人自己移動的距離，避免跳到隔壁欄。
 *
 * <p>結束條件：連續 {@code kSettleFrames} 張畫面都在容許誤差內（成功）；
 * 或找不到目標、看丟太久、總時間逾時（失敗）。結果記在 {@code VisionAlign/Success}。
 */
public class VisionAlign extends Command {
  public enum Target {
    /** 亮紅燈的加分格那一欄（比賽前 60 秒）。 */
    LIT,
    LEFT,
    MID,
    RIGHT,
    /** 離射球點最近的一欄。 */
    NEAREST
  }

  // 可在 NT /Tuning/VisionAlign/* 現場調整，所有 VisionAlign 共用
  private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/VisionAlign/kP", 1.5);
  private static final LoggedNetworkNumber kMaxSpeed = new LoggedNetworkNumber("/Tuning/VisionAlign/MaxSpeedMps", 0.5);
  private static final LoggedNetworkNumber kMinSpeed = new LoggedNetworkNumber("/Tuning/VisionAlign/MinSpeedMps", 0.06);
  private static final LoggedNetworkNumber kTolerance = new LoggedNetworkNumber("/Tuning/VisionAlign/ToleranceMeters", 0.02);

  private static final int kSettleFrames = 3;
  private static final double kTrackGateMeters = VisionConstants.kColumnSpacingMeters / 3;
  private static final double kAcquireTimeoutSec = 1.0;
  private static final double kLostTimeoutSec = 0.5;
  private static final double kTotalTimeoutSec = 5.0;

  private final Drive drive;
  private final Vision vision;
  private final Target target;
  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  private final Timer totalTimer = new Timer();
  private final Timer seenTimer = new Timer();

  private double lockedError;
  private int settledFrames;
  private Pose2d lastPose;

  public VisionAlign(Drive drive, Vision vision, Target target) {
    this.drive = drive;
    this.vision = vision;
    this.target = target;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    lockedError = Double.NaN;
    settledFrames = 0;
    lastPose = drive.getPose();
    totalTimer.restart();
    seenTimer.restart();
    Logger.recordOutput("VisionAlign/Target", target.name());
    Logger.recordOutput("VisionAlign/Success", false);
  }

  @Override
  public void execute() {
    double distance = rangeSensor.getTargetDistance();

    // 里程計補償：機器人往前開了 delta，目標相對位置就少 delta
    Pose2d pose = drive.getPose();
    double delta = pose.getTranslation().minus(lastPose.getTranslation())
      .rotateBy(lastPose.getRotation().unaryMinus()).getX();
    lastPose = pose;
    if (isLocked()) lockedError -= delta;

    if (vision.hasNewFrame()) {
      OptionalDouble measured = isLocked()
        ? Vision.nearest(vision.getColumnErrors(distance), lockedError, kTrackGateMeters)
        : acquire(distance);
      if (measured.isPresent()) {
        lockedError = measured.getAsDouble();
        seenTimer.restart();
        settledFrames = Math.abs(lockedError) <= kTolerance.get() ? settledFrames + 1 : 0;
      }
    }

    double speed = 0;
    if (isLocked() && Math.abs(lockedError) > kTolerance.get()) {
      speed = Math.clamp(kP.get() * lockedError, -kMaxSpeed.get(), kMaxSpeed.get());
      if (Math.abs(speed) < kMinSpeed.get()) speed = Math.copySign(kMinSpeed.get(), lockedError);
    }
    drive.setVelocity(speed, speed);

    Logger.recordOutput("VisionAlign/Locked", isLocked());
    Logger.recordOutput("VisionAlign/ErrorMeters", lockedError);
    Logger.recordOutput("VisionAlign/SpeedMps", speed);
    Logger.recordOutput("VisionAlign/SettledFrames", settledFrames);
  }

  private OptionalDouble acquire(double distance) {
    return switch (target) {
      case LIT -> vision.getLitError(distance);
      case LEFT -> vision.getColumnError(Column.LEFT, distance);
      case MID -> vision.getColumnError(Column.MID, distance);
      case RIGHT -> vision.getColumnError(Column.RIGHT, distance);
      case NEAREST -> Vision.nearest(vision.getColumnErrors(distance), 0, Double.POSITIVE_INFINITY);
    };
  }

  private boolean isLocked() {
    return !Double.isNaN(lockedError);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
    Logger.recordOutput("VisionAlign/Success", !interrupted && settledFrames >= kSettleFrames);
  }

  @Override
  public boolean isFinished() {
    if (settledFrames >= kSettleFrames) return true;
    double lostTimeout = isLocked() ? kLostTimeoutSec : kAcquireTimeoutSec;
    return seenTimer.hasElapsed(lostTimeout) || totalTimer.hasElapsed(kTotalTimeoutSec);
  }
}
