package frc.robot.commands.drive;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.Command;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;

import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Column;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * 11/20：用 YOLO 偵測結果對準九宮格的某一欄，<b>原地轉</b>（不沿牆走，不會撞到場地 Frame）。
 * 和 {@link VisionAlign} 用同一套欄位判斷，只是把「沿牆差幾公尺」換成「要轉幾度」：
 * 射擊方向（機器人右側）要轉 atan(誤差 / 射球點到板面的斜距) 才會打到那一欄。
 *
 * <p>假設開始時車身正對牆面（自動流程前面有 LaserAlign；手動時先擺正比較準）。轉了多少看 {@link #turnedRad()}，
 * 射擊距離要用 正對距離 / cos(轉角)。
 *
 * <p>每張新畫面重新量一次、更新目標角度；畫面之間用陀螺儀追目標角度。
 * 結束條件同 {@link VisionAlign}：連續 {@code kSettleFrames} 張都在容許誤差內（成功）；找不到、看丟太久、逾時（失敗）。
 */
public class VisionTurnAlign extends Command {
  private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/VisionTurnAlign/kP", 0.8);
  /** 輪子線速度上下限（m/s）。0.15 ≈ 39°/s。 */
  private static final LoggedNetworkNumber kMaxWheel = new LoggedNetworkNumber("/Tuning/VisionTurnAlign/MaxWheelMps", 0.15);
  /** 太低轉不動；太高停不準（0.02 m/s ≈ 5°/s，3 m 外光點 0.27 m/s）。 */
  private static final LoggedNetworkNumber kMinWheel = new LoggedNetworkNumber("/Tuning/VisionTurnAlign/MinWheelMps", 0.02);
  /**
   * Jetson 回報以外的延遲（秒）：曝光、Limelight MJPEG 串流。用來找「拍照當下」的朝向。
   * 上機看 VisionTurnAlign/ErrorMeters 停下來後還會不會自己跳一下，跳的話加大。
   */
  private static final LoggedNetworkNumber kExtraLatency = new LoggedNetworkNumber("/Tuning/VisionTurnAlign/ExtraLatencySec", 0.0);
  /** 容許誤差：落點沿牆差多少（公尺），和 VisionAlign 同單位。 */
  private static final LoggedNetworkNumber kTolerance = new LoggedNetworkNumber("/Tuning/VisionTurnAlign/ToleranceMeters", 0.02);
  /** 最多轉幾度（相對正對牆面）。轉太多射擊距離、車身 Luna 都會變差。 */
  private static final LoggedNetworkNumber kMaxTurnDeg = new LoggedNetworkNumber("/Tuning/VisionTurnAlign/MaxTurnDeg", 30);
  /**
   * 11/20：瞄準修正（度，正 = 對準後再<b>順時針</b>多轉這麼多，射擊方向往車尾那側）。
   * 實機：VisionTurnAlign 對準完要再順時針多轉一點點才準。1° 是先猜的，3.3 m 外落點沿牆約移 5.8 cm；
   * 射偏往車頭那側就加大、往車尾那側就減小（每 1° ≈ 射擊距離 × 1.75 cm/m）。
   */
  private static final LoggedNetworkNumber kAimTrimDeg = new LoggedNetworkNumber("/Tuning/VisionTurnAlign/AimTrimDeg", 1.0);
  /** 測試用：換掉瞄準修正的來源。 */
  static java.util.function.DoubleSupplier aimTrimDeg = kAimTrimDeg::get;
  /** 被轉角上限擋住、停著也到不了：連續這麼多張畫面就放棄（不要等 8 秒逾時）。 */
  private static final int kClampedGiveUpFrames = 5;

  private static final int kSettleFrames = 3;
  private static final double kTrackGateMeters = VisionConstants.kColumnSpacingMeters / 3;
  private static final double kAcquireTimeoutSec = 2.0;
  private static final double kLostTimeoutSec = 1.5;
  private static final double kTotalTimeoutSec = 8.0;

  private final Drive drive;
  private final Vision vision;
  private final VisionAlign.Target target;
  /** 車身正對牆面時的朝向（rad，絕對）；null = 用開始時的朝向。 */
  private final java.util.function.DoubleSupplier squareHeading;
  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  private final Timer totalTimer = new Timer();
  private final Timer seenTimer = new Timer();

  private double startHeading;
  /** 目標朝向（rad，絕對）；NaN = 還沒鎖定。 */
  private double targetHeading;
  private double lastError;
  private int settledFrames;
  /** 最近約 1 秒的朝向歷史 {時間, 朝向}，用來找拍照當下的朝向（延遲補償）。 */
  private final java.util.ArrayDeque<double[]> headingHistory = new java.util.ArrayDeque<>();
  /** 上個週期有沒有在轉。轉動中拍的畫面不算「已對準」。 */
  private boolean turning;
  /** 連續幾張畫面「想轉的角度超過上限、而且已經停在上限」。 */
  private int clampedFrames;
  private boolean succeeded;

  public VisionTurnAlign(Drive drive, Vision vision, VisionAlign.Target target) {
    this(drive, vision, target, null);
  }

  /**
   * @param squareHeading 車身正對牆面時的朝向（rad，絕對）。自動流程每一發前都重新對準，開始時車身已經轉過了，
   *     要用一開始擺正時記下的朝向，轉角和射擊距離才算得對。
   */
  public VisionTurnAlign(Drive drive, Vision vision, VisionAlign.Target target,
      java.util.function.DoubleSupplier squareHeading) {
    this.drive = drive;
    this.vision = vision;
    this.target = target;
    this.squareHeading = squareHeading;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startHeading = squareHeading == null ? squareHeadingNow(drive) : squareHeading.getAsDouble();
    targetHeading = Double.NaN;
    lastError = Double.NaN;
    settledFrames = 0;
    succeeded = false;
    turning = false;
    clampedFrames = 0;
    headingHistory.clear();
    totalTimer.restart();
    seenTimer.restart();
    Logger.recordOutput("VisionTurnAlign/Target", target.name());
    Logger.recordOutput("VisionTurnAlign/Success", false);
  }

  /**
   * 車身正對牆面時的朝向（rad，絕對）：有車身 Luna／IMU 算出的牆面偏角就用它（現在朝向 − 偏角），
   * 沒有就當作現在就是正對的。
   */
  public static double squareHeadingNow(Drive drive) {
    double h = drive.getPose().getRotation().getRadians();
    var yaw = RangeSensors.get().getYaw();
    if (yaw == null) return h;
    double wallYaw = yaw.in(org.wpilib.units.Units.Radians) - Math.PI / 2;
    // 偏角 > 45° = 牆面基準不對（或沒對著九宮格），不用它
    return Double.isFinite(wallYaw) && Math.abs(wallYaw) < Math.toRadians(45) ? h - wallYaw : h;
  }

  private double heading() {
    return drive.getPose().getRotation().getRadians();
  }

  /** time 那一刻的朝向（歷史不夠舊時用最舊的）。 */
  private double headingAt(double time) {
    double best = heading();
    for (java.util.Iterator<double[]> it = headingHistory.descendingIterator(); it.hasNext();) {
      double[] e = it.next();
      best = e[1];
      if (e[0] <= time) break;
    }
    return best;
  }

  /** 相對正對牆面時轉了多少（rad，逆時針為正）。 */
  public double turnedRad() {
    return MathUtil.angleModulus(heading() - startHeading);
  }

  @Override
  public void execute() {
    double h = heading();
    double now = Timer.getTimestamp();
    headingHistory.addLast(new double[] {now, h});
    while (headingHistory.size() > 1 && headingHistory.peekFirst()[0] < now - 1.0) headingHistory.removeFirst();
    double cos = Math.max(Math.cos(turnedRad()), 0.5);
    // 射球點／鏡頭到板面：正對距離 / cos(轉角)
    double shooterRay = rangeSensor.getTargetDistance() / cos;
    double cameraDistance = Vision.cameraDistance(rangeSensor.getTargetDistance()) / cos;

    // 瞄準修正換算成沿牆公尺：對準完成時，視覺誤差應該停在 +bias（目標在射球點往車頭側一點 = 車子多順時針轉）
    double bias = shooterRay * Math.tan(Math.toRadians(aimTrimDeg.getAsDouble()));
    if (vision.hasNewFrame()) {
      OptionalDouble measured;
      if (isLocked()) {
        double predicted = shooterRay * Math.tan(MathUtil.angleModulus(targetHeading - h)) + bias;
        measured = Vision.nearest(vision.getColumnErrors(cameraDistance), predicted, kTrackGateMeters);
      } else {
        measured = acquire(cameraDistance);
      }
      if (measured.isPresent()) {
        double e = measured.getAsDouble() - bias;
        lastError = e;
        // 誤差是拍照當下的：用那時候的朝向算目標角度，轉動中才不會多轉
        double captured = headingAt(now - vision.getFrameLatencySeconds() - kExtraLatency.get());
        // 誤差為正 = 目標在射球點往車頭方向 → 逆時針轉（射擊方向往車頭那側）
        double desired = captured + Math.atan2(e, shooterRay);
        double limit = Math.toRadians(kMaxTurnDeg.get());
        double wanted = MathUtil.angleModulus(desired - startHeading);
        double rel = Math.clamp(wanted, -limit, limit);
        targetHeading = startHeading + rel;
        clampedFrames = rel != wanted && !turning ? clampedFrames + 1 : 0;
        seenTimer.restart();
        // 停著的時候拍到的畫面在容許誤差內才算數
        settledFrames = !turning && Math.abs(e) <= kTolerance.get() ? settledFrames + 1 : 0;
      }
    }

    double wheel = 0;
    double errMeters = Double.NaN;
    if (isLocked()) {
      double err = MathUtil.angleModulus(targetHeading - h);
      errMeters = shooterRay * Math.tan(err);
      if (Math.abs(errMeters) > kTolerance.get()) {
        wheel = Math.clamp(kP.get() * err, -kMaxWheel.get(), kMaxWheel.get());
        if (Math.abs(wheel) < kMinWheel.get()) wheel = Math.copySign(kMinWheel.get(), err);
      }
    }
    // 逆時針：左輪往後、右輪往前
    drive.setVelocity(-wheel, wheel);
    turning = wheel != 0;

    Logger.recordOutput("VisionTurnAlign/Locked", isLocked());
    // 已扣掉 AimTrim：對準完成時這個 ≈ 0
    Logger.recordOutput("VisionTurnAlign/ErrorMeters", lastError);
    Logger.recordOutput("VisionTurnAlign/HeadingErrorMeters", errMeters);
    Logger.recordOutput("VisionTurnAlign/TurnedDeg", Math.toDegrees(turnedRad()));
    Logger.recordOutput("VisionTurnAlign/WheelMps", wheel);
    Logger.recordOutput("VisionTurnAlign/SettledFrames", settledFrames);
    Logger.recordOutput("VisionTurnAlign/Clamped", clampedFrames > 0);
    Logger.recordOutput("VisionTurnAlign/AimTrimMeters", bias);
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
    return !Double.isNaN(targetHeading);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
    succeeded = !interrupted && settledFrames >= kSettleFrames;
    Logger.recordOutput("VisionTurnAlign/Success", succeeded);
  }

  /** 上一次執行是否成功對準。給自動流程決定要不要重試。 */
  public boolean succeeded() {
    return succeeded;
  }

  @Override
  public boolean isFinished() {
    if (settledFrames >= kSettleFrames) return true;
    if (clampedFrames >= kClampedGiveUpFrames) return true;
    double lostTimeout = isLocked() ? kLostTimeoutSec : kAcquireTimeoutSec;
    return seenTimer.hasElapsed(lostTimeout) || totalTimer.hasElapsed(kTotalTimeoutSec);
  }
}
