package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turretlidar.TurretLidar;

/**
 * 沿牆直走，用砲台 TF-Luna 找洞並停在洞中間（不用視覺）。
 *
 * <ol>
 *   <li>APPROACH：先走 {@code ApproachMeters}（不看 Luna）。</li>
 *   <li>SEARCH：慢慢走，光束穿過洞 → 記下「進洞」位置。最多再走 {@code SearchMeters}。</li>
 *   <li>IN_HOLE：繼續走到光束離開洞 → 記下「出洞」位置；目標 = 兩者中間（停在洞的第一個邊緣會偏 15 cm）。</li>
 *   <li>CENTER：用里程計開回中間。</li>
 * </ol>
 * 走不動（被對手擋住，0.5 s 內前進 &lt; 2 cm）→ {@link Result#BLOCKED}。結果看 {@link #result()}。
 *
 * <p>方向：dir = +1 往車頭、-1 往車尾。射球朝機器人右側，所以右起點往車頭走會靠近中欄，左起點要往車尾。
 */
public class LidarHoleAlign extends Command {
  public enum Result { NOT_RUN, CENTERED, IN_HOLE, NOT_FOUND, BLOCKED, UNAVAILABLE }

  private enum State { APPROACH, BACKUP, SEARCH, IN_HOLE, CENTER, DONE }

  private static final LoggedNetworkNumber kApproachMeters = new LoggedNetworkNumber("/Tuning/LunaAuto/ApproachMeters", 0.3);
  private static final LoggedNetworkNumber kSearchMeters = new LoggedNetworkNumber("/Tuning/LunaAuto/SearchMeters", 1.5);
  private static final LoggedNetworkNumber kApproachSpeed = new LoggedNetworkNumber("/Tuning/LunaAuto/ApproachSpeedMps", 0.4);
  private static final LoggedNetworkNumber kSearchSpeed = new LoggedNetworkNumber("/Tuning/LunaAuto/SearchSpeedMps", 0.12);

  /** 進洞後最多走這麼遠還沒出洞 → 不等了，停在「進洞 + 0.15 m」。 */
  private static final double kMaxHoleWidthMeters = 0.5;
  /** 洞寬小於這個值 → 當成雜訊，繼續找。 */
  private static final double kMinHoleWidthMeters = 0.05;
  private static final double kCenterKp = 2.0;
  private static final double kCenterMaxSpeed = 0.2;
  private static final double kCenterMinSpeed = 0.04;
  private static final double kCenterToleranceMeters = 0.01;
  /** 偵測延遲（debounce + Jetson/NT 約 20 ms）：進洞、出洞都晚這麼久，中點會往前偏 速度 × 延遲，要扣回來。 */
  private static final double kDetectLatencySec = TurretLidar.kDebounceSeconds + 0.02;
  private static final double kCenterTimeoutSec = 3.0;
  private static final double kStallWindowSec = 0.5;
  private static final double kStallMinProgressMeters = 0.02;

  private final java.util.function.DoubleSupplier approach;
  private final java.util.function.DoubleSupplier search;
  private final Drive drive;
  private final TurretLidar lidar;
  private final double dir;

  private State state;
  private Result result = Result.NOT_RUN;
  private Translation2d origin;
  private Translation2d axis;
  private double entry, exit, target;
  private int settled;
  private final Timer stallTimer = new Timer();
  private final Timer centerTimer = new Timer();
  private double stallRef;
  private boolean firstSearchCycle;
  private State lastState;

  public LidarHoleAlign(Drive drive, TurretLidar lidar, double dir) {
    this(drive, lidar, dir, kApproachMeters::get, kSearchMeters::get);
  }

  /** 手動對齊用：自己給前進／搜尋距離（不用自動流程的 NT 參數）。 */
  public LidarHoleAlign(Drive drive, TurretLidar lidar, double dir, double approachMeters, double searchMeters) {
    this(drive, lidar, dir, () -> approachMeters, () -> searchMeters);
  }

  private LidarHoleAlign(Drive drive, TurretLidar lidar, double dir,
      java.util.function.DoubleSupplier approachMeters, java.util.function.DoubleSupplier searchMeters) {
    this.approach = approachMeters;
    this.search = searchMeters;
    this.drive = drive;
    this.lidar = lidar;
    this.dir = Math.signum(dir) == 0 ? 1 : Math.signum(dir);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d p = drive.getPose();
    origin = p.getTranslation();
    axis = new Translation2d(p.getRotation().getCos(), p.getRotation().getSin()).times(dir);
    state = approach.getAsDouble() > 0 ? State.APPROACH : State.SEARCH;
    firstSearchCycle = true;
    lastState = state;
    result = lidar.isUsable() ? Result.NOT_RUN : Result.UNAVAILABLE;
    settled = 0;
    stallRef = 0;
    stallTimer.restart();
    log();
  }

  /** 沿出發時的方向走了多遠（往 dir 方向為正）。 */
  private double progress() {
    Translation2d d = drive.getPose().getTranslation().minus(origin);
    return d.getX() * axis.getX() + d.getY() * axis.getY();
  }

  @Override
  public void execute() {
    double s = progress();
    boolean hole = lidar.isRawThroughHole();
    double speed = 0;
    switch (state) {
      case APPROACH -> {
        speed = kApproachSpeed.get();
        if (s >= approach.getAsDouble()) state = result == Result.UNAVAILABLE ? State.DONE : State.SEARCH;
      }
      case BACKUP -> {
        // 開始找的時候已經在洞裡：不知道進洞邊緣在哪，先倒退到洞外，再重新往前量
        speed = -kSearchSpeed.get();
        if (!hole) state = State.SEARCH;
      }
      case SEARCH -> {
        speed = kSearchSpeed.get();
        boolean first = firstSearchCycle;
        firstSearchCycle = false;
        if (result == Result.UNAVAILABLE) {
          state = State.DONE;
        } else if (hole && first) {
          state = State.BACKUP;
        } else if (hole) {
          entry = s;
          state = State.IN_HOLE;
        } else if (s - Math.max(approach.getAsDouble(), 0) > search.getAsDouble()) {
          finish(Result.NOT_FOUND);
        }
      }
      case IN_HOLE -> {
        speed = kSearchSpeed.get();
        if (!hole) {
          exit = s;
          if (exit - entry < kMinHoleWidthMeters) {
            state = State.SEARCH;
          } else {
            target = (entry + exit) / 2 - kSearchSpeed.get() * kDetectLatencySec;
            startCenter();
          }
        } else if (s - entry > kMaxHoleWidthMeters) {
          target = entry + 0.15;
          startCenter();
        }
      }
      case CENTER -> {
        double err = target - s;
        if (Math.abs(err) <= kCenterToleranceMeters) {
          settled++;
        } else {
          settled = 0;
          speed = Math.clamp(kCenterKp * err, -kCenterMaxSpeed, kCenterMaxSpeed);
          if (Math.abs(speed) < kCenterMinSpeed) speed = Math.copySign(kCenterMinSpeed, err);
        }
        if (settled >= 3 || centerTimer.hasElapsed(kCenterTimeoutSec)) {
          finish(hole ? Result.CENTERED : Result.IN_HOLE);
        }
      }
      case DONE -> {}
    }

    // 換狀態（例如倒退 → 往前）時速度會先降到 0，重新計算「被擋住」的時間窗，不然會誤判
    if (state != lastState) {
      stallRef = s;
      stallTimer.restart();
      lastState = state;
    }
    // 被擋住：有在往前推，0.5 s 內卻幾乎沒前進
    if (state == State.APPROACH || state == State.BACKUP || state == State.SEARCH || state == State.IN_HOLE) {
      if (stallTimer.hasElapsed(kStallWindowSec)) {
        if (Math.abs(s - stallRef) < kStallMinProgressMeters) {
          finish(state == State.IN_HOLE && hole ? Result.IN_HOLE : Result.BLOCKED);
          speed = 0;
        }
        stallRef = s;
        stallTimer.restart();
      }
    }
    if (state == State.DONE) speed = 0;
    drive.setVelocity(speed * dir, speed * dir);
    Logger.recordOutput("LunaAuto/ProgressMeters", s);
    log();
  }

  private void startCenter() {
    state = State.CENTER;
    settled = 0;
    centerTimer.restart();
  }

  private void finish(Result r) {
    result = r;
    state = State.DONE;
  }

  private void log() {
    Logger.recordOutput("LunaAuto/AlignState", state.name());
    Logger.recordOutput("LunaAuto/AlignResult", result.name());
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
    if (state != State.DONE && result == Result.NOT_RUN) result = Result.NOT_FOUND;
    Logger.recordOutput("LunaAuto/EntryMeters", entry);
    Logger.recordOutput("LunaAuto/ExitMeters", exit);
    log();
  }

  @Override
  public boolean isFinished() {
    return state == State.DONE;
  }

  public Result result() {
    return result;
  }

  /** 光束在洞裡（可以射）。 */
  public boolean found() {
    return result == Result.CENTERED || result == Result.IN_HOLE;
  }
}
