package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.Command;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.drive.DifferentialDrive.WheelVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turretlidar.TurretLidar;

/**
 * 沿牆走不過去（被對手擋住）時：原地慢慢轉，讓射擊方向斜斜掃過九宮格，用砲台 TF-Luna 判斷有沒有穿過洞。
 * 和 {@link LidarHoleAlign} 一樣記下「進洞」「出洞」的角度，轉回兩者中間。
 *
 * <p>dir = +1 逆時針（射擊方向往車頭那側掃）、-1 順時針。轉了多少看 {@link #turnedRad()}（給射擊算斜距用）。
 * 掃到 {@code ScanMaxDeg} 還沒找到 → 轉回原來角度、{@link #found()} = false（不射）。
 */
public class LidarTurnScan extends Command {
  private enum State { SCAN, IN_HOLE, CENTER, RETURN, DONE }

  private static final LoggedNetworkNumber kMaxDeg = new LoggedNetworkNumber("/Tuning/LunaAuto/ScanMaxDeg", 40);
  /** 輪子線速度（m/s），0.04 ≈ 10°/s，光點在 3 m 外約 0.5 m/s。 */
  private static final LoggedNetworkNumber kScanWheelSpeed = new LoggedNetworkNumber("/Tuning/LunaAuto/ScanWheelMps", 0.04);

  private static final double kMinHoleWidthRad = Math.toRadians(0.8);
  private static final double kMaxHoleWidthRad = Math.toRadians(12);
  private static final double kKp = 0.5;
  private static final double kMaxTurn = 0.1;
  private static final double kMinTurn = 0.025;
  private static final double kToleranceRad = Math.toRadians(0.3);
  private static final double kCenterTimeoutSec = 3.0;

  private final Drive drive;
  private final TurretLidar lidar;
  private final double dir;

  private State state;
  private double startHeading;
  private double entry, exit, target;
  private int settled;
  private boolean found;
  private final Timer timer = new Timer();

  public LidarTurnScan(Drive drive, TurretLidar lidar, double dir) {
    this.drive = drive;
    this.lidar = lidar;
    this.dir = Math.signum(dir) == 0 ? 1 : Math.signum(dir);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startHeading = drive.getPose().getRotation().getRadians();
    state = lidar.isUsable() ? State.SCAN : State.DONE;
    found = false;
    settled = 0;
    log();
  }

  /** 從開始轉了多少（rad，往 dir 方向為正）。 */
  private double angle() {
    return MathUtil.angleModulus(drive.getPose().getRotation().getRadians() - startHeading) * dir;
  }

  /** 相對開始時轉了多少（rad，帶正負號，逆時針為正）。 */
  public double turnedRad() {
    return MathUtil.angleModulus(drive.getPose().getRotation().getRadians() - startHeading);
  }

  @Override
  public void execute() {
    double a = angle();
    boolean hole = lidar.isRawThroughHole();
    double turn = 0; // 往 dir 方向轉的輪速（m/s）
    switch (state) {
      case SCAN -> {
        turn = kScanWheelSpeed.get();
        if (hole) {
          entry = a;
          state = State.IN_HOLE;
        } else if (a > Math.toRadians(kMaxDeg.get())) {
          target = 0;
          startCenter(State.RETURN);
        }
      }
      case IN_HOLE -> {
        turn = kScanWheelSpeed.get();
        if (!hole) {
          exit = a;
          if (exit - entry < kMinHoleWidthRad) {
            state = State.SCAN;
          } else {
            target = (entry + exit) / 2;
            startCenter(State.CENTER);
          }
        } else if (a - entry > kMaxHoleWidthRad) {
          target = entry + kMaxHoleWidthRad / 4;
          startCenter(State.CENTER);
        }
      }
      case CENTER, RETURN -> {
        double err = target - a;
        if (Math.abs(err) <= kToleranceRad) {
          settled++;
        } else {
          settled = 0;
          turn = Math.clamp(kKp * err, -kMaxTurn, kMaxTurn);
          if (Math.abs(turn) < kMinTurn) turn = Math.copySign(kMinTurn, err);
        }
        if (settled >= 3 || timer.hasElapsed(kCenterTimeoutSec)) {
          found = state == State.CENTER && hole;
          state = State.DONE;
        }
      }
      case DONE -> {}
    }
    // arcadeDriveIK：zRotation 正 = 逆時針
    WheelVelocities w = DifferentialDrive.arcadeDriveIK(0, turn * dir, false);
    drive.setVelocity(w.left, w.right);
    Logger.recordOutput("LunaAuto/ScanDeg", Math.toDegrees(a));
    log();
  }

  private void startCenter(State next) {
    state = next;
    settled = 0;
    timer.restart();
  }

  private void log() {
    Logger.recordOutput("LunaAuto/ScanState", state.name());
    Logger.recordOutput("LunaAuto/ScanFound", found);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
    Logger.recordOutput("LunaAuto/ScanEntryDeg", Math.toDegrees(entry));
    Logger.recordOutput("LunaAuto/ScanExitDeg", Math.toDegrees(exit));
    log();
  }

  @Override
  public boolean isFinished() {
    return state == State.DONE;
  }

  /** 轉完後光束在洞裡（可以射）。 */
  public boolean found() {
    return found;
  }
}
