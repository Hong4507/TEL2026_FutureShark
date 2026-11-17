package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Timer;

/**
 * 模擬用：依機器人 pose 與九宮格位置算出攝影機會看到的偵測結果，用來在模擬器測 VisionAlign。
 *
 * <p>假設攝影機在機器人中心、朝機器人右側（面向九宮格），水平視角 {@link #kHorizontalFovRad}。
 * 加分洞隨機一格亮燈。
 *
 * <p>延遲：發布的偵測結果是 {@link #latencySeconds} 之前的 pose 看到的畫面（曝光 + USB + 推論 + NT），
 * 和實機一樣，讓 VisionAlign 的速度調整在模擬裡不會太樂觀。
 */
public class VisionIOSim implements VisionIO {
  private static final double kHorizontalFovRad = Math.toRadians(kHorizontalFovDeg);
  /** Limelight 2 實測 VFOV 48.9°。 */
  private static final double kVerticalFovRad = Math.toRadians(48.9);
  private static final double kHoleDiameterMeters = 0.40;
  private static final double kBonusDiameterMeters = 0.20;
  private static final double kNoiseRad = Math.toRadians(0.15);

  /** 預設總延遲（秒）。Jetson 實測從抓到畫面到發布約 20 ms，另估曝光與 USB 約 40 ms。 */
  public static final double kDefaultLatencySeconds = 0.060;

  /** 所有 VisionIOSim 共用的延遲，測試可以改。 */
  public static double latencySeconds = kDefaultLatencySeconds;

  /** 預設幀率。jsing wb1 webcam 實測約 15 fps（2026-11-14）。 */
  public static final double kDefaultFrameRateHz = 30;

  /** 所有 VisionIOSim 共用的幀率，測試可以改。 */
  public static double frameRateHz = kDefaultFrameRateHz;

  /** 模擬攝影機 USB 斷線：這段時間（Timer.getTimestamp()）內沒有新畫面。預設 NaN = 不斷線。 */
  public static double dropoutFrom = Double.NaN;
  public static double dropoutUntil = Double.NaN;

  private final Supplier<Pose2d> poseSupplier;
  /** {時間, pose} 歷史，用來取 latencySeconds 之前的 pose。 */
  private final ArrayDeque<Object[]> poseHistory = new ArrayDeque<>();
  private final Random random = new Random();
  private final int litColumn;

  private long heartbeat = 0;
  private double lastFrameTime = Double.NEGATIVE_INFINITY;
  private double[] lastTargets = new double[0];

  public VisionIOSim(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    this.litColumn = random.nextInt(3);
  }

  /** 指定亮燈欄（0 = LEFT、1 = MID、2 = RIGHT）與亂數種子，給單元測試用。 */
  public VisionIOSim(Supplier<Pose2d> poseSupplier, int litColumn, long seed) {
    this.poseSupplier = poseSupplier;
    this.litColumn = litColumn;
    random.setSeed(seed);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getTimestamp();
    poseHistory.addLast(new Object[] {now, poseSupplier.get()});
    while (poseHistory.size() > 1 && (double) poseHistory.peekFirst()[0] < now - latencySeconds - 0.5) {
      poseHistory.removeFirst();
    }
    boolean dropped = now >= dropoutFrom && now < dropoutUntil;
    inputs.newFrame = !dropped && now - lastFrameTime >= 1.0 / frameRateHz;
    if (inputs.newFrame) {
      lastFrameTime = now;
      heartbeat++;
      lastTargets = render(poseAt(now - latencySeconds));
    }
    inputs.connected = true;
    inputs.heartbeat = heartbeat;
    inputs.targets = lastTargets;
    inputs.latencyMs = latencySeconds * 1000;
    inputs.fps = frameRateHz;
  }

  /** 最接近 time（不晚於 time）的歷史 pose；歷史不夠舊時用最舊的。 */
  private Pose2d poseAt(double time) {
    Pose2d best = (Pose2d) poseHistory.peekFirst()[1];
    for (Object[] entry : poseHistory) {
      if ((double) entry[0] > time) break;
      best = (Pose2d) entry[1];
    }
    return best;
  }

  private double[] render(Pose2d pose) {
    double heading = pose.getRotation().getRadians();
    double hx = Math.cos(heading), hy = Math.sin(heading);
    // 攝影機朝機器人右側
    double fx = Math.cos(heading - Math.PI / 2), fy = Math.sin(heading - Math.PI / 2);
    double sign = kImageRightIsRobotForward ? 1 : -1;
    double rx = sign * hx, ry = sign * hy;

    double[] columnsY = {kLeftTargetY, kMidTargetY, kRightTargetY};
    double[] holeHeights = {kLowTargetHeight, kMidTargetHeight, kHighTargetHeight};

    List<double[]> out = new ArrayList<>();
    for (int c = 0; c < 3; c++) {
      for (double z : holeHeights) {
        add(out, pose, fx, fy, rx, ry, kTargetX, columnsY[c], z, kClassHole, kHoleDiameterMeters, -1);
      }
      double lit = c == litColumn ? 0.6 : 0.02;
      add(out, pose, fx, fy, rx, ry, kTargetX, columnsY[c], kSpecialTargetHeight, kClassBonus, kBonusDiameterMeters, lit);
    }

    double[] flat = new double[out.size() * kTargetStride];
    for (int i = 0; i < out.size(); i++) {
      System.arraycopy(out.get(i), 0, flat, i * kTargetStride, kTargetStride);
    }
    return flat;
  }

  private void add(List<double[]> out, Pose2d pose, double fx, double fy, double rx, double ry,
      double x, double y, double z, int cls, double diameter, double litScore) {
    double dx = x - pose.getX(), dy = y - pose.getY();
    double depth = dx * fx + dy * fy;
    if (depth <= 0.3) return;
    double right = dx * rx + dy * ry;
    // 攝影機往上仰 kCameraPitchDeg：世界座標（前 depth、右 right、上 up）→ 相機座標
    double up = z - kCameraHeightMeters;
    double pitch = Math.toRadians(kCameraPitchDeg);
    double zc = depth * Math.cos(pitch) + up * Math.sin(pitch);
    double yc = -depth * Math.sin(pitch) + up * Math.cos(pitch);
    if (zc <= 0.1) return;
    double tx = Math.atan2(right, zc) + random.nextGaussian() * kNoiseRad;
    if (Math.abs(tx) > kHorizontalFovRad / 2) return;
    double ty = Math.atan2(yc, Math.hypot(right, zc));
    if (Math.abs(ty) > kVerticalFovRad / 2) return;
    // 和 Jetson 一樣：框左右兩邊（同一列）的水平角度差
    double width = Math.atan((right + diameter / 2) / zc) - Math.atan((right - diameter / 2) / zc);
    out.add(new double[] {cls, tx, ty, 0.9, width, litScore});
  }
}
