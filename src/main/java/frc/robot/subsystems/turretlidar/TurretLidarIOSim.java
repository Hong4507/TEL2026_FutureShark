package frc.robot.subsystems.turretlidar;

import static frc.robot.Constants.FieldConstants.*;

import java.util.function.Supplier;

import org.wpilib.math.geometry.Pose2d;

/**
 * 模擬砲台 TF-Luna：從機器人位置往右側（射球方向）以 {@link TurretLidar#kMountPitchDeg} 仰角打一道光束到九宮格平面
 * （x = kTargetX，板子 85° 當成垂直），光點整個落在洞內 → 讀 0（穿過洞，後面沒東西）；否則讀到板子的斜距。
 *
 * <p>光點大小：TF-Luna 視角 2°，半徑 ≈ 斜距 × tan 1°。鏡頭位置當成在機器人中心（實機深度只差 1 cm）。
 */
public class TurretLidarIOSim implements TurretLidarIO {
  private static final double kHoleRadius = 0.20;
  private static final double kBonusRadius = 0.10;
  private static final double[] kColumns = {kLeftTargetY, kMidTargetY, kRightTargetY};
  private static final double[] kRows = {kLowTargetHeight, kMidTargetHeight, kHighTargetHeight};

  private final Supplier<Pose2d> pose;
  /** 洞後面黑箱子離板面多遠（公尺，水平）；NaN = 沒有箱子（穿過洞讀 0）。 */
  private final double boxBehindMeters;
  private long heartbeat = 0;

  public TurretLidarIOSim(Supplier<Pose2d> pose) {
    this(pose, Double.NaN);
  }

  public TurretLidarIOSim(Supplier<Pose2d> pose, double boxBehindMeters) {
    this.pose = pose;
    this.boxBehindMeters = boxBehindMeters;
  }

  @Override
  public void updateInputs(TurretLidarIOInputs inputs) {
    double d = read(pose.get(), boxBehindMeters);
    inputs.connected = true;
    inputs.distanceMeters = d;
    inputs.strength = d == 0 ? 0 : 1000;
    inputs.heartbeat = ++heartbeat;
  }

  /** 光束讀值（公尺）；穿過洞或打不到板子 = 0。 */
  public static double read(Pose2d p) {
    return read(p, Double.NaN);
  }

  /** @param boxBehind 洞後面箱子離板面的水平距離；NaN = 沒箱子 */
  public static double read(Pose2d p, double boxBehind) {
    double h = p.getRotation().getRadians();
    double dx = Math.sin(h), dy = -Math.cos(h); // 機器人右側方向
    if (dx <= 0.05) return 0;
    double horizontal = (kTargetX - p.getX()) / dx;
    if (horizontal <= 0) return 0;
    double pitch = Math.toRadians(TurretLidar.kMountPitchDeg);
    double y = p.getY() + dy * horizontal;
    double z = TurretLidar.kMountHeightMeters + horizontal * Math.tan(pitch);
    double slant = horizontal / Math.cos(pitch);
    double spot = slant * Math.tan(Math.toRadians(1.0));
    for (double cy : kColumns) {
      for (double cz : kRows) {
        if (Math.hypot(y - cy, z - cz) < kHoleRadius - spot) return throughHole(horizontal, dx, pitch, boxBehind);
      }
      if (Math.hypot(y - cy, z - kSpecialTargetHeight) < kBonusRadius - spot) return throughHole(horizontal, dx, pitch, boxBehind);
    }
    return slant > 8 ? 0 : slant;
  }

  /** 穿過洞：沒箱子 → 0；有箱子 → 光束繼續走到箱子的斜距。 */
  private static double throughHole(double horizontal, double dx, double pitch, double boxBehind) {
    if (Double.isNaN(boxBehind)) return 0;
    return (horizontal + boxBehind / dx) / Math.cos(pitch);
  }
}
