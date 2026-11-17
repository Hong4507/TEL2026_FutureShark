package frc.FSLib.rangefinder;

import static org.wpilib.units.Units.Radians;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;

import frc.FSLib.rangefinder.DualTFLunaRangeSensor.Reading;

/**
 * 車身兩顆 TF-Luna ＋ IMU ＋ Limelight 融合後的距離（規則見 {@link RangeFusion}）。
 * 由 {@link RangeSensors#installFusion} 換上，之後所有 {@code RangeSensors.get()} 都拿到這個。
 *
 * <p>NT：{@code RangeSensor/Source}（Luna×2／Luna單邊+IMU／Limelight／預設）、{@code RangeSensor/TargetDistance}、
 * {@code RangeSensor/LunaOnlyDistance}（原本只用 Luna 的值，對照用）、{@code RangeSensor/VisionDistance}、
 * {@code RangeSensor/WallHeadingRad}、{@code RangeSensor/ImuYawErrorDeg}。
 */
public class FusedRangeSensor implements TargetRangeSensor {
  private static final double kCacheSeconds = 0.005;

  private final DualTFLunaRangeSensor luna;
  private final DoubleSupplier gyroYawRad;
  private final Supplier<OptionalDouble> visionDistance;
  private final RangeFusion fusion = new RangeFusion();

  private RangeFusion.Result result = new RangeFusion.Result(
      RangeSensorConstants.kFallbackDistanceMeters, Double.NaN, Double.NaN, RangeFusion.Source.FALLBACK);
  private double cachedTime = Double.NEGATIVE_INFINITY;

  public FusedRangeSensor(DualTFLunaRangeSensor luna, DoubleSupplier gyroYawRad, Supplier<OptionalDouble> visionDistance) {
    this.luna = luna;
    this.gyroYawRad = gyroYawRad;
    this.visionDistance = visionDistance;
  }

  public DualTFLunaRangeSensor luna() {
    return luna;
  }

  private RangeFusion.Result update() {
    double now = Timer.getTimestamp();
    if (now - cachedTime < kCacheSeconds) return result;
    cachedTime = now;
    double lunaOnly = luna.getTargetDistance(); // 也會寫原本的 RangeSensor/* log
    Reading r = luna.getReading();
    OptionalDouble vision = visionDistance.get();
    result = fusion.update(now, r.left(), r.right(), r.leftValid(), r.rightValid(), gyroYawRad.getAsDouble(), vision);

    Logger.recordOutput("RangeSensor/Source", result.source().label);
    Logger.recordOutput("RangeSensor/TargetDistance", result.targetDistance());
    Logger.recordOutput("RangeSensor/WallDistance", result.wallDistance());
    Logger.recordOutput("RangeSensor/LunaOnlyDistance", lunaOnly);
    Logger.recordOutput("RangeSensor/VisionDistance", vision.orElse(Double.NaN));
    Logger.recordOutput("RangeSensor/WallHeadingRad", fusion.getWallHeading());
    Logger.recordOutput("RangeSensor/ImuYawErrorDeg", Math.toDegrees(result.yawRad()));
    // 11/19 校正視覺距離用：Luna×2 時 Luna ÷ 視覺，乘上目前的 /Tuning/Vision/WidthScale 就是新值
    Logger.recordOutput("Vision/DistanceRatio", vision.isPresent() && result.source() == RangeFusion.Source.LUNA_DUAL
        ? lunaOnly / vision.getAsDouble() : Double.NaN);
    return result;
  }

  public RangeFusion.Source getSource() {
    return update().source();
  }

  @Override
  public double getTargetDistance() {
    return update().targetDistance();
  }

  @Override
  public OptionalDouble getWallDistanceMeters() {
    double wall = update().wallDistance();
    return Double.isNaN(wall) ? OptionalDouble.empty() : OptionalDouble.of(wall);
  }

  /** 兩顆 Luna 一致時用 Luna 偏角；一顆被擋時改用 IMU（LaserAlign 不會因為一顆讀不到就直接結束）。 */
  @Override
  public Angle getYaw() {
    double yaw = update().yawRad();
    if (Double.isNaN(yaw)) return null;
    Logger.recordOutput("RangeSensor/YawRad", Math.PI / 2 + yaw);
    return Radians.of(Math.PI / 2 + yaw);
  }
}
