package frc.robot.subsystems.turretlidar;

import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.TimestampedInteger;
import org.wpilib.system.Timer;

/**
 * 讀 Jetson 上 {@code turret_tfluna_nt_publisher.py} 發布的砲台 TF-Luna。
 * NT 表 {@code TurretLidar}：distance（m）、strength、temperature（°C）、heartbeat（int）、i2cErrors（int）。
 */
public class TurretLidarIONT implements TurretLidarIO {
  public static final String kTableName = "TurretLidar";

  private final DoubleSubscriber distanceSub;
  private final DoubleSubscriber strengthSub;
  private final DoubleSubscriber temperatureSub;
  private final IntegerSubscriber heartbeatSub;
  private final IntegerSubscriber errorsSub;

  private long lastHeartbeat = Long.MIN_VALUE;
  private double lastUpdateTime = Double.NEGATIVE_INFINITY;

  public TurretLidarIONT() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(kTableName);
    distanceSub = table.getDoubleTopic("distance").subscribe(Double.NaN);
    strengthSub = table.getDoubleTopic("strength").subscribe(0);
    temperatureSub = table.getDoubleTopic("temperature").subscribe(Double.NaN);
    heartbeatSub = table.getIntegerTopic("heartbeat").subscribe(0);
    errorsSub = table.getIntegerTopic("i2cErrors").subscribe(0);
  }

  @Override
  public void updateInputs(TurretLidarIOInputs inputs) {
    TimestampedInteger hb = heartbeatSub.getAtomic();
    double now = Timer.getTimestamp();
    // timestamp == 0：從沒收過值（同 VisionIONT，避免開機假連線）
    if (hb.timestamp != 0 && hb.value != lastHeartbeat) {
      lastHeartbeat = hb.value;
      lastUpdateTime = now;
    }
    inputs.connected = now - lastUpdateTime < TurretLidar.kTimeoutSeconds;
    inputs.heartbeat = hb.value;
    inputs.i2cErrors = errorsSub.get();
    if (inputs.connected) {
      inputs.distanceMeters = distanceSub.get();
      inputs.strength = strengthSub.get();
      inputs.temperatureC = temperatureSub.get();
    } else {
      inputs.distanceMeters = Double.NaN;
      inputs.strength = 0;
      inputs.temperatureC = Double.NaN;
    }
  }
}
