package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.TimestampedInteger;
import org.wpilib.system.Timer;

/** 從 NetworkTables 讀 Jetson（或 vision_fake_publisher.py）發布的偵測結果。 */
public class VisionIONT implements VisionIO {
  private final DoubleArraySubscriber targetsSub;
  private final IntegerSubscriber heartbeatSub;
  private final DoubleSubscriber latencySub;
  private final DoubleSubscriber fpsSub;

  private long lastHeartbeat = Long.MIN_VALUE;
  private double lastUpdateTime = Double.NEGATIVE_INFINITY;

  public VisionIONT() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(kTableName);
    targetsSub = table.getDoubleArrayTopic(kTargetsTopic).subscribe(new double[0]);
    heartbeatSub = table.getIntegerTopic(kHeartbeatTopic).subscribe(0);
    latencySub = table.getDoubleTopic(kLatencyTopic).subscribe(0);
    fpsSub = table.getDoubleTopic(kFpsTopic).subscribe(0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    TimestampedInteger hb = heartbeatSub.getAtomic();
    long heartbeat = hb.value;
    double now = Timer.getTimestamp();
    // timestamp == 0：從沒收過 Jetson 的值（訂閱者預設值），不能算新畫面，不然開機會假連線 0.25 秒
    boolean received = hb.timestamp != 0;
    inputs.newFrame = received && heartbeat != lastHeartbeat;
    if (inputs.newFrame) {
      lastHeartbeat = heartbeat;
      lastUpdateTime = now;
    }
    inputs.connected = now - lastUpdateTime < kTimeoutSeconds;
    inputs.heartbeat = heartbeat;
    inputs.targets = inputs.connected ? targetsSub.get() : new double[0];
    inputs.latencyMs = latencySub.get();
    // 只用機器人自己的時鐘：距離上次收到新 heartbeat 多久。
    // 不用 hb.timestamp：那是發布端的時間，跨程序/跨機器沒同步時會差到幾十秒（11/12 e2e 測到 70~130 s）。
    inputs.ageMs =
        lastUpdateTime == Double.NEGATIVE_INFINITY ? Double.NaN : (now - lastUpdateTime) * 1000.0;
    inputs.fps = fpsSub.get();
  }
}
