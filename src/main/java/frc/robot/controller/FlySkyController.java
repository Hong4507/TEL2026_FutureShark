package frc.robot.controller;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.framework.RobotBase;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;

/**
 * FlySky 遙控器。
 *
 * <p>接收機的 i-BUS 經 USB 轉 TTL 模組直接接到 SystemCore 的 USB，由 {@link IBusSerialReader} 讀取。
 *
 * <p>備援：USB-TTL 沒資料時，改讀 Jetson 透過 NetworkTables 發布的 {@code /FlySky/channels}、
 * {@code /FlySky/heartbeat}（舊接法，jetson_tools/flysky_nt_publisher.py）。模擬器只用 NT。
 */
public class FlySkyController {
  public double getRawAxis(int axis) {
    return getRawAxisMap.get(axis).getAsDouble();
  }

  public boolean getRawButton(int button) {
    return getRawBottonMap.get(button).getAsBoolean();
  }

  private final Map<Integer, DoubleSupplier> getRawAxisMap = Map.of(
    Axis.kLeftX.value, this::getLeftX,
    Axis.kLeftY.value, this::getLeftY,
    Axis.kRightX.value, this::getRightX,
    Axis.kRightY.value, this::getRightY
  );

  private final Map<Integer, BooleanSupplier> getRawBottonMap = Map.of(
    Button.kLeftSwitch.value, this::getLeftSwitch,
    Button.kRightSwitch.value, this::getRightSwitch
  );

  public enum Button {
    /** Leftmost switch */
    kLeftSwitch(8),
    /** Second Leftmost switch. */
    kRightSwitch(9);

    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  public enum Axis {
    /** Left X axis. */
    kLeftX(4),
    /** Right X axis. */
    kRightX(7),
    /** Left Y axis. */
    kLeftY(5),
    /** Right Y axis. */
    kRightY(6);

    /** Axis value. */
    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This
     * is done by
     * stripping the leading `k`, and appending `Axis` if the name ends with
     * `Trigger`.
     *
     * <p>
     * Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  /**
   * FlySky 各功能對應的 iBus 通道（0 = CH1）。已依實際遙控器確認（2026-11-07）。
   * 若遙控器的通道設定改變，請修改這裡。
   */
  public static final class IBusChannel {
    public static final int kLeftX = 0;       // CH1
    public static final int kLeftY = 1;       // CH2
    public static final int kRightY = 2;      // CH3
    public static final int kRightX = 3;      // CH4
    public static final int kLeftSwitch = 4;  // CH5
    public static final int kRightSwitch = 5; // CH6
  }

  /**
   * USB 轉 TTL 模組（CP2102）在 SystemCore 上的裝置檔，依序嘗試（第一個存在的就用）。
   * 優先用 /dev/serial/by-id 的固定路徑，重開機或換 USB 孔都不會變；找不到才退回 ttyUSB0。
   * 注意：/dev/ttyACM0 是 Limelight，絕對不能列進來。
   */
  public static final String[] kSerialPorts = {
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    "/dev/ttyUSB0"
  };

  /** 備援：Jetson 發布的 NetworkTables 表名稱（模擬器的 sim_smoke_test.py 也用這個）。 */
  public static final String kTableName = "FlySky";
  /** iBus 各通道脈寬（μs，約 1000~2000），double[]，index 0 = CH1。 */
  public static final String kChannelsTopic = "channels";
  /** 每收到一個 iBus 封包就 +1 的計數器，用來判斷 Jetson 是否還在傳資料。 */
  public static final String kHeartbeatTopic = "heartbeat";
  /** 超過這個時間（秒）沒有新封包，就視為斷線，所有搖桿歸零、開關為 false。 */
  public static final double kTimeoutSeconds = 0.1;

  /** 資料來源，給 Log 看目前用哪一個。 */
  public enum Source { kSerial, kNetworkTables, kNone }

  /** 實機上直接讀 USB-TTL；模擬器上為 null（只用 NT）。 */
  private final IBusSerialReader serialReader;
  private final DoubleArraySubscriber channelsSub;
  private final IntegerSubscriber heartbeatSub;

  private final int leftXChannel;
  private final int leftYChannel;
  private final int rightXChannel;
  private final int rightYChannel;
  private final int leftSwitchChannel;
  private final int rightSwitchChannel;

  private long lastHeartbeat = Long.MIN_VALUE;
  private double lastUpdateTime = Double.NEGATIVE_INFINITY;

  private boolean lastLeftSwitch1Value = false;
  private boolean lastLeftSwitch2Value = false;

  /**
   * @param leftXChannel 各參數為 iBus 通道 index（0 = CH1）
   */
  public FlySkyController(
    int leftXChannel, int leftYChannel, int rightXChannel, int rightYChannel, int leftSwitchChannel, int rightSwitchChannel
  ) {
    serialReader = RobotBase.isReal() ? new IBusSerialReader(kSerialPorts) : null;

    NetworkTable table = NetworkTableInstance.getDefault().getTable(kTableName);
    channelsSub = table.getDoubleArrayTopic(kChannelsTopic).subscribe(new double[0]);
    heartbeatSub = table.getIntegerTopic(kHeartbeatTopic).subscribe(0);

    this.leftXChannel = leftXChannel;
    this.leftYChannel = leftYChannel;
    this.rightXChannel = rightXChannel;
    this.rightYChannel = rightYChannel;
    this.leftSwitchChannel = leftSwitchChannel;
    this.rightSwitchChannel = rightSwitchChannel;
  }

  public FlySkyController() {
    this(
      IBusChannel.kLeftX, IBusChannel.kLeftY, IBusChannel.kRightX, IBusChannel.kRightY,
      IBusChannel.kLeftSwitch, IBusChannel.kRightSwitch
    );
  }

  /** USB-TTL 在 {@link #kTimeoutSeconds} 內有收到正確封包。 */
  private boolean isSerialConnected() {
    if (serialReader == null) return false;
    IBusSerialReader.Frame frame = serialReader.getLatestFrame();
    return frame != null && frame.ageSeconds() < kTimeoutSeconds;
  }

  /** Jetson 在 {@link #kTimeoutSeconds} 內有透過 NT 送來新的 iBus 封包。 */
  private boolean isNetworkTablesConnected() {
    long heartbeat = heartbeatSub.get();
    double now = Timer.getTimestamp();
    if (heartbeat != lastHeartbeat) {
      lastHeartbeat = heartbeat;
      lastUpdateTime = now;
    }
    return now - lastUpdateTime < kTimeoutSeconds;
  }

  /** 目前使用的資料來源：USB-TTL 優先，沒資料時改用 Jetson（NT）。 */
  public Source getSource() {
    if (isSerialConnected()) return Source.kSerial;
    if (isNetworkTablesConnected()) return Source.kNetworkTables;
    return Source.kNone;
  }

  /**
   * 是否在 {@link #kTimeoutSeconds} 內收到新的 iBus 封包（USB-TTL 或 Jetson 任一個）。
   *
   * @return true 表示連線正常
   */
  public boolean isConnected() {
    return getSource() != Source.kNone;
  }

  /** 各通道脈寬（μs），斷線時為空陣列。 */
  public double[] getChannelsMicroseconds() {
    switch (getSource()) {
      case kSerial: {
        int[] raw = serialReader.getLatestFrame().channels();
        double[] us = new double[raw.length];
        for (int i = 0; i < raw.length; i++) us[i] = raw[i];
        return us;
      }
      case kNetworkTables:
        return channelsSub.get();
      default:
        return new double[0];
    }
  }

  /** 把連線狀態與通道值記到 AdvantageKit Log（每個 loop 呼叫一次）。 */
  public void logTelemetry() {
    Logger.recordOutput("FlySky/Source", getSource().name());
    Logger.recordOutput("FlySky/ChannelsUs", getChannelsMicroseconds());
    if (serialReader != null) {
      Logger.recordOutput("FlySky/Serial/Port", String.valueOf(serialReader.getOpenPort()));
      Logger.recordOutput("FlySky/Serial/Frames", serialReader.getFrameCount());
      Logger.recordOutput("FlySky/Serial/BadFrames", serialReader.getBadFrames());
    }
  }

  /**
   * 讀取某個 iBus 通道並正規化到 -1.0 ~ 1.0（1500μs = 0）。斷線或沒有該通道時回傳 0。
   */
  private double getChannel(int channel) {
    double[] channels = getChannelsMicroseconds();
    if (channel < 0 || channel >= channels.length) return 0;
    double normalized = (channels[channel] - 1500) / 500;
    return Math.max(-1, Math.min(1, normalized));
  }

  /**
   * Read the value of the leftmost switch on the controller.
   *
   * @return The state of the switch.
   */
  public boolean getLeftSwitch() {
    lastLeftSwitch1Value = getChannel(leftSwitchChannel) > 0;
    return lastLeftSwitch1Value;
  }

  /**
   * Whether the leftmost switch was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftSwitch1Pressed() {
    boolean result = !lastLeftSwitch1Value && getLeftSwitch();
    lastLeftSwitch1Value = getLeftSwitch();
    return result;
  }

  /**
   * Whether the leftmost switch was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftSwitch1Released() {
    boolean result = lastLeftSwitch1Value && !getLeftSwitch();
    lastLeftSwitch1Value = getLeftSwitch();
    return result;
  }

    /**
   * Read the value of the second leftmost switch on the controller.
   *
   * @return The state of the switch.
   */
  public boolean getRightSwitch() {
    lastLeftSwitch2Value = getChannel(rightSwitchChannel) > 0;
    return lastLeftSwitch2Value;
  }

  /**
   * Whether the second leftmost switch was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftSwitch2Pressed() {
    boolean result = !lastLeftSwitch2Value && getRightSwitch();
    lastLeftSwitch2Value = getRightSwitch();
    return result;
  }

  /**
   * Whether the second leftmost was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftSwitch2Released() {
    boolean result = lastLeftSwitch2Value && !getRightSwitch();
    lastLeftSwitch2Value = getRightSwitch();
    return result;
  }

  /**
   * Get the X axis value of left side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getChannel(leftXChannel);
  }

  /**
   * Get the X axis value of right side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return getChannel(rightXChannel);
  }

  /**
   * Get the Y axis value of left side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getChannel(leftYChannel);
  }

  /**
   * Get the Y axis value of right side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return getChannel(rightYChannel);
  }
}
