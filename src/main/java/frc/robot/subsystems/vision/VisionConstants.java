package frc.robot.subsystems.vision;

/**
 * 九宮格視覺的常數，以及與 Jetson {@code vision_nt_publisher.py} 共用的 NetworkTables 格式。
 *
 * <p>NT 表 {@code /Vision}（Jetson 發布）：
 * <ul>
 *   <li>{@code targets}  double[]：每個偵測 {@link #kTargetStride} 個數字，依序為
 *       類別、水平角 tx、垂直角 ty、信心度、角寬度、亮燈分數（見 {@code kIdx*}）</li>
 *   <li>{@code heartbeat} int：每處理完一張畫面 +1</li>
 *   <li>{@code latencyMs} double：從拿到畫面到發布的時間</li>
 *   <li>{@code fps} double：Jetson 實際處理幀率</li>
 * </ul>
 *
 * <p>角度慣例（攝影機座標，已去畸變）：tx 往畫面右邊為正、ty 往上為正，單位 rad。
 */
public final class VisionConstants {
  private VisionConstants() {}

  // ---------- NT 格式（改這裡要同步改 jetson_tools/vision_nt_publisher.py）----------
  public static final String kTableName = "Vision";
  public static final String kTargetsTopic = "targets";
  public static final String kHeartbeatTopic = "heartbeat";
  public static final String kLatencyTopic = "latencyMs";
  public static final String kFpsTopic = "fps";

  public static final int kTargetStride = 6;
  public static final int kIdxClass = 0;
  public static final int kIdxTx = 1;
  public static final int kIdxTy = 2;
  public static final int kIdxConfidence = 3;
  public static final int kIdxWidth = 4;
  public static final int kIdxLitScore = 5;

  /** YOLO 類別編號，要和訓練用的 dataset.yaml 一致。 */
  public static final int kClassHole = 0;
  public static final int kClassBonus = 1;

  /** 超過這個時間（秒）heartbeat 沒變，就當作 Jetson 斷線。 */
  public static final double kTimeoutSeconds = 0.25;

  // ---------- 攝影機安裝 ----------
  /**
   * 攝影機裝在機器人側面、面向九宮格。機器人沿牆面直走就是水平移動。
   * 攝影機朝機器人右側時，畫面右邊 = 機器人後方，這個值為 false（與舊版 Align 的方向相同）。
   * ⚠️ 11/14 實機確認：機器人往前開時，目標應該往畫面右邊移動才對（false 的情況）。
   */
  public static final boolean kImageRightIsRobotForward = false;

  /**
   * 射球點在攝影機前方（機器人前進方向）多少公尺。對準時目標要出現在射球點正前方。
   * 沿用舊版 Align 的 0.15 m。
   */
  public static final double kShooterForwardOfCameraMeters = 0.15;

  /** 攝影機高度（公尺），只用在模擬。 */
  public static final double kCameraHeightMeters = 0.45;

  // ---------- 九宮格幾何（規則 p.13）----------
  /** 相鄰兩欄中心距：洞徑 40 cm + 間距 50 cm。 */
  public static final double kColumnSpacingMeters = 0.90;
  /** 偵測框分到同一欄的容許距離（沿牆方向）。 */
  public static final double kColumnClusterMeters = 0.25;

  // ---------- 預設參數（可在 NT /Tuning/VisionAlign/* 現場調整）----------
  public static final double kDefaultMinConfidence = 0.5;
  /** 加分洞外圈紅色像素比例超過這個值，就判定為亮燈。11/14 用 LED 燈條實測後調整。 */
  public static final double kDefaultLitThreshold = 0.15;
}
