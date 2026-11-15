package frc.FSLib.rangefinder;

import org.wpilib.hardware.bus.I2C;

public final class RangeSensorConstants {
  private RangeSensorConstants() {}

  public static final String kRPLidarTable = "LaserScan";

  /**
   * 兩顆 TF-Luna 的 I2C 埠與位址（TF-Luna 預設 0x10）。
   * 左右以「機器人面向目標」為準：左邊那顆接 I2C 0、右邊那顆接 I2C 1，兩顆都用出廠位址，不用改設定。
   * 若兩顆接同一個埠，必須先把其中一顆改成不同位址（例如 0x11），再改這裡。
   */
  public static final I2C.Port kLeftTFLunaPort = I2C.Port.PORT_0;
  public static final int kLeftTFLunaAddress = 0x10;
  public static final I2C.Port kRightTFLunaPort = I2C.Port.PORT_1;
  public static final int kRightTFLunaAddress = 0x10;

  /**
   * 左右 TF-Luna 量哪一面牆（Robot-Brief §8.1／§8.2）。
   * <ul>
   *   <li>{@link RangeReference#GRID_WALL}：光束直接打九宮格牆面（11/15 起的預設）。</li>
   *   <li>{@link RangeReference#LOW_WALL}：備案。黑板讀不到（讀值 0／Amp 太低）時，把 Luna 調回對矮牆再切這個。
   *       用舊公式（+ {@link #kWallToTargetMeters}），預設常數和舊版完全相同。</li>
   * </ul>
   * 改完要重新部署。<b>量插值表和比賽要用同一個設定</b>（表是在這個公式下量的）。
   */
  public static final RangeReference kRangeReference = RangeReference.GRID_WALL;
  public enum RangeReference { GRID_WALL, LOW_WALL }

  // ===== GRID_WALL：左右 Luna 量九宮格牆面（11/14 實測幾何） =====
  /** 左 Luna 比右 Luna 靠車頭方向多少（公尺）= 基線。 */
  public static final double kBaselineMeters = 0.17072;
  /** 右 Luna 鏡頭在右 frame 內側多少（公尺）。 */
  public static final double kRightLunaInboardMeters = 0.07042;
  /** 左 Luna 鏡頭在右 frame 內側多少（公尺）（機器人寬 0.4953 − 左 frame 內側 0.0127）。 */
  public static final double kLeftLunaInboardMeters = 0.4826;
  /**
   * DownShooter（下飛輪，固定位置）輪心在右 frame（面向九宮格那側）內側多少（公尺），垂直牆面方向的深度。
   * 11/15 實測 169.53 mm。用輪心不用出球點：Hood 調仰角時出球點會動，輪心不會。
   * TargetDistance = 牆距 + 這個值；插值表的距離也是用它算的，改了要重量插值表。
   */
  public static final double kShooterInboardMeters = 0.16953;
  /**
   * 兩顆 Luna 的固定差值（公尺）：機器人擺正時 (dL − kLeftLunaInboard) − (dR − kRightLunaInboard)。
   * 【待量 11/15】看 NT {@code RangeSensor/RawFrameDiffMeters}，擺正時的值直接填進來。
   */
  public static final double kLunaTrimMeters = 0.0;

  // ===== LOW_WALL：舊公式（備案）=====
  /** 牆面（矮牆）到目標（九宮格）的距離（公尺）。只有 LOW_WALL 用。 */
  public static final double kWallToTargetMeters = 2.5;
  /** 舊公式的感測器 offset（公尺）。只有 LOW_WALL 用。 */
  public static final double kSensorOffsetMeters = 0.04;
  /**
   * 舊公式的基線（公尺）。只有 LOW_WALL 用。
   *
   * <p>⚠️ 舊公式沒有扣左右 Luna 的安裝內縮（兩顆擺正時差 0.412 m），角度會算錯。如果 11/15 真的要切 LOW_WALL
   * 而且 Luna 位置沒動，把下面三個改成 GRID_WALL 的值：{@code kLowWallBaselineMeters = 0.17072}、
   * {@code kLowWallLeftInboardMeters = 0.4826}、{@code kLowWallRightInboardMeters = 0.07042}，
   * 並把 {@code kLowWallYawSign} 改成 -1（左 Luna 在車頭，舊公式是用「右 − 左」算角度）。
   */
  public static final double kLowWallBaselineMeters = 0.30;
  public static final double kLowWallLeftInboardMeters = 0.0;
  public static final double kLowWallRightInboardMeters = 0.0;
  public static final double kLowWallYawSign = 1.0;

  // ===== 濾波／死區（§8.3）=====
  /** 左右 Luna 讀值的移動平均時間窗（秒）。0 = 不濾波。 */
  public static final double kLunaFilterSec = 0.2;
  /** LaserAlign 角度死區（度）：誤差小於這個值就不轉，避免在目標附近抖。 */
  public static final double kAlignDeadbandDeg = 1.5;

  /** TF-Luna 有效量測範圍（公尺）。 */
  public static final double kMinValidRangeMeters = 0.2;
  public static final double kMaxValidRangeMeters = 8.0;

  /** 訊號強度低於此值或等於 65535（過曝）時視為無效。 */
  public static final double kMinStrength = 100;
  public static final double kSaturatedStrength = 65535;

  /** 偏航角方向（GRID_WALL）。【待量 11/15】若 LaserAlign 越轉越歪（LaserAlign/WrongDirection = true），改成 -1。 */
  public static final double kYawSign = 1.0;

  /** 完全沒有有效資料時的距離（公尺），與 RPLidar 實作一致。 */
  public static final double kFallbackDistanceMeters = 3.0;
}
