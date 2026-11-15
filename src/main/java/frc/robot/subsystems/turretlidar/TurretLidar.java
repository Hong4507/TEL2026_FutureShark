package frc.robot.subsystems.turretlidar;

import java.util.OptionalDouble;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.filter.Debouncer;

import frc.FSLib.rangefinder.RangeSensorConstants;
import frc.robot.Constants.FieldConstants;

/**
 * 砲台正前方的第三顆 TF-Luna（固定、不跟砲台仰角動，本身有一個固定仰角），接在 Jetson 的 I2C，經 NetworkTables 傳過來。
 *
 * <p>用途：<b>只用來確認左右對準了某一欄</b>（光束穿過洞 = 對準）。選哪一列由 ShotTable 調仰角。
 * 結果只在 Elastic 亮燈，<b>不會阻止射擊</b>。
 *
 * <p>判斷：光束打在板子上 → 讀到「到板子的距離」；穿過洞 → 讀到後面更遠的東西，或後面沒東西 → TF-Luna 回傳 0（超出範圍／訊號太弱）。
 * <ul>
 *   <li>{@link Beam#THROUGH_HOLE}：讀值 0／訊號太弱，或比預期的板子距離遠 {@link #kHoleMarginMeters} 以上。</li>
 *   <li>{@link Beam#BOARD}：讀值在預期板子距離 ± margin 內。</li>
 *   <li>{@link Beam#BLOCKED}：比板子近很多（前面有球、有人、另一台車）。</li>
 *   <li>{@link Beam#NO_DATA}：Jetson 沒資料。</li>
 * </ul>
 *
 * <p>⚠️ 九宮格板是黑色，TF-Luna 在 10% 反射率時官方量程只有約 2.5 m：如果打在板子上也讀到 0，這個方法就分不出來。
 * 上機時一定要先確認射擊距離下「打在板子上」讀得到正常距離（Checklist §5.8）。
 */
public class TurretLidar extends SubsystemBase {
  /** 超過這麼久沒收到新讀值就算斷線（Jetson 約 100 Hz 發布）。 */
  public static final double kTimeoutSeconds = 0.25;

  // ===== 安裝幾何（TODO 11/15 上機量測） =====
  /** 鏡頭離地高度（公尺）。 */
  public static final double kMountHeightMeters = 0.52;
  /** 光束仰角（度，往上為正）。 */
  public static final double kMountPitchDeg = 0.0;
  /**
   * 砲台 Luna 鏡頭在右 frame 內側多少（公尺）。鏡頭到板面的水平距離 = 車身 Luna 牆距（右 frame → 九宮格）+ 這個值（§8.4）。
   * 【待量 11/15】量到前先用 kShooterInboardMeters 的暫定值。
   */
  public static final double kInboardMeters = 0.25;
  /** 光束要穿過哪一列（§8.4 定案：中列）。 */
  public static final double kAimRowHeightMeters = FieldConstants.kMidTargetHeight;
  /** 光束打在板上的高度離洞中心超過（洞半徑 − 這個值）就算「超出範圍」，不判斷穿洞。 */
  public static final double kRowEdgeMarginMeters = 0.05;

  // ===== 判斷門檻 =====
  /** 讀值比預期板子距離遠超過這個值 → 穿過洞。 */
  public static final double kHoleMarginMeters = 0.3;
  /** 讀值比預期板子距離近超過這個值 → 前面被擋住。 */
  public static final double kBlockedMarginMeters = 0.3;
  /** 狀態要連續維持這麼久才算數，避免邊緣閃爍。 */
  public static final double kDebounceSeconds = 0.06;
  /** 洞的半徑（Ø40 cm），用來估光束打在哪一列。 */
  private static final double kHoleRadiusMeters = 0.20;

  /** OUT_OF_RANGE：距離不在有效範圍（光束打不到目標列）或沒有牆距；DISABLED：NT 關掉了。 */
  public enum Beam { NO_DATA, BOARD, THROUGH_HOLE, BLOCKED, OUT_OF_RANGE, DISABLED }

  /** §8.4：上機測 10 分鐘，讀不到黑板就在 Elastic／NT 把這個關掉（顯示「停用」）。 */
  private static final LoggedNetworkBoolean enabled = new LoggedNetworkBoolean("/Tuning/TurretLidar/Enabled", true);

  private final TurretLidarIO io;
  private final TurretLidarIOInputsAutoLogged inputs = new TurretLidarIOInputsAutoLogged();
  /** 右 frame 到九宮格牆面的距離（RangeSensor/WallDistance），無效時 empty。 */
  private final Supplier<OptionalDouble> wallDistanceMeters;
  private final Debouncer holeDebouncer = new Debouncer(kDebounceSeconds, Debouncer.DebounceType.BOTH);

  private Beam beam = Beam.NO_DATA;
  private boolean throughHole = false;

  public TurretLidar(TurretLidarIO io, Supplier<OptionalDouble> wallDistanceMeters) {
    this.io = io;
    this.wallDistanceMeters = wallDistanceMeters;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TurretLidar", inputs);

    OptionalDouble wall = wallDistanceMeters.get();
    double horizontal = wall.isPresent() ? wall.getAsDouble() + kInboardMeters : Double.NaN;
    double expected = expectedBoardDistance(horizontal);
    double beamHeight = beamHeightAtBoard(horizontal);
    if (!enabled.get()) {
      beam = Beam.DISABLED;
    } else {
      beam = classify(inputs.connected, inputs.distanceMeters, inputs.strength, expected);
      if (beam != Beam.NO_DATA && !inValidRange(beamHeight)) beam = Beam.OUT_OF_RANGE;
    }
    throughHole = holeDebouncer.calculate(beam == Beam.THROUGH_HOLE);

    Logger.recordOutput("TurretLidar/Valid", isValid());
    Logger.recordOutput("TurretLidar/ExpectedBoardMeters", expected);
    Logger.recordOutput("TurretLidar/BeamHeightAtBoardMeters", beamHeight);
    Logger.recordOutput("TurretLidar/BeamRow", rowAtHeight(beamHeight));
    Logger.recordOutput("TurretLidar/Beam", beam.name());
    Logger.recordOutput("TurretLidar/ThroughHole", throughHole);
  }

  /** 沿光束方向到板面的距離。 */
  static double expectedBoardDistance(double horizontalMeters) {
    return horizontalMeters / Math.cos(Math.toRadians(kMountPitchDeg));
  }

  /** 光束打到板面時的高度。 */
  static double beamHeightAtBoard(double horizontalMeters) {
    return kMountHeightMeters + horizontalMeters * Math.tan(Math.toRadians(kMountPitchDeg));
  }

  /**
   * 光束打在板上的高度是否夠接近目標列的洞中心（|高度 − 列高| ≤ 洞半徑 − margin）。
   * 換成水平距離約 ±(0.2 − 0.05) / tan(光束仰角)。NaN（沒有牆距）→ false。
   */
  static boolean inValidRange(double beamHeightMeters) {
    return Math.abs(beamHeightMeters - kAimRowHeightMeters) <= kHoleRadiusMeters - kRowEdgeMarginMeters;
  }

  /** 給 Elastic 的中文狀態。 */
  public String getBeamText() {
    return switch (beam) {
      case NO_DATA -> "沒資料";
      case BOARD -> "打在板上";
      case THROUGH_HOLE -> "穿過洞";
      case BLOCKED -> "被擋住";
      case OUT_OF_RANGE -> "超出範圍";
      case DISABLED -> "停用";
    };
  }

  /** 光束高度落在哪一列洞的範圍內（LOW／MID／HIGH），都不在 → NONE（這樣永遠穿不過洞，要改安裝角度）。 */
  static String rowAtHeight(double heightMeters) {
    if (Math.abs(heightMeters - FieldConstants.kLowTargetHeight) <= kHoleRadiusMeters) return "LOW";
    if (Math.abs(heightMeters - FieldConstants.kMidTargetHeight) <= kHoleRadiusMeters) return "MID";
    if (Math.abs(heightMeters - FieldConstants.kHighTargetHeight) <= kHoleRadiusMeters) return "HIGH";
    return "NONE";
  }

  /** 純判斷，不含 debounce。distance = 0 或訊號太弱代表「前方範圍內沒東西」。 */
  static Beam classify(boolean connected, double distanceMeters, double strength, double expectedMeters) {
    if (!connected || Double.isNaN(distanceMeters)) return Beam.NO_DATA;
    boolean noReturn = distanceMeters < RangeSensorConstants.kMinValidRangeMeters
        || strength < RangeSensorConstants.kMinStrength;
    if (noReturn || distanceMeters > expectedMeters + kHoleMarginMeters) return Beam.THROUGH_HOLE;
    if (distanceMeters < expectedMeters - kBlockedMarginMeters) return Beam.BLOCKED;
    return Beam.BOARD;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  /** 光束穿過洞（已 debounce）。只給顯示用。 */
  public boolean isThroughHole() {
    return throughHole;
  }

  public Beam getBeam() {
    return beam;
  }

  /** 有連線且讀值可信（範圍與訊號強度門檻和左右兩顆 TF-Luna 相同）。 */
  public boolean isValid() {
    return inputs.connected
        && inputs.distanceMeters >= RangeSensorConstants.kMinValidRangeMeters
        && inputs.distanceMeters <= RangeSensorConstants.kMaxValidRangeMeters
        && inputs.strength >= RangeSensorConstants.kMinStrength
        && inputs.strength != RangeSensorConstants.kSaturatedStrength;
  }

  /** 原始距離（公尺）；讀值不可信（包含 TF-Luna 回傳 0）時 empty。 */
  public OptionalDouble getDistanceMeters() {
    return isValid() ? OptionalDouble.of(inputs.distanceMeters) : OptionalDouble.empty();
  }
}
