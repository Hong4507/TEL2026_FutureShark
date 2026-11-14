package frc.robot.subsystems.turretlidar;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
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
  /** 鏡頭到板面的水平距離 = TargetDistance + 這個值（鏡頭比砲台射球點更靠近板子 → 負值）。 */
  public static final double kHorizontalOffsetMeters = 0.0;

  // ===== 判斷門檻 =====
  /** 讀值比預期板子距離遠超過這個值 → 穿過洞。 */
  public static final double kHoleMarginMeters = 0.3;
  /** 讀值比預期板子距離近超過這個值 → 前面被擋住。 */
  public static final double kBlockedMarginMeters = 0.3;
  /** 狀態要連續維持這麼久才算數，避免邊緣閃爍。 */
  public static final double kDebounceSeconds = 0.06;
  /** 洞的半徑（Ø40 cm），用來估光束打在哪一列。 */
  private static final double kHoleRadiusMeters = 0.20;

  public enum Beam { NO_DATA, BOARD, THROUGH_HOLE, BLOCKED }

  private final TurretLidarIO io;
  private final TurretLidarIOInputsAutoLogged inputs = new TurretLidarIOInputsAutoLogged();
  /** 砲台到目標的水平距離（RangeSensor/TargetDistance）。 */
  private final DoubleSupplier targetDistanceMeters;
  private final Debouncer holeDebouncer = new Debouncer(kDebounceSeconds, Debouncer.DebounceType.BOTH);

  private Beam beam = Beam.NO_DATA;
  private boolean throughHole = false;

  public TurretLidar(TurretLidarIO io, DoubleSupplier targetDistanceMeters) {
    this.io = io;
    this.targetDistanceMeters = targetDistanceMeters;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TurretLidar", inputs);

    double horizontal = targetDistanceMeters.getAsDouble() + kHorizontalOffsetMeters;
    double expected = expectedBoardDistance(horizontal);
    double beamHeight = beamHeightAtBoard(horizontal);
    beam = classify(inputs.connected, inputs.distanceMeters, inputs.strength, expected);
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
