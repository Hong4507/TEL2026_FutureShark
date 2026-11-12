package frc.robot.subsystems.scoreboard;

import static frc.robot.Constants.FieldConstants.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.command2.SubsystemBase;

import frc.FSLib.rangefinder.RangeSensors;
import frc.robot.commands.AutoRoutines.Half;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Column;

/**
 * 記分板：記錄九宮格 + 加分格每一格射了幾發，給 Elastic 顯示（版面：{@code elastic-layout.json}）。
 *
 * <p>機器人沒有進球偵測，所以這裡記的是「射擊數」：
 * <ul>
 *   <li>ColumnShot 射出的球：列 = 瞄準高度，欄 = 視覺判斷射球點對著哪一欄，自動 +1。</li>
 *   <li>手動 / Assist 射出的球、或視覺分辨不出欄位：記在「未歸類」，由操作員在 Elastic 按 +1 歸到正確的格子。</li>
 *   <li>沒進的球：操作員按 −1。</li>
 * </ul>
 *
 * <p>NT：
 * <ul>
 *   <li>顯示 {@code /AdvantageKit/RealOutputs/Scoreboard/<欄>_<列>}：例如 {@code ●●○}（實心 = 已記、空心 = 還能算分），
 *       超過上限顯示 {@code +n}；亮燈的加分格前面加 ★。</li>
 *   <li>按鈕 {@code /Scoreboard/Plus/<欄>_<列>}、{@code /Scoreboard/Minus/<欄>_<列>}、{@code /Scoreboard/Reset}
 *       （Elastic Toggle Button；機器人讀到 true 就動作並改回 false）。</li>
 * </ul>
 */
public class Scoreboard extends SubsystemBase {
  /** 由上到下：加分格、高、中、低。 */
  public enum Row {
    BONUS(kSpecialTargetHeight), HIGH(kHighTargetHeight), MID(kMidTargetHeight), LOW(kLowTargetHeight);

    public final double heightMeters;

    Row(double heightMeters) {
      this.heightMeters = heightMeters;
    }

    /** 依瞄準高度找列；不是這四個高度就回傳 empty。 */
    static Optional<Row> fromHeight(double heightMeters) {
      for (Row r : values()) {
        if (Math.abs(r.heightMeters - heightMeters) < 0.05) return Optional.of(r);
      }
      return Optional.empty();
    }
  }

  private final Turret turret;
  private final Vision vision;
  private final Supplier<Half> half;

  private final int[][] counts = new int[Row.values().length][Column.values().length];
  private int unassigned = 0;
  private Column litColumn = null;
  private int lastShotCount;

  private final LoggedNetworkBoolean[][] plusButtons = new LoggedNetworkBoolean[Row.values().length][Column.values().length];
  private final LoggedNetworkBoolean[][] minusButtons = new LoggedNetworkBoolean[Row.values().length][Column.values().length];
  private final LoggedNetworkBoolean resetButton = new LoggedNetworkBoolean("/Scoreboard/Reset", false);
  private final LoggedNetworkBoolean clearUnassignedButton = new LoggedNetworkBoolean("/Scoreboard/ClearUnassigned", false);

  public Scoreboard(Turret turret, Vision vision, Supplier<Half> half) {
    this.turret = turret;
    this.vision = vision;
    this.half = half;
    this.lastShotCount = turret.getShotCount();
    for (Row r : Row.values()) {
      for (Column c : Column.values()) {
        plusButtons[r.ordinal()][c.ordinal()] = new LoggedNetworkBoolean("/Scoreboard/Plus/" + key(r, c), false);
        minusButtons[r.ordinal()][c.ordinal()] = new LoggedNetworkBoolean("/Scoreboard/Minus/" + key(r, c), false);
      }
    }
  }

  public static String key(Row row, Column column) {
    return column.name() + "_" + row.name();
  }

  /** 比賽開始時歸零（自動流程啟動時呼叫，Elastic 也有按鈕）。 */
  public void reset() {
    for (int[] row : counts) java.util.Arrays.fill(row, 0);
    unassigned = 0;
    litColumn = null;
  }

  public int getCount(Row row, Column column) {
    return counts[row.ordinal()][column.ordinal()];
  }

  public int getUnassigned() {
    return unassigned;
  }

  @Override
  public void periodic() {
    double distance = RangeSensors.get().getTargetDistance();

    // 亮燈欄：看到就記住（燈只亮前 60 秒，之後加分格也不算分）
    vision.getLitColumn(distance).ifPresent(c -> litColumn = c);

    // 新射出的球
    int shots = turret.getShotCount();
    for (; lastShotCount < shots; lastShotCount++) {
      Optional<Row> row = Row.fromHeight(turret.getLastShotAimHeight());
      Optional<Column> column = row.isPresent() && row.get() == Row.BONUS && litColumn != null
        ? Optional.of(litColumn)
        : vision.getAimedColumn(distance);
      if (row.isPresent() && column.isPresent()) {
        counts[row.get().ordinal()][column.get().ordinal()]++;
      } else {
        unassigned++;
      }
    }

    // Elastic 按鈕
    for (Row r : Row.values()) {
      for (Column c : Column.values()) {
        if (consume(plusButtons[r.ordinal()][c.ordinal()])) {
          counts[r.ordinal()][c.ordinal()]++;
          if (unassigned > 0) unassigned--; // 通常是把未歸類的球歸到這格
        }
        if (consume(minusButtons[r.ordinal()][c.ordinal()])) {
          counts[r.ordinal()][c.ordinal()] = Math.max(0, counts[r.ordinal()][c.ordinal()] - 1);
        }
      }
    }
    if (consume(resetButton)) reset();
    if (consume(clearUnassignedButton)) unassigned = 0;

    publish();
  }

  private static boolean consume(LoggedNetworkBoolean button) {
    if (!button.get()) return false;
    button.set(false);
    return true;
  }

  private void publish() {
    Half h = half.get();
    for (Row r : Row.values()) {
      int cap = r == Row.BONUS ? h.bonusCap : h.holeCap;
      for (Column c : Column.values()) {
        String text = dots(counts[r.ordinal()][c.ordinal()], cap);
        if (r == Row.BONUS && c == litColumn) text = "★ " + text;
        Logger.recordOutput("Scoreboard/" + key(r, c), text);
      }
    }
    Logger.recordOutput("Scoreboard/Unassigned", unassigned == 0 ? "0" : unassigned + " 發待歸類");
    Logger.recordOutput("Scoreboard/Score", estimatedScore(h) + " 分");
    Logger.recordOutput("Scoreboard/Half", h == Half.FIRST ? "上半場" : "下半場");
  }

  /** ●●○：實心 = 已記（算分的部分），空心 = 還能算分；超過上限加 +n。 */
  static String dots(int count, int cap) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < cap; i++) sb.append(i < count ? '●' : '○');
    if (count > cap) sb.append(" +").append(count - cap);
    return sb.toString();
  }

  /** 依目前記錄估算分數（假設記到的都有進）。 */
  int estimatedScore(Half h) {
    int score = 0;
    boolean[][] has = new boolean[3][3]; // [列 高中低][欄]
    for (Row r : Row.values()) {
      for (Column c : Column.values()) {
        int n = counts[r.ordinal()][c.ordinal()];
        if (r == Row.BONUS) {
          if (c == litColumn) score += Math.min(n, h.bonusCap) * 4;
        } else {
          score += Math.min(n, h.holeCap) * 2;
          has[r.ordinal() - 1][c.ordinal()] = n > 0;
        }
      }
    }
    if (h == Half.SECOND) {
      for (int i = 0; i < 3; i++) {
        if (has[i][0] && has[i][1] && has[i][2]) score += 2; // 水平
        if (has[0][i] && has[1][i] && has[2][i]) score += 4; // 垂直
      }
      if (has[0][0] && has[1][1] && has[2][2]) score += 8; // 斜線
      if (has[0][2] && has[1][1] && has[2][0]) score += 8;
    }
    return score;
  }
}
