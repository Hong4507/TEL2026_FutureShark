package frc.robot.subsystems.status;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.RobotController;

import frc.FSLib.rangefinder.DualTFLunaRangeSensor;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.Constants;
import frc.robot.commands.AutoRoutines;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turretlidar.TurretLidar;
import frc.robot.subsystems.vision.Vision;

/**
 * Elastic「狀態」分頁的資料來源：機器人模式、射擊狀態、各部件連線。
 * 第二操作手不能碰電腦，所以這頁只有顯示、沒有按鈕，而且盡量做成一眼看得懂的文字。
 *
 * <p>全部發布在 {@code /AdvantageKit/RealOutputs/Status/...}（版面：{@code elastic-layout.json}）。
 */
public class StatusBoard extends SubsystemBase {
  /** 電池低於這個電壓就在警示列提醒。 */
  public static final double kLowBatteryVolts = 11.5;
  /** READY 燈的電池門檻（§8.9）：先通電排隊會耗電，不夠就換電池，不要硬上。 */
  public static final double kReadyBatteryVolts = 12.5;
  /** READY 燈的 Jetson 幀率門檻。 */
  public static final double kReadyVisionFps = 10;
  /** 飛輪低於這個速度（rad/s）視為停止。 */
  private static final double kStoppedRadPerSec = 5;

  private final Turret turret;
  private final TurretLidar turretLidar;
  private final Vision vision;
  /** 目前選的自動流程；還沒選時回傳 null。 */
  private final Supplier<AutoRoutines.Plan> autoPlan;

  /** 名稱 → 是否連線；順序就是警示列的順序。 */
  private final Map<String, BooleanSupplier> connections = new LinkedHashMap<>();

  public StatusBoard(Drive drive, Turret turret, Vision vision, TurretLidar turretLidar, BooleanSupplier flySkyConnected,
      Supplier<AutoRoutines.Plan> autoPlan) {
    this.vision = vision;
    this.autoPlan = autoPlan;
    this.turret = turret;
    this.turretLidar = turretLidar;

    TargetRangeSensor range = RangeSensors.get();
    DualTFLunaRangeSensor luna = range instanceof DualTFLunaRangeSensor d ? d
        : range instanceof frc.FSLib.rangefinder.FusedRangeSensor f ? f.luna() : null;

    connections.put("FlySky", flySkyConnected);
    connections.put("Vision", vision::isConnected);
    connections.put("TFLunaLeft", () -> luna == null || luna.isLeftConnected());
    connections.put("TFLunaRight", () -> luna == null || luna.isRightConnected());
    connections.put("TurretLidar", turretLidar::isConnected);
    connections.put("ShooterUp", turret::isUpConnected);
    connections.put("ShooterDown", turret::isDownConnected);
    connections.put("PitchEncoder", turret::isEncoderConnected);
    connections.put("Gyro", drive::isGyroConnected);
  }

  /** 警示列用的中文名稱。 */
  private static String label(String key) {
    return switch (key) {
      case "FlySky" -> "遙控器";
      case "Vision" -> "Jetson視覺";
      case "TFLunaLeft" -> "左TF-Luna";
      case "TFLunaRight" -> "右TF-Luna";
      case "TurretLidar" -> "砲台TF-Luna";
      case "ShooterUp" -> "上飛輪";
      case "ShooterDown" -> "下飛輪";
      case "PitchEncoder" -> "仰角編碼器";
      case "Gyro" -> "IMU";
      default -> key;
    };
  }

  /**
   * READY 條件：連線全部正常（含仰角編碼器）、電池 &gt; {@link #kReadyBatteryVolts}、Jetson fps &gt; {@link #kReadyVisionFps}、
   * Auto 已選。回傳不 READY 的原因（空 = READY）。
   */
  static List<String> readyProblems(List<String> connectionProblems, double batteryVolts, double visionFps,
      boolean autoSelected) {
    List<String> out = new ArrayList<>(connectionProblems);
    out.remove("電池低");
    if (batteryVolts <= kReadyBatteryVolts) {
      out.add(String.format(Locale.ROOT, "電池 %.1fV（要 >%.1f）", batteryVolts, kReadyBatteryVolts));
    }
    if (visionFps <= kReadyVisionFps) {
      out.add(String.format(Locale.ROOT, "視覺 %.0ffps（要 >%.0f）", visionFps, kReadyVisionFps));
    }
    if (!autoSelected) out.add("Auto 未選");
    return out;
  }

  @Override
  public void periodic() {
    // ---- 模式 ----
    String mode = switch (Constants.kRobotMode) {
      case kManual -> "手動";
      case kRealManual -> "真手動";
      case kAssist -> "輔助";
      case kAuto -> "自動";
      case kDisable -> "未選模式";
    };
    boolean enabled = RobotState.isEnabled();
    Logger.recordOutput("Status/Mode", mode + (enabled ? "・啟用" : "・停用"));
    Logger.recordOutput("Status/Enabled", enabled);

    // ---- 射擊 ----
    double target = TurretConstants.kMaxSpeedRadPerSec;
    boolean ready = turret.isNearTargetSpeeds(target, target * 0.8);
    double up = turret.getUpVelocityRadPerSec();
    String shooter = ready ? "就緒"
        : Math.abs(up) < kStoppedRadPerSec ? "停止"
        : String.format(Locale.ROOT, "加速 %d%%", Math.round(100 * up / target));
    Logger.recordOutput("Status/Shooter", shooter);
    Logger.recordOutput("Status/ShooterReady", ready);
    double pitch = turret.getPitchPositionRad();
    Logger.recordOutput("Status/Pitch", String.format(Locale.ROOT, "%.3f rad / %.1f°", pitch, Math.toDegrees(pitch)));
    Logger.recordOutput("Status/Shots", turret.getShotCount() + " 發");

    // ---- 距離 ----
    TargetRangeSensor rs = RangeSensors.get();
    String source = rs instanceof frc.FSLib.rangefinder.FusedRangeSensor f ? " " + f.getSource().label : "";
    Logger.recordOutput("Status/Distance", String.format(Locale.ROOT, "%.2f m%s", rs.getTargetDistance(), source));
    // 雙Luna距離：不管融合現在用哪個來源，永遠顯示兩顆 Luna 公式算的值（對照用）
    frc.FSLib.rangefinder.DualTFLunaRangeSensor luna =
        rs instanceof frc.FSLib.rangefinder.FusedRangeSensor f ? f.luna()
            : rs instanceof frc.FSLib.rangefinder.DualTFLunaRangeSensor d ? d : null;
    String dualText = "—";
    double dualMeters = Double.NaN;
    if (luna != null) {
      var r = luna.getReading();
      if (r.leftValid() && r.rightValid()) {
        var sol = frc.FSLib.rangefinder.DualTFLunaRangeSensor.solve(r.left(), r.right(), true, true);
        dualMeters = sol.targetDistance();
        dualText = String.format(Locale.ROOT, "%.3f m  偏 %+.1f°", dualMeters, Math.toDegrees(sol.yawRad()));
      } else {
        dualText = !r.leftValid() && !r.rightValid() ? "兩顆都無效" : !r.leftValid() ? "左無效" : "右無效";
      }
    }
    Logger.recordOutput("Status/DualLunaDistance", dualText);
    Logger.recordOutput("RangeSensor/DualLunaDistance", dualMeters);
    // TF-Luna 前方範圍內沒東西或訊號太弱時回傳 0，不能直接顯示 0（看起來像貼著東西）
    String lidarText = !turretLidar.isConnected() ? "未連線"
        : turretLidar.getDistanceMeters().isPresent()
            ? String.format(Locale.ROOT, "%.2f m", turretLidar.getDistanceMeters().getAsDouble())
            : "超出範圍";
    Logger.recordOutput("Status/TurretLidarDistance", lidarText);
    Logger.recordOutput("Status/TurretLidarBeam", turretLidar.getBeamText());
    Logger.recordOutput("Status/ThroughHole", turretLidar.isThroughHole());

    // ---- 電池 ----
    double battery = RobotController.getBatteryVoltage();
    Logger.recordOutput("Status/Battery", String.format(Locale.ROOT, "%.1f V", battery));

    // ---- 連線 + 警示 ----
    List<String> problems = new ArrayList<>();
    for (var e : connections.entrySet()) {
      boolean ok = e.getValue().getAsBoolean();
      Logger.recordOutput("Status/Conn/" + e.getKey(), ok);
      if (!ok) problems.add(label(e.getKey()));
    }
    if (battery < kLowBatteryVolts) problems.add("電池低");

    // ---- 自動流程選擇（大字，§8.9：選錯邊是最容易發生、後果最嚴重的錯誤）----
    AutoRoutines.Plan plan = autoPlan.get();
    Logger.recordOutput("Status/AutoSide", plan == null ? "未選" : plan.side().label);
    Logger.recordOutput("Status/AutoHalf", plan == null ? "未選" : plan.halfLabel());
    Logger.recordOutput("Status/AutoColumn", plan == null ? "未選"
        : plan.method() == AutoRoutines.AutoMethod.LUNA ? "第一個洞" : plan.column().label);
    Logger.recordOutput("Status/AutoPath", plan == null ? "未選"
        : plan.method() == AutoRoutines.AutoMethod.LUNA ? (plan.side() == AutoRoutines.StartSide.RIGHT ? "Luna往車頭找" : "Luna往車尾找")
        : plan.runPath() ? "跑路徑" : "不跑路徑");
    Logger.recordOutput("Status/AutoSummary", plan == null ? "⚠ Auto 未選" : plan.describe());

    // ---- READY 燈（§8.9）----
    List<String> notReady = readyProblems(problems, battery, vision.getFps(), plan != null);
    Logger.recordOutput("Status/Ready", notReady.isEmpty());
    Logger.recordOutput("Status/ReadyText", notReady.isEmpty() ? "READY" : "未就緒：" + String.join("、", notReady));
    Logger.recordOutput("Status/Alert", notReady.isEmpty() ? "全部正常" : "⚠ " + String.join("、", notReady));
  }
}
