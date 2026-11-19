package frc.robot.commands.turret;

import static frc.robot.Constants.FieldConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.FSLib.util.Util;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.TurretConstants;

/**
 * Assist 模式（11/19 改）：射大洞，仰角依車身 Luna 距離查 ShotTable 自動調整。
 *
 * <ul>
 *   <li>右搖桿選列（推進去就記住，放開後維持）：左下 = 低（Low）、右下 = 中（Mid）、右上 = 高（Top）。預設中列。
 *       正下方不選列，所以 Y 推到底微調仰角也不會換列。</li>
 *   <li>右搖桿 Y 微調仰角 ±{@link #kAdjustRad} rad（放開回到 0）。搖桿在選列區時不微調，選列不會順便把仰角往下拉。</li>
 *   <li>左搖桿 X 往右推 = 擊發一次（和 Manual 一樣：推下去的那一刻擊發 {@link #kTriggerSeconds} 秒，要再射就放開再推）。</li>
 * </ul>
 */
public class AssistTurret extends Command {
  private static final double kTriggerSeconds = 0.15;
  /** 右搖桿 Y 推到底時的仰角微調量（rad）。 */
  private static final double kAdjustRad = 0.1;

  public enum Row {
    HIGH("高", kHighTargetHeight), MID("中", kMidTargetHeight), LOW("低", kLowTargetHeight);

    public final String label;
    public final double heightMeters;

    Row(String label, double heightMeters) {
      this.label = label;
      this.heightMeters = heightMeters;
    }
  }

  private final Turret turret;
  private final DoubleSupplier adjustSupplier;
  private final BooleanSupplier fireSupplier;
  private final BooleanSupplier selectHigh;
  private final BooleanSupplier selectMid;
  private final BooleanSupplier selectLow;

  private final Timer triggerTimer = new Timer();
  private boolean lastFire = false;
  /** 診斷：這次進入 Assist 後擊發了幾次（左搖桿推一次 = 1）。 */
  private int fireCount = 0;
  /** 選的列在指令重新開始（右開關關掉再打開）時保留。 */
  private Row row = Row.MID;

  private final double targetVelocity = TurretConstants.kMaxSpeedRadPerSec;
  private final TargetRangeSensor rangeSensor = RangeSensors.get();

  public AssistTurret(Turret turret, DoubleSupplier adjustSupplier, BooleanSupplier fireSupplier,
      BooleanSupplier selectHigh, BooleanSupplier selectMid, BooleanSupplier selectLow) {
    this.turret = turret;
    this.adjustSupplier = adjustSupplier;
    this.fireSupplier = fireSupplier;
    this.selectHigh = selectHigh;
    this.selectMid = selectMid;
    this.selectLow = selectLow;
    addRequirements(turret);
  }

  public Row getRow() {
    return row;
  }

  @Override
  public void initialize() {
    triggerTimer.stop();
    triggerTimer.reset();
    // 進入時左搖桿已經推著 → 不算擊發，要放開再推
    lastFire = fireSupplier.getAsBoolean();
    fireCount = 0;
  }

  @Override
  public void execute() {
    boolean selecting = true;
    if (selectHigh.getAsBoolean()) row = Row.HIGH;
    else if (selectMid.getAsBoolean()) row = Row.MID;
    else if (selectLow.getAsBoolean()) row = Row.LOW;
    else selecting = false;

    double adjust = selecting ? 0 : Util.mapAxis(adjustSupplier.getAsDouble(), -kAdjustRad, kAdjustRad);
    double distance = rangeSensor.getTargetDistance();
    double pitch = Util.clamp(ShotTable.pitchRad(row.heightMeters, distance) + adjust,
        TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
    turret.setAimHeight(row.heightMeters);
    turret.setVelocity(targetVelocity, targetVelocity * 0.8);
    turret.setPitch(pitch);

    boolean fire = fireSupplier.getAsBoolean();
    if (fire && !lastFire) {
      triggerTimer.restart();
      fireCount++;
      // 診斷「射 3 發就停」：每次擊發記下當時的飛輪轉速、電池電壓
      Logger.recordOutput("Assist/FireUpRadPerSec", turret.getUpVelocityRadPerSec());
      Logger.recordOutput("Assist/FireBatteryVolts", org.wpilib.system.RobotController.getBatteryVoltage());
    }
    Logger.recordOutput("Assist/FireCount", fireCount);
    Logger.recordOutput("Assist/FireInput", fire);
    lastFire = fire;
    if (triggerTimer.isRunning()) {
      turret.setTrigger(triggerTimer.get() < kTriggerSeconds ? 12 : 0);
    }

    Logger.recordOutput("Assist/Row", row.name());
    Logger.recordOutput("Assist/AdjustRad", adjust);
    Logger.recordOutput("Assist/PitchRad", pitch);
    Logger.recordOutput("Assist/AtSpeed", turret.isNearTargetSpeeds(targetVelocity, targetVelocity * 0.8));
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    turret.setTrigger(0);
    turret.setAimHeight(Double.NaN);
    triggerTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
