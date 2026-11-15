package frc.robot.commands.turret;

import static frc.robot.subsystems.turret.TurretConstants.kMaxSpeedRadPerSec;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.system.Timer;

import frc.FSLib.util.Util;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

/**
 * 真手動：仰角、飛輪轉速固定，只用按鍵控制 Trigger（左搖桿 X 往右推 = 擊發一次）。
 * 要改仰角／轉速就改下面三個常數。
 */
public class RealManualTurret extends Command {
  /** 固定仰角（rad），會夾在 kMinAngleRad ~ kMaxAngleRad 之間。 */
  private static final double kPitchRad = 1.2;
  /** 固定轉速（rad/s）：上輪、下輪。 */
  private static final double kUpSpeedRadPerSec = kMaxSpeedRadPerSec;
  private static final double kDownSpeedRadPerSec = kMaxSpeedRadPerSec * 0.8;
  /** 每按一次 Trigger 通電幾秒。 */
  private static final double kTriggerSeconds = 0.5;

  private final Turret turret;
  private final BooleanSupplier triggerSupplier;
  private final double pitchRad = Util.clamp(kPitchRad, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);

  private boolean lastValue = false;
  private final Timer timer = new Timer();

  public RealManualTurret(Turret turret, BooleanSupplier triggerSupplier) {
    this.turret = turret;
    this.triggerSupplier = triggerSupplier;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    // 開關打開時如果已經推著擊發，不要立刻射；要放開再推一次
    lastValue = triggerSupplier.getAsBoolean();
    timer.stop();
    timer.reset();
  }

  @Override
  public void execute() {
    turret.setPitch(pitchRad);
    turret.setVelocity(kUpSpeedRadPerSec, kDownSpeedRadPerSec);
    Logger.recordOutput("RealManualTurret/PitchRad", pitchRad);

    boolean pressed = triggerSupplier.getAsBoolean();
    if (pressed && !lastValue) {
      timer.restart();
    }
    lastValue = pressed;

    boolean firing = timer.isRunning() && timer.get() < kTriggerSeconds;
    turret.setTrigger(firing ? 12 : 0);
    if (timer.isRunning() && !firing) {
      timer.stop();
    }
    Logger.recordOutput("RealManualTurret/Firing", firing);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    turret.setTrigger(0);
  }
}
