package frc.robot.subsystems.turret;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.system.Timer;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;
import frc.FSLib.util.SparkUtil;
import frc.FSLib.util.Util;
import frc.robot.Constants;

public class TurretReal implements TurretIO {
  private final CANBus canBus = new CANBus(Constants.kCANPort);

  private final TalonFX upShooterMotor = new TalonFX(kUpShooterMotorID, canBus);
  private final TalonFX downShooterMotor = new TalonFX(kDownShooterMotorID, canBus);
  private final SparkMax angleMotor = new SparkMax(Constants.kCANPort, kAngleMotorID, MotorType.kBrushless);
  private final SparkMax triggerMotor = new SparkMax(Constants.kCANPort, kTriggerMotorID, MotorType.kBrushless);

  private final CANcoder encoder = new CANcoder(kEncoderID, canBus);

  private final StatusSignal<AngularVelocity> upVelocity = upShooterMotor.getVelocity();
  private final StatusSignal<Voltage> upVoltage = upShooterMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> downVelocity = downShooterMotor.getVelocity();
  private final StatusSignal<Voltage> downVoltage = downShooterMotor.getMotorVoltage();
  /** 相對位置（圈，CANcoder 端）：開機時設成最低角度，所以**開機時砲台必須靠在下限位**。 */
  private final StatusSignal<Angle> pitch = encoder.getPosition();
  /** 絕對位置只拿來記錄（Turret/PitchEncoderAbsRot），之後要改回絕對位置時校正用，不參與控制。 */
  private final StatusSignal<Angle> pitchAbs = encoder.getAbsolutePosition();

  private final MotionMagicVelocityVoltage shooterMotorRequest = new MotionMagicVelocityVoltage(0);

  /**
   * 仰角 PID + 梯形速度規劃：目標不會一下子跳過去，而是以最快 kPitchMaxVelRadPerSec 慢慢移過去，
   * PID 只要追「現在該在哪」，誤差一直很小，不會全力衝過頭。
   * 週期要和機器人 loop 一樣（Constants.kDefaultPeriod = 10 ms），不然 D／I 會算錯（之前用預設 20 ms，D 只剩一半）。
   */
  private final ProfiledPIDController pitchPID = new ProfiledPIDController(20, 8, 0.2,
      new TrapezoidProfile.Constraints(kPitchMaxVelRadPerSec, kPitchMaxAccelRadPerSecSq), Constants.kDefaultPeriod);
  /** 上一次 setPitch 的時間；隔太久（換指令、剛切模式）就重設 PID，避免舊的積分／微分一次衝出去。 */
  private double lastPitchCallSec = Double.NEGATIVE_INFINITY;

  // 暴衝保護：出力 > kRunawayVolts 持續 kRunawaySec，讀值卻往反方向跑（或跑出行程）→ 停馬達直到下次重新開始
  private static final double kRunawayVolts = 2.0;
  private static final double kRunawaySec = 0.3;
  private static final double kOutOfRangeRad = 0.05;
  private boolean pitchFault = false;
  private double lastPitchRad = Double.NaN;
  private double wrongWaySec = 0;

  public TurretReal() {
    TalonFXConfiguration upShooterConfig = new TalonFXConfiguration();
    upShooterConfig.MotorOutput
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
    upShooterConfig.MotionMagic
      .withMotionMagicAcceleration(200)
      .withMotionMagicJerk(1000);
    upShooterConfig.Slot0
      .withKS(0.60912)
      .withKV(0.098707)
      .withKA(0.0057739)
      .withKP(0.053142);
    upShooterMotor.getConfigurator().apply(upShooterConfig);
    upShooterMotor.setPosition(0);

    TalonFXConfiguration downShooterConfig = new TalonFXConfiguration();
    downShooterConfig.MotorOutput
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
    downShooterConfig.MotionMagic
      .withMotionMagicAcceleration(200)
      .withMotionMagicJerk(1000);
    downShooterConfig.Slot0
      .withKS(0.56211)
      .withKV(0.1019)
      .withKA(0.044766)
      .withKP(0.13141);
    downShooterMotor.getConfigurator().apply(downShooterConfig);
    downShooterMotor.setPosition(0);

    // 相對位置：開機位置 = 最低角度 kMinAngleRad（砲台開機前要放到最低點）。
    // 磁鐵偏移照舊套用，只影響記錄用的絕對位置；相對位置由下面的 setPosition 決定。
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor
      .withMagnetOffset(kEncoderMagnetOffsetRot)
      .withAbsoluteSensorDiscontinuityPoint(kEncoderDiscontinuityPointRot);
    encoder.getConfigurator().apply(encoderConfig);
    encoder.setPosition(0); // 下限位 = 0 圈 = kMinAngleRad

    BaseStatusSignal.setUpdateFrequencyForAll(100, upVelocity, upVoltage, downVelocity, downVoltage, pitch);
    pitchAbs.setUpdateFrequency(10);
    upShooterMotor.optimizeBusUtilization();
    downShooterMotor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();

    SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
    angleMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    angleMotorConfig.signals.apply(SparkUtil.optimizedAllSignalConfig());
    angleMotorConfig.smartCurrentLimit(20);
    angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    angleMotor.getEncoder().setPosition(0);

    // 積分只在接近目標（±0.05 rad ≈ 3°）時累積，而且最多貢獻 ±3 V，避免大誤差時積分越積越大衝過頭
    pitchPID.setIZone(0.05);
    pitchPID.setIntegratorRange(-3, 3);

    SparkMaxConfig triggerMotorConfig = new SparkMaxConfig();
    triggerMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    triggerMotorConfig.closedLoop.maxMotion
      .cruiseVelocity(1)
      .maxAcceleration(5)
      .allowedProfileError(0.01);
    // 射出偵測看擊發馬達轉速：轉速訊號不能用 optimizedAll（會變 500 ms 一筆，0.15 s 的擊發根本看不到）
    triggerMotorConfig.signals
      .apply(SparkUtil.optimizedSignalConfig())
      .primaryEncoderVelocityPeriodMs(20);
    triggerMotor.configure(triggerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(pitch, pitchAbs, upVelocity, upVoltage, downVelocity, downVoltage);
    // 沒有指令在控制仰角（超過 0.1 秒沒呼叫 setPitch）→ 停馬達，靠 brake 撐住；
    // 不然 SparkMax 會一直維持最後一次的電壓
    if (Timer.getTimestamp() - lastPitchCallSec > 0.1) {
      angleMotor.setVoltage(0);
    }
    inputs.upConnected = BaseStatusSignal.isAllGood(upVelocity, upVoltage);
    inputs.downConnected = BaseStatusSignal.isAllGood(downVelocity, downVoltage);
    inputs.encoderConnected = BaseStatusSignal.isAllGood(pitch);
    inputs.pitchPositionRad = getPitchRad();
    inputs.pitchEncoderAbsRot = pitchAbs.getValueAsDouble();
    inputs.upVelocityRadPerSec = upVelocity.getValue().in(RadiansPerSecond);
    inputs.upAppliedVolts = upVoltage.getValueAsDouble();
    inputs.downVelocityRadPerSec = downVelocity.getValue().in(RadiansPerSecond);
    inputs.downAppliedVolts = downVoltage.getValueAsDouble();
    inputs.triggerVolts = triggerMotor.getAppliedOutput().get() * triggerMotor.getBusVoltage().get();
    inputs.triggerCurrentAmps = triggerMotor.getOutputCurrent().get();
    inputs.triggerVelocityRpm = triggerMotor.getEncoder().getVelocity().get();
    // 仰角馬達自己的 encoder（開機 = 0，正 = 馬達正轉）；和 CANcoder 比正負就知道方向有沒有反
    Logger.recordOutput("Turret/PitchMotorRot", angleMotor.getEncoder().getPosition().get());
  }

  @Override
  public void setVelocity(double upVelocityRadPerSec, double downVelocityRadPerSec) {
    upShooterMotor.setControl(shooterMotorRequest.withVelocity(Units.radiansToRotations(upVelocityRadPerSec)).withEnableFOC(true));
    downShooterMotor.setControl(shooterMotorRequest.withVelocity(Units.radiansToRotations(downVelocityRadPerSec)).withEnableFOC(true));
  }

  @Override
  public void setPitch(double pitchPositionRad) {
    double now = Timer.getTimestamp();
    double dt = now - lastPitchCallSec;
    double current = getPitchRad();
    if (dt > 0.1) { // 新的一次開始（換指令、剛切模式）：從目前位置開始規劃，重設 PID 與暴衝保護
      pitchPID.reset(current);
      pitchFault = false;
      wrongWaySec = 0;
      lastPitchRad = current;
      dt = 0;
    }
    lastPitchCallSec = now;

    double target = Util.clamp(pitchPositionRad, kMinAngleRad, kMaxAngleRad);
    double volts = Util.clamp(pitchPID.calculate(current, target), -kPitchMaxVolts, kPitchMaxVolts);
    // 軟限位：已經到上限還往上、到下限還往下 → 不出力
    if ((current >= kMaxAngleRad && volts > 0) || (current <= kMinAngleRad && volts < 0)) {
      volts = 0;
    }
    checkRunaway(current, volts, dt);
    if (pitchFault) {
      volts = 0;
    }
    angleMotor.setVoltage(volts);
    Logger.recordOutput("Turret/PitchTargetRad", target);
    Logger.recordOutput("Turret/PitchSetpointRad", pitchPID.getSetpoint().position);
    Logger.recordOutput("Turret/PitchVolts", volts);
  }

  /** 用上一次的電壓方向判斷讀值有沒有往反方向跑；觸發後 pitchFault = true（記 log），直到下次重新開始。 */
  private double lastVolts = 0;

  private void checkRunaway(double current, double volts, double dt) {
    double moved = current - lastPitchRad;
    lastPitchRad = current;
    if (pitchFault) {
      return;
    }
    String reason = null;
    if (current > kMaxAngleRad + kOutOfRangeRad || current < kMinAngleRad - kOutOfRangeRad) {
      reason = String.format("讀值 %.3f rad 超出行程 [%.3f, %.3f]", current, kMinAngleRad, kMaxAngleRad);
    } else if (Math.abs(lastVolts) > kRunawayVolts && moved * Math.signum(lastVolts) < 0) {
      wrongWaySec += dt;
      if (wrongWaySec >= kRunawaySec) {
        reason = String.format("給 %+.1f V 但讀值往反方向跑 %.2f 秒：CANcoder 方向可能和馬達相反", lastVolts, wrongWaySec);
      }
    } else {
      wrongWaySec = 0;
    }
    lastVolts = volts;
    if (reason != null) {
      pitchFault = true;
      System.out.println("[Turret] 仰角暴衝保護觸發，停止仰角馬達：" + reason);
      Logger.recordOutput("Turret/PitchFaultReason", reason);
    }
    Logger.recordOutput("Turret/PitchFault", pitchFault);
  }

  /** 開機時 CANcoder = 0（下限位），每轉 kSensorToMechanismRatio 圈 = 砲台轉一圈。 */
  private double getPitchRad() {
    return kMinAngleRad + pitch.getValue().in(Radians) / kSensorToMechanismRatio;
  }

  @Override
  public void setTrigger(double volts) {
    triggerMotor.setVoltage(volts);
  }
}
