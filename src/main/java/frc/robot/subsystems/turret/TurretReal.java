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

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;
import frc.FSLib.util.SparkUtil;
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
  /** 絕對位置（單圈），砲台最低點 = 0 圈（由 kEncoderMagnetOffsetRot 校正）。 */
  private final StatusSignal<Angle> pitch = encoder.getAbsolutePosition();

  private final MotionMagicVelocityVoltage shooterMotorRequest = new MotionMagicVelocityVoltage(0);

  private final PIDController pitchPID = new PIDController(20, 8, 0.2);

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

    // 用 CANcoder 絕對位置決定仰角，開機時砲台停在哪裡都可以（不再把開機位置當成最低點）
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor
      .withMagnetOffset(kEncoderMagnetOffsetRot)
      .withAbsoluteSensorDiscontinuityPoint(kEncoderDiscontinuityPointRot);
    encoder.getConfigurator().apply(encoderConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(100, upVelocity, upVoltage, downVelocity, downVoltage, pitch);
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

    SparkMaxConfig triggerMotorConfig = new SparkMaxConfig();
    triggerMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    triggerMotorConfig.closedLoop.maxMotion
      .cruiseVelocity(1)
      .maxAcceleration(5)
      .allowedProfileError(0.01);
    triggerMotorConfig.signals
      .apply(SparkUtil.optimizedAllSignalConfig());
    triggerMotor.configure(triggerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(pitch, upVelocity, upVoltage, downVelocity, downVoltage);
    inputs.upConnected = BaseStatusSignal.isAllGood(upVelocity, upVoltage);
    inputs.downConnected = BaseStatusSignal.isAllGood(downVelocity, downVoltage);
    inputs.encoderConnected = BaseStatusSignal.isAllGood(pitch);
    inputs.pitchPositionRad = getPitchRad();
    inputs.pitchEncoderAbsRot = pitch.getValueAsDouble();
    inputs.upVelocityRadPerSec = upVelocity.getValue().in(RadiansPerSecond);
    inputs.upAppliedVolts = upVoltage.getValueAsDouble();
    inputs.downVelocityRadPerSec = downVelocity.getValue().in(RadiansPerSecond);
    inputs.downAppliedVolts = downVoltage.getValueAsDouble();
    inputs.triggerVolts = triggerMotor.getAppliedOutput().get() * triggerMotor.getBusVoltage().get();
    inputs.triggerCurrentAmps = triggerMotor.getOutputCurrent().get();
  }

  @Override
  public void setVelocity(double upVelocityRadPerSec, double downVelocityRadPerSec) {
    upShooterMotor.setControl(shooterMotorRequest.withVelocity(Units.radiansToRotations(upVelocityRadPerSec)).withEnableFOC(true));
    downShooterMotor.setControl(shooterMotorRequest.withVelocity(Units.radiansToRotations(downVelocityRadPerSec)).withEnableFOC(true));
  }

  @Override
  public void setPitch(double pitchPositionRad) {
    angleMotor.setVoltage(pitchPID.calculate(getPitchRad(), pitchPositionRad));
  }

  /** 絕對位置 0 圈 = 最低角度，往上每 kSensorToMechanismRatio 圈 = 砲台轉一圈。 */
  private double getPitchRad() {
    return kMinAngleRad + pitch.getValue().in(Radians) / kSensorToMechanismRatio;
  }

  @Override
  public void setTrigger(double volts) {
    triggerMotor.setVoltage(volts);
  }
}
