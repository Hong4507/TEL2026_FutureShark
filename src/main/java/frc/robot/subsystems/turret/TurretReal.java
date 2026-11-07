package frc.robot.subsystems.turret;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
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
  private final StatusSignal<Angle> pitch = encoder.getPosition();

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

    encoder.setPosition(TurretConstants.kMinAngleRad * kSensorToMechanismRatio / 2 / Math.PI);

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
    inputs.pitchPositionRad = pitch.getValue().in(Radians) / kSensorToMechanismRatio;
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
    angleMotor.setVoltage(pitchPID.calculate(pitch.getValue().in(Radians) / kSensorToMechanismRatio, pitchPositionRad));
  }

  @Override
  public void setTrigger(double volts) {
    triggerMotor.setVoltage(volts);
  }
}
