package frc.robot.subsystems.drive;

import static frc.FSLib.util.SparkUtil.assertOk;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.wpilib.math.controller.SimpleMotorFeedforward;
import frc.FSLib.util.SparkUtil;
import frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class DriveIOSpark implements DriveIO {
  private final SparkMax frontLeftMotor = new SparkMax(Constants.kCANPort, kFrontLeftMotorID, MotorType.kBrushless);
  private final SparkMax backLeftMotor = new SparkMax(Constants.kCANPort, kBackLeftMotorID, MotorType.kBrushless);
  private final SparkMax frontRightMotor = new SparkMax(Constants.kCANPort, kFrontRightMotorID, MotorType.kBrushless);
  private final SparkMax backRightMotor = new SparkMax(Constants.kCANPort, kBackRightMotorID, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = frontRightMotor.getEncoder();

  private final SparkClosedLoopController leftController = frontLeftMotor.getClosedLoopController();
  private final SparkClosedLoopController rightController = frontRightMotor.getClosedLoopController();

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  // REVLib 2027 移除了 positionConversionFactor / velocityConversionFactor，
  // 編碼器固定回傳馬達的「圈」與「RPM」，所以單位換算改在程式中處理。
  // PID 增益也乘上 kVelocityConversionFactor，讓誤差單位由 m/s 換成 RPM 後行為不變。
  public DriveIOSpark() {
    SparkMaxConfig frontLeftMotorConfig = new SparkMaxConfig();
    frontLeftMotorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true) 
      .smartCurrentLimit(30);
    frontLeftMotorConfig.signals.apply(SparkUtil.optimizedSignalConfig())
      .appliedOutputPeriodMs(5)
      .primaryEncoderPositionAlwaysOn(true)
      .primaryEncoderPositionPeriodMs(5)
      .primaryEncoderVelocityAlwaysOn(true)
      .primaryEncoderVelocityPeriodMs(5);
    frontLeftMotorConfig.closedLoop
      .pid(kP * kVelocityConversionFactor, kI * kVelocityConversionFactor, kD * kVelocityConversionFactor);
    assertOk(frontLeftMotor, () -> frontLeftMotor.configure(frontLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig backLeftMotorConfig = new SparkMaxConfig();
    backLeftMotorConfig
      .idleMode(IdleMode.kBrake)
      .follow(frontLeftMotor, false)
      .smartCurrentLimit(30);
    backLeftMotorConfig.signals.apply(SparkUtil.optimizedAllSignalConfig());
    assertOk(backLeftMotor, () -> backLeftMotor.configure(backLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig frontRightMotorConfig = new SparkMaxConfig();
    frontRightMotorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(30);
    frontRightMotorConfig.signals.apply(SparkUtil.optimizedSignalConfig())
      .appliedOutputPeriodMs(5)
      .primaryEncoderPositionAlwaysOn(true)
      .primaryEncoderPositionPeriodMs(5)
      .primaryEncoderVelocityAlwaysOn(true)
      .primaryEncoderVelocityPeriodMs(5);
    frontRightMotorConfig.closedLoop
      .pid(kP * kVelocityConversionFactor, kI * kVelocityConversionFactor, kD * kVelocityConversionFactor);
    assertOk(frontRightMotor, () -> frontRightMotor.configure(frontRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig backRightMotorConfig = new SparkMaxConfig();
    backRightMotorConfig
      .idleMode(IdleMode.kBrake)
      .follow(frontRightMotor, false)
      .smartCurrentLimit(30);
    backRightMotorConfig.signals.apply(SparkUtil.optimizedAllSignalConfig());
    assertOk(backRightMotor, () -> backRightMotor.configure(backRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionMeters = leftEncoder.getPosition().get() * kPositionConversionFactor;
    inputs.leftVelocityMetersPerSec = leftEncoder.getVelocity().get() * kVelocityConversionFactor;
    inputs.leftAppliedVolts = frontLeftMotor.getAppliedOutput().get() * frontLeftMotor.getBusVoltage().get();
    inputs.leftCurrentAmps = frontLeftMotor.getOutputCurrent().get();

    inputs.rightPositionMeters = rightEncoder.getPosition().get() * kPositionConversionFactor;
    inputs.rightVelocityMetersPerSec = rightEncoder.getVelocity().get() * kVelocityConversionFactor;
    inputs.rightAppliedVolts = frontRightMotor.getAppliedOutput().get() * frontRightMotor.getBusVoltage().get();
    inputs.rightCurrentAmps = frontRightMotor.getOutputCurrent().get();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    double leftFFVolts = feedforward.calculate(leftMetersPerSec);
    double rightFFVolts = feedforward.calculate(rightMetersPerSec);
    // 速度設定點單位為馬達 RPM
    leftController.setSetpoint(leftMetersPerSec / kVelocityConversionFactor, ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFFVolts);
    rightController.setSetpoint(rightMetersPerSec / kVelocityConversionFactor, ControlType.kVelocity, ClosedLoopSlot.kSlot0, rightFFVolts);
  }
}
