package frc.robot.subsystems.storage;

// SystemCore Smart I/O 0~5（原 roboRIO：DIO 0~3、PWM 8、9）
public class StorageConstants {
  public static final int kStorageForwardPort = 1; // Smart I/O 1（原 DIO 1）
  public static final int kStorageReversePort = 0; // Smart I/O 0（原 DIO 0）
  public static final int kTriggerForwardPort = 3; // Smart I/O 3（原 DIO 3）
  public static final int kTriggerReversePort = 2; // Smart I/O 2（原 DIO 2）

  public static final int kStorageSpeedPort = 4;   // Smart I/O 4（原 PWM 8）
  public static final int kTriggerSpeedPort = 5;   // Smart I/O 5（原 PWM 9）
}
