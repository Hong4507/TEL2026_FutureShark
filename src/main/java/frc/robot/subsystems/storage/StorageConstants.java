package frc.robot.subsystems.storage;

// SystemCore Smart I/O 0~5，全部接 L298N 的 IN 腳（EN 腳 jumper 保持插上 = 常開）
public class StorageConstants {
  public static final int kTriggerReversePort = 0;      // Smart I/O 0
  public static final int kTriggerForwardPort = 1;      // Smart I/O 1
  public static final int kStorageLeftReversePort = 2;  // Smart I/O 2
  public static final int kStorageLeftForwardPort = 3;  // Smart I/O 3
  public static final int kStorageRightReversePort = 4; // Smart I/O 4
  public static final int kStorageRightForwardPort = 5; // Smart I/O 5
}
