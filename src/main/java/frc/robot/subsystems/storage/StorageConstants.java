package frc.robot.subsystems.storage;

// SystemCore Smart I/O 0~5，全部接 L298N 的 IN 腳（EN 腳 jumper 保持插上 = 常開）
public class StorageConstants {
  public static final int kTriggerReversePort = 0;      // Smart I/O 0（11/15 實測：Trigger 方向要用這組）
  public static final int kTriggerForwardPort = 1;      // Smart I/O 1
  public static final int kStorageLeftReversePort = 3;  // Smart I/O 3
  public static final int kStorageLeftForwardPort = 2;  // Smart I/O 2
  public static final int kStorageRightReversePort = 5; // Smart I/O 5
  public static final int kStorageRightForwardPort = 4; // Smart I/O 4

  /** 射擊模式（右開關開）偵測到射出後：先 L298N Trigger 轉這麼久、Storage 不動（秒）。 */
  public static final double kShotFeedTriggerSec = 2;
  /** 接著 Trigger 停、Storage 轉這麼久（秒）。 */
  public static final double kShotFeedStorageSec = 0.7;
}
