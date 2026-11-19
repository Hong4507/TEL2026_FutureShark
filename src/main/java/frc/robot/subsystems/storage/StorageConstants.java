package frc.robot.subsystems.storage;

// SystemCore Smart I/O 0~5，每顆馬達兩支腳：forward、reverse。
// 接 L298N：接 IN 腳（EN jumper 插著 = 常開）。
// 接繼電器（11/19）：每顆馬達兩個 SPDT 繼電器，forward 腳控制 A、reverse 腳控制 B（接法見 Stroage.java）。
public class StorageConstants {
  /**
   * 控制訊號是不是「低電位觸發」。L298N 和高電位觸發的繼電器模組 = false。
   * 很多便宜的 5V 繼電器模組是**低電位觸發**（IN 接 GND 才吸合，板上通常有 H/L jumper 可切）→ 設 true。
   * 判斷方法：程式停用時（所有腳都輸出「關」）繼電器如果是吸合的（燈亮、喀一聲），就改這個。
   */
  public static final boolean kOutputActiveLow = false;

  public static final int kTriggerReversePort = 0;      // Smart I/O 0（11/15 實測：Trigger 方向要用這組）
  public static final int kTriggerForwardPort = 1;      // Smart I/O 1
  public static final int kStorageLeftReversePort = 3;  // Smart I/O 3
  public static final int kStorageLeftForwardPort = 2;  // Smart I/O 2
  public static final int kStorageRightReversePort = 5; // Smart I/O 5
  public static final int kStorageRightForwardPort = 4; // Smart I/O 4
}
