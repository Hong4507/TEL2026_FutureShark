package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    /** Jetson 在 {@link VisionConstants#kTimeoutSeconds} 內有送新畫面。 */
    public boolean connected = false;
    /** 這個週期是否收到新的一張畫面結果。 */
    public boolean newFrame = false;
    public long heartbeat = 0;
    /** 原始偵測資料，格式見 {@link VisionConstants}。 */
    public double[] targets = new double[0];
    /** Jetson 端：拿到畫面到發布（不含曝光、USB、網路）。 */
    public double latencyMs = 0.0;
    /**
     * 機器人時鐘：距離上次收到新結果幾毫秒（資料新鮮度）。不含網路傳輸本身（直連 &lt; 1 ms），
     * 完整年齡 ≈ latencyMs + ageMs。沒收過 = NaN。
     */
    public double ageMs = Double.NaN;
    public double fps = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
