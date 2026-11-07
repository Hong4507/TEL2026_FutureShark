package frc.FSLib.rangefinder;

import org.wpilib.units.measure.Angle;

/**
 * 量測「到目標的距離」與「相對牆面的偏航角」的感測器抽象層。
 * 實作：{@link RPLidarRangeSensor}（RPLidar A1）、{@link DualTFLunaRangeSensor}（兩顆 TF-Luna，I2C）。
 */
public interface TargetRangeSensor {
  /**
   * @return 到目標的距離（公尺，已含牆面到目標的 offset）。資料無效時回傳 fallback 值。
   */
  double getTargetDistance();

  /**
   * @return 偏航角，π/2 代表正對牆面；無法計算時回傳 {@code null}。
   */
  Angle getYaw();
}
