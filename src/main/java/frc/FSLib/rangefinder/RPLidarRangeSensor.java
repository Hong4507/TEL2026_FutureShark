package frc.FSLib.rangefinder;

import org.wpilib.units.measure.Angle;
import frc.FSLib.ros.laser.RPLidarA1.LaserScanEntry;
import frc.FSLib.ros.laser.RPLidarA1.LaserScanMath;

/** 原本的 RPLidar A1 實作：讀 NT 表 "LaserScan"，用 {@link LaserScanMath} 直線擬合。 */
public class RPLidarRangeSensor implements TargetRangeSensor {
  private final LaserScanEntry entry;

  public RPLidarRangeSensor(String tableName) {
    entry = new LaserScanEntry(tableName);
  }

  @Override
  public double getTargetDistance() {
    return LaserScanMath.calculateTargetDistance(entry.get());
  }

  @Override
  public Angle getYaw() {
    return LaserScanMath.calculateYaw(entry.get());
  }
}
