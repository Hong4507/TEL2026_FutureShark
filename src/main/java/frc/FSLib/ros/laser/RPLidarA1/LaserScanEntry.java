package frc.FSLib.ros.laser.RPLidarA1;

import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

public class LaserScanEntry {

  private final NetworkTable m_laserScanTable;
  private final DoubleSubscriber angleMin;
  private final DoubleSubscriber angleMax;
  private final DoubleSubscriber angleIncrement;
  private final DoubleSubscriber timeIncrement;
  private final DoubleSubscriber scanTime;
  private final DoubleSubscriber rangeMin;
  private final DoubleSubscriber rangeMax;
  private final DoubleArraySubscriber ranges_start;
  private final DoubleArraySubscriber ranges_end;
  private final DoubleArraySubscriber intensities;
  private final DoubleSubscriber length;
  
  public LaserScanEntry(String tableName) {
    m_laserScanTable = NetworkTableInstance.getDefault().getTable(tableName);
    angleMin = m_laserScanTable.getDoubleTopic("angle_min").subscribe(-Math.PI);
    angleMax = m_laserScanTable.getDoubleTopic("angle_max").subscribe(Math.PI);
    angleIncrement= m_laserScanTable.getDoubleTopic("angle_increment").subscribe(0.005);
    timeIncrement = m_laserScanTable.getDoubleTopic("time_increment").subscribe(0.00012);
    scanTime = m_laserScanTable.getDoubleTopic("scan_time").subscribe(0.135);
    rangeMin = m_laserScanTable.getDoubleTopic("range_min").subscribe(0.15);
    rangeMax = m_laserScanTable.getDoubleTopic("range_max").subscribe(12.0);
    ranges_start = m_laserScanTable.getDoubleArrayTopic("ranges_start").subscribe(new double[] {});
    ranges_end = m_laserScanTable.getDoubleArrayTopic("ranges_end").subscribe(new double[] {});
    intensities = m_laserScanTable.getDoubleArrayTopic("intensities").subscribe(new double[] {});
    length = m_laserScanTable.getDoubleTopic("length").subscribe(0.0);
  }

  public LaserScan get() {
    return new LaserScan(
      angleMin.get(),
      angleMax.get(),
      angleIncrement.get(),
      timeIncrement.get(),
      scanTime.get(),
      rangeMin.get(),
      rangeMax.get(),
      ranges_start.get(),
      ranges_end.get(),
      intensities.get(),
      length.get()
    );
  }
}