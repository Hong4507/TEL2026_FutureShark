package frc.FSLib.ros.laser.RPLidarA1;

import org.wpilib.util.protobuf.ProtobufSerializable;
import org.wpilib.util.struct.StructSerializable;
import java.util.Arrays;

/**
 * Represents a ROS LaserScan message.
 *
 * <p>
 * This class encapsulates laser scan data including angular parameters,
 * timing information, range limits, and the actual range/intensity
 * measurements.
 */
public class LaserScan implements ProtobufSerializable, StructSerializable {

  /** start angle of the scan [rad] */
  private final double m_angleMin;

  /** end angle of the scan [rad] */
  private final double m_angleMax;

  /** angular distance between measurements [rad] */
  private final double m_angleIncrement;

  /** time between measurements [seconds] */
  private final double m_timeIncrement;

  /** time between scans [seconds] */
  private final double m_scanTime;

  /** minimum range value [m] */
  private final double m_rangeMin;

  /** maximum range value [m] */
  private final double m_rangeMax;

  /** range data [m] (values < range_min or > range_max should be discarded) */
  private final double[] m_rangesStart;
  private final double[] m_rangesEnd;

  /** intensity data [device-specific units] */
  private final double[] m_intensities;

  private final double m_length;

  /**
   * Constructs a LaserScan with all fields set to zero/empty.
   */
  public LaserScan() {
    this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, new double[0], new double[0], new double[0], 0.0);
  }

  /**
   * Constructs a LaserScan with specified parameters.
   *
   * @param angleMin       start angle of the scan [rad]
   * @param angleMax       end angle of the scan [rad]
   * @param angleIncrement angular distance between measurements [rad]
   * @param timeIncrement  time between measurements [seconds]
   * @param scanTime       time between scans [seconds]
   * @param rangeMin       minimum range value [m]
   * @param rangeMax       maximum range value [m]
   * @param rangesStart    range data [m]
   * @param rangesEnd      range data [m]
   * @param intensities    intensity data [device-specific units]
   * @param length         range data length
   */
  public LaserScan(
      double angleMin,
      double angleMax,
      double angleIncrement,
      double timeIncrement,
      double scanTime,
      double rangeMin,
      double rangeMax,
      double[] rangesStart,
      double[] rangesEnd,
      double[] intensities,
      double length) {
    m_angleMin = angleMin;
    m_angleMax = angleMax;
    m_angleIncrement = angleIncrement;
    m_timeIncrement = timeIncrement;
    m_scanTime = scanTime;
    m_rangeMin = rangeMin;
    m_rangeMax = rangeMax;
    m_rangesStart = Arrays.copyOf(rangesStart, rangesStart.length);
    m_rangesEnd = Arrays.copyOf(rangesEnd, rangesEnd.length);
    m_intensities = Arrays.copyOf(intensities, intensities.length);
    m_length = length;
  }

  /**
   * Returns the start angle of the scan [rad].
   *
   * @return The start angle of the scan.
   */
  public double getAngleMin() {
    return m_angleMin;
  }

  /**
   * Returns the end angle of the scan [rad].
   *
   * @return The end angle of the scan.
   */
  public double getAngleMax() {
    return m_angleMax;
  }

  /**
   * Returns the angular distance between measurements [rad].
   *
   * @return The angular distance between measurements.
   */
  public double getAngleIncrement() {
    return m_angleIncrement;
  }

  /**
   * Returns the time between measurements [seconds].
   *
   * @return The time between measurements.
   */
  public double getTimeIncrement() {
    return m_timeIncrement;
  }

  /**
   * Returns the time between scans [seconds].
   *
   * @return The time between scans.
   */
  public double getScanTime() {
    return m_scanTime;
  }

  /**
   * Returns the minimum range value [m].
   *
   * @return The minimum range value.
   */
  public double getRangeMin() {
    return m_rangeMin;
  }

  /**
   * Returns the maximum range value [m].
   *
   * @return The maximum range value.
   */
  public double getRangeMax() {
    return m_rangeMax;
  }

  /**
   * Returns a copy of the range data [m].
   * Values less than range_min or greater than range_max should be discarded.
   *
   * @return A copy of the range data.
   */
  public double[] getRangesStart() {
    return Arrays.copyOf(m_rangesStart, m_rangesStart.length);
  }

  /**
   * Returns a copy of the range data [m].
   * Values less than range_min or greater than range_max should be discarded.
   *
   * @return A copy of the range data.
   */
  public double[] getRangesEnd() {
    return Arrays.copyOf(m_rangesEnd, m_rangesEnd.length);
  }

  /**
   * Returns a copy of the intensity data [device-specific units].
   *
   * @return A copy of the intensity data.
   */
  public double[] getIntensities() {
    return Arrays.copyOf(m_intensities, m_intensities.length);
  }

  public double getLength() {
    return m_length;
  }

}