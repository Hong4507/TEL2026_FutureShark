package frc.FSLib.ros.laser.RPLidarA1;

import static org.wpilib.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.units.measure.Angle;
import org.littletonrobotics.junction.Logger;
import frc.FSLib.maths.LineFitter;
import frc.FSLib.maths.LineFitter.Line;
import frc.FSLib.maths.LineFitter.Point2D;

public class LaserScanMath {
  private static double minValidRange = 0.15; // meters
  private static double maxValidRange = 5.0; // meters
  private static int minPointsRequired = 10; // minimum points needed for valid calculation
  private static double maxLineError = 0.01; // maximum allowed fit error (meters)

  public static double calculateTargetDistance(LaserScan scan) {
    Line bestLine = calculateBestLine(scan);
    double distance;
    if (bestLine != null) {
      double theta = Math.atan(bestLine.slope);
      distance = -(-bestLine.intercept / bestLine.slope) + 1 / Math.cos(-theta + Math.copySign(Math.PI / 2, theta)) * 2.5 + 0.04;
    } else if (scan.getRangesStart().length > 0) {
      distance = scan.getRangesStart()[0] + 2.54;
    } else {
      distance = 3;
    }
    Logger.recordOutput("distance", distance);
    return distance;
  }

  /**
   * Calculates the robot's yaw angle from a LaserScan message
   * 
   * @param scan The LaserScan message
   * @return Yaw angle in radians, or null if calculation fails
   */
  public static Angle calculateYaw(LaserScan scan) {
    Line line = calculateBestLine(scan);

    if (line == null) {
      return null;
    }

    // Calculate yaw from the line slope
    // atan(slope) gives the angle of the line relative to x-axis
    double theta = Math.atan(line.slope);

    // coordinate convertion
    theta = -theta + Math.copySign(Math.PI / 2, theta);

    // offset
    theta += Math.PI / 2;

    return Radians.of(theta);
  }

  public static Line calculateBestLine(LaserScan scan) {
    // Convert polar scan data to Cartesian points
    List<Point2D> points = convertScanToPoints(scan);    

    // Check if we have enough valid points
    if (points.size() < minPointsRequired) {
      // System.err.println("Insufficient valid points: " + points.size());
      return null;
    }

    // Fit a line to the points using least squares
    LineFitter.IncrementalFitter fitter = new LineFitter.IncrementalFitter();
    fitter.reset();
    for (Point2D point : points) {
      fitter.addPoint(point);
    }
    Line line = fitter.fit();

    // Check if the line fit is good enough
    if (line.error > maxLineError) {
      // System.err.println("Line fit error too large: " + line.error);
      return null;
    }

    Logger.recordOutput("error", line.error);

    return line;
  }

  /**
   * Converts LaserScan polar coordinates to Cartesian points
   */
  private static List<Point2D> convertScanToPoints(LaserScan scan) {
    List<Point2D> points = new ArrayList<>();

    for (int i = 0; i < scan.getRangesEnd().length; i++) {
      double range = scan.getRangesEnd()[i];

      // Filter out invalid ranges
      if (Double.isNaN(range) || Double.isInfinite(range) ||
          range < minValidRange || range > maxValidRange ||
          range < scan.getRangeMin() || range > scan.getRangeMax()) {
        continue;
      }

      // Calculate angle for this measurement
      double angle = scan.getAngleMax() - ((32 - i) * scan.getAngleIncrement());

      // Convert polar to Cartesian (lidar frame)
      double x = range * Math.cos(angle);
      double y = range * Math.sin(angle);

      points.add(new Point2D(x, y));
    }

    for (int i = 0; i < scan.getRangesStart().length; i++) {
      double range = scan.getRangesStart()[i];

      // Filter out invalid ranges
      if (Double.isNaN(range) || Double.isInfinite(range) ||
          range < minValidRange || range > maxValidRange ||
          range < scan.getRangeMin() || range > scan.getRangeMax()) {
        continue;
      }

      // Calculate angle for this measurement
      double angle = scan.getAngleMin() + (i * scan.getAngleIncrement());

      // Convert polar to Cartesian (lidar frame)
      double x = range * Math.cos(angle);
      double y = range * Math.sin(angle);

      points.add(new Point2D(x, y));
    }

    return points;
  }

  // Setter methods for tunable parameters

  public static void setMinValidRange(double minRange) {
    minValidRange = minRange;
  }

  public static void setMaxValidRange(double maxRange) {
    maxValidRange = maxRange;
  }

  public static void setMinPointsRequired(int minPoints) {
    minPointsRequired = minPoints;
  }

  public static void setMaxLineError(double maxError) {
    maxLineError = maxError;
  }
}
