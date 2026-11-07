package frc.FSLib.maths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Calculates turret angles (yaw and pitch) to hit a stationary target
 * while the robot chassis is moving, accounting for ballistic trajectory
 * and turret offset from robot center of rotation.
 */
public class TurretBallisticCalculator {

  private static final double DEFAULT_GRAVITY = 9.81; // m/s²
  private static final double DEFAULT_TOLERANCE = 0.01; // seconds
  private static final int DEFAULT_MAX_ITERATIONS = 10;

  /**
   * Result class containing calculated turret angles
   */
  public static class TurretAngles {
    public final Rotation2d yaw;
    public final Rotation2d pitch;
    public final boolean valid;
    public final String errorMessage;

    public TurretAngles(Rotation2d yaw, Rotation2d pitch) {
      this.yaw = yaw;
      this.pitch = pitch;
      this.valid = true;
      this.errorMessage = "";
    }

    public TurretAngles(String errorMessage) {
      this.yaw = new Rotation2d();
      this.pitch = new Rotation2d();
      this.valid = false;
      this.errorMessage = errorMessage;
    }

    @Override
    public String toString() {
      if (valid) {
        return String.format("Yaw: %.2f°, Pitch: %.2f°", yaw.getDegrees(), pitch.getDegrees());
      } else {
        return "Invalid: " + errorMessage;
      }
    }
  }

  /**
   * Calculates turret angles to hit a target while robot is moving,
   * accounting for turret offset from robot center.
   * 
   * @param targetRelativePosition Target position relative to robot center in
   *                               body frame (forward=x, left=y, up=z)
   * @param turretOffset           Turret position offset from robot center in
   *                               body frame (forward=x, left=y, up=z)
   * @param muzzleVelocity         Projectile speed in m/s
   * @return TurretAngles object containing yaw and pitch
   */
  public static TurretAngles calculateTurretAngles(
    Translation3d targetRelativePosition,
    Translation3d turretOffset,
    double muzzleVelocity
  ) {
    return calculateTurretAngles(
      targetRelativePosition,
      turretOffset,
      new ChassisVelocities(),
      muzzleVelocity,
      DEFAULT_GRAVITY,
      DEFAULT_TOLERANCE,
      DEFAULT_MAX_ITERATIONS
    );
  }

  /**
   * Calculates turret angles to hit a target while robot is moving,
   * accounting for turret offset from robot center.
   * 
   * @param targetRelativePosition Target position relative to robot center in
   *                               body frame (forward=x, left=y, up=z)
   * @param turretOffset           Turret position offset from robot center in
   *                               body frame (forward=x, left=y, up=z)
   * @param chassisSpeeds          Current chassis velocities (vx, vy, omega)
   * @param muzzleVelocity         Projectile speed in m/s
   * @return TurretAngles object containing yaw and pitch
   */
  public static TurretAngles calculateTurretAngles(
    Translation3d targetRelativePosition,
    Translation3d turretOffset,
    ChassisVelocities chassisSpeeds,
    double muzzleVelocity
  ) {
    return calculateTurretAngles(
      targetRelativePosition,
      turretOffset,
      chassisSpeeds,
      muzzleVelocity,
      DEFAULT_GRAVITY,
      DEFAULT_TOLERANCE,
      DEFAULT_MAX_ITERATIONS
    );
  }

  /**
   * Calculates turret angles to hit a target while robot is moving,
   * accounting for turret offset from robot center.
   * 
   * @param targetRelativePosition Target position relative to robot center in
   *                               body frame (forward=x, left=y, up=z)
   * @param turretOffset           Turret position offset from robot center in
   *                               body frame (forward=x, left=y, up=z)
   * @param chassisSpeeds          Current chassis velocities (vx, vy, omega)
   * @param muzzleVelocity         Projectile speed in m/s
   * @param gravity                Acceleration due to gravity in m/s²
   * @param tolerance              Convergence tolerance in seconds
   * @param maxIterations          Maximum number of iterations
   * @return TurretAngles object containing yaw and pitch
   */
  public static TurretAngles calculateTurretAngles(
    Translation3d targetRelativePosition,
    Translation3d turretOffset,
    ChassisVelocities chassisSpeeds,
    double muzzleVelocity,
    double gravity,
    double tolerance,
    int maxIterations
  ) {
    // Validate inputs
    if (muzzleVelocity <= 0) {
      return new TurretAngles("Muzzle velocity must be positive");
    }
    if (gravity <= 0) {
      return new TurretAngles("Gravity must be positive");
    }

    // Calculate target position relative to turret (not robot center)
    double dx = targetRelativePosition.getX() - turretOffset.getX();
    double dy = targetRelativePosition.getY() - turretOffset.getY();
    double dz = targetRelativePosition.getZ() - turretOffset.getZ();

    // Extract chassis velocities
    double vx = chassisSpeeds.vx;
    double vy = chassisSpeeds.vy;
    double vomega = chassisSpeeds.omega;

    // Initialize flight time guess (straight-line distance / speed)
    double straightLineDistance = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (straightLineDistance < 1e-6) {
      return new TurretAngles("Target is at turret position");
    }

    double t = straightLineDistance / muzzleVelocity;

    // Iterative solution
    double theta = 0;
    double phi = 0;

    for (int i = 0; i < maxIterations; i++) {
      // Calculate turret velocity due to robot rotation
      // v_turret = v_chassis + omega * r_turret_offset
      // In 2D: vx_turret = vx - omega * turret_y
      // vy_turret = vy + omega * turret_x
      double vxTurret = vx - vomega * turretOffset.getY();
      double vyTurret = vy + vomega * turretOffset.getX();

      // Calculate apparent target velocity relative to turret position
      // The target appears to move opposite to turret motion
      double vxApparent = -vxTurret + vomega * dy;
      double vyApparent = -vyTurret - vomega * dx;
      double vzApparent = 0; // Assume target height doesn't change

      // Compute adjusted target position (where target "appears" to be)
      double dxAdj = dx + vxApparent * t;
      double dyAdj = dy + vyApparent * t;
      double dzAdj = dz + vzApparent * t;

      // Effective horizontal range
      double r = Math.sqrt(dxAdj * dxAdj + dyAdj * dyAdj);

      // Handle special case: target directly above/below turret
      if (r < 1e-6) {
        theta = 0;
        // Vertical shot
        if (dzAdj > 0) {
          phi = Math.PI / 2;
        } else {
          phi = -Math.PI / 2;
        }
        break;
      }

      // Solve quadratic equation for tan(phi)
      // Ballistic trajectory equation: dz = r*tan(phi) - (g*r²)/(2*v²*cos²(phi))
      // Rearranged: a*tan²(phi) + b*tan(phi) + c = 0
      double v2 = muzzleVelocity * muzzleVelocity;
      double a = gravity * r * r / (2 * v2);
      double b = -r;
      double c = dzAdj + gravity * r * r / (2 * v2);

      // Calculate discriminant
      double discriminant = b * b - 4 * a * c;

      if (discriminant < 0) {
        return new TurretAngles("Target out of range (discriminant < 0)");
      }

      // Choose lower arc (negative sign) for flatter trajectory
      // Use positive sign for higher arc if needed
      double tanPhi = (-b - Math.sqrt(discriminant)) / (2 * a);
      phi = Math.atan(tanPhi);

      // Calculate yaw angle
      theta = Math.atan2(dyAdj, dxAdj);

      // Update flight time
      double cosPhi = Math.cos(phi);
      if (Math.abs(cosPhi) < 1e-6) {
        return new TurretAngles("Invalid pitch angle (near vertical)");
      }

      double tNew = r / (muzzleVelocity * cosPhi);

      // Check convergence
      if (Math.abs(tNew - t) < tolerance) {
        break;
      }

      t = tNew;

      // Check if we've reached max iterations
      if (i == maxIterations - 1) {
        // Still use the calculated values but could add warning
        System.out.println("Warning: Max iterations reached, using last calculated values");
      }
    }

    return new TurretAngles(new Rotation2d(theta), new Rotation2d(phi));
  }

  /**
   * Convenience method that takes target in field coordinates and robot pose
   * 
   * @param targetFieldPosition Target position in field coordinates
   * @param robotPose           Current robot pose (position and rotation)
   * @param turretOffset        Turret position offset from robot center in body
   *                            frame
   * @param muzzleVelocity      Projectile speed in m/s
   * @return TurretAngles object containing yaw and pitch
   */
  public static TurretAngles calculateTurretAnglesFromFieldCoordinates(
    Translation3d targetFieldPosition,
    Pose2d robotPose,
    Translation3d turretOffset,
    double muzzleVelocity
  ) {
    return calculateTurretAnglesFromFieldCoordinates(
      targetFieldPosition,
      robotPose,
      turretOffset,
      new ChassisVelocities(),
      muzzleVelocity
    );
  }

  /**
   * Convenience method that takes target in field coordinates and robot pose
   * 
   * @param targetFieldPosition Target position in field coordinates
   * @param robotPose           Current robot pose (position and rotation)
   * @param turretOffset        Turret position offset from robot center in body
   *                            frame
   * @param chassisSpeeds       Current chassis velocities
   * @param muzzleVelocity      Projectile speed in m/s
   * @return TurretAngles object containing yaw and pitch
   */
  public static TurretAngles calculateTurretAnglesFromFieldCoordinates(
    Translation3d targetFieldPosition,
    Pose2d robotPose,
    Translation3d turretOffset,
    ChassisVelocities chassisSpeeds,
    double muzzleVelocity
  ) {
    // Convert target from field frame to robot body frame
    Translation2d robotPosition = robotPose.getTranslation();
    Rotation2d robotRotation = robotPose.getRotation();

    // Calculate relative position in field frame
    double fieldDx = targetFieldPosition.getX() - robotPosition.getX();
    double fieldDy = targetFieldPosition.getY() - robotPosition.getY();
    double fieldDz = targetFieldPosition.getZ();

    // Rotate to body frame (relative to robot center)
    double bodyDx = fieldDx * robotRotation.getCos() + fieldDy * robotRotation.getSin();
    double bodyDy = -fieldDx * robotRotation.getSin() + fieldDy * robotRotation.getCos();
    double bodyDz = fieldDz;

    Translation3d targetBodyPosition = new Translation3d(bodyDx, bodyDy, bodyDz);

    return calculateTurretAngles(targetBodyPosition, turretOffset, chassisSpeeds, muzzleVelocity);
  }
}