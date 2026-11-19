package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.DifferentialDriveKinematics;
import org.wpilib.math.kinematics.DifferentialDriveOdometry;
import org.wpilib.math.kinematics.DifferentialDriveWheelVelocities;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private final DriveIO driveIO;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(kTrackWidth);
  private final DifferentialDriveOdometry odometry = 
    new DifferentialDriveOdometry(Rotation2d.ZERO, 0.0, 0.0);

  private double gyroYawRad = 0.0;
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;

  public Drive (DriveIO driveIO, GyroIO gyroIO) {
    this.driveIO = driveIO;
    this.gyroIO = gyroIO;

    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException();
    }

    AutoBuilder.configure(
      this::getPose,
      this::setPose,
      this::getTwist,
      speeds -> {
        DifferentialDriveWheelVelocities wheelVelocities = kinematics.toWheelVelocities(speeds);
        driveIO.setVelocity(wheelVelocities.left, wheelVelocities.right);
      },
      new PPLTVController(Constants.kDefaultPeriod),
      robotConfig,
      () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
      this
    );

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
      path -> Logger.recordOutput("Drive/ActivatePath", path.toArray(new Pose2d[path.size()]))
    );
    PathPlannerLogging.setLogTargetPoseCallback(
      pose -> Logger.recordOutput("Drive/TargetPose", pose)
    );
  }

  @Override
  public void periodic() {
    driveIO.updateInputs(driveInputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", driveInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    if (gyroInputs.connected) {
      gyroYawRad = gyroInputs.yawRad;
    } else {
      Twist2d twist = kinematics.toTwist2d(
        driveInputs.leftPositionMeters - lastLeftPositionMeters,
        driveInputs.rightPositionMeters - lastRightPositionMeters
      );
      gyroYawRad = gyroYawRad + twist.dtheta;
      lastLeftPositionMeters = driveInputs.leftPositionMeters;
      lastRightPositionMeters = driveInputs.rightPositionMeters;
    }

    odometry.update(Rotation2d.fromRadians(gyroYawRad), driveInputs.leftPositionMeters, driveInputs.rightPositionMeters);

    // posePublisher.set(getPose());
    // twistPublisher.set(getTwist());
    // driveNTInst.flush();
  }

  @AutoLogOutput
  public boolean isGyroConnected() {
    return gyroInputs.connected;
  }

  /** 陀螺儀累積的偏航角（rad，逆時針為正）。不受 setPose 影響（PathPlanner 重設位置時不會跳），給測距融合當角度基準。 */
  public double getGyroYawRad() {
    return gyroYawRad;
  }

  /**
   * 11/19：Auto 開始時呼叫。陀螺儀偏航歸零；odometry 同步重設，所以 getPose() 的位置和朝向不會跳。
   * 之後 {@link #getGyroYawRad()} 從 0 開始算。
   */
  public void resetGyro() {
    Pose2d pose = getPose();
    gyroIO.resetYaw();
    gyroYawRad = 0.0;
    lastLeftPositionMeters = driveInputs.leftPositionMeters;
    lastRightPositionMeters = driveInputs.rightPositionMeters;
    odometry.resetPosition(Rotation2d.ZERO, driveInputs.leftPositionMeters, driveInputs.rightPositionMeters, pose);
    Logger.recordOutput("Drive/GyroResetCount", ++gyroResetCount);
  }

  private int gyroResetCount = 0;

  public Pose2d getPose() {
    return odometry.getPose();
  }

  public void setPose(Pose2d pose) {
    odometry.resetPose(pose);
  }

  @AutoLogOutput
  public ChassisVelocities getTwist() {
    return kinematics.toChassisVelocities(
      new DifferentialDriveWheelVelocities(driveInputs.leftVelocityMetersPerSec, driveInputs.rightVelocityMetersPerSec)
    );
  }

  public void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    driveIO.setVelocity(leftMetersPerSec, rightMetersPerSec);
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    driveIO.setVoltage(leftVolts, rightVolts);
  }
}
