package frc.robot.commands.drive;

import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import frc.FSLib.maths.Maths;
import frc.FSLib.rangefinder.RangeSensors;
import frc.FSLib.rangefinder.TargetRangeSensor;
import frc.robot.subsystems.drive.Drive;

public class Align extends Command {
  private final Drive drive;

  private final Timer dontSeeTimer = new Timer();

  private double targetPixel;
  private double detectedPixel, lastDetectedPixel;

  private final TargetRangeSensor rangeSensor = RangeSensors.get();
  private final DoubleArraySubscriber poses = NetworkTableInstance.getDefault()
    .getTable("Vision").getDoubleArrayTopic("poses").subscribe(new double[] {});

  public Align(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    double distance = rangeSensor.getTargetDistance();
    targetPixel = -(Math.atan(0.15 / distance) * 203.718327);
    double detectionResults[] = poses.get();
    lastDetectedPixel = Maths.findNearestLinear(detectionResults, 0);
    detectedPixel = Double.NaN;
    // Start the lost-target timeout; it is restarted every cycle the target is seen.
    dontSeeTimer.restart();
  }

  @Override
  public void execute() {
    double distance = rangeSensor.getTargetDistance();
    targetPixel = -(Math.atan(0.15 / distance) * 203.718327);
    double detectionResults[] = poses.get();
    detectedPixel = Maths.findNearestLinear(detectionResults, 0);
    // Accept the detection if it's the first one we've seen (no previous lock),
    // or if it's close to the previous one (rejects jumps to a different target).
    boolean hasDetection = !Double.isNaN(detectedPixel);
    boolean consistent = Double.isNaN(lastDetectedPixel) || Math.abs(detectedPixel - lastDetectedPixel) < 30;
    if (hasDetection && consistent) {
      double speed = Math.copySign(0.15, targetPixel - detectedPixel);
      drive.setVelocity(speed, speed);
      dontSeeTimer.restart();
      lastDetectedPixel = detectedPixel;
      Logger.recordOutput("targetPixel", targetPixel);
    } else {
      drive.setVelocity(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0); 
  }

  @Override
  public boolean isFinished() {
    return MathUtil.isNear(targetPixel, detectedPixel, 1.5) || dontSeeTimer.hasElapsed(0.3);
  }
}
