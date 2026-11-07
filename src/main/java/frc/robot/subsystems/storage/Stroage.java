package frc.robot.subsystems.storage;

import static frc.robot.subsystems.storage.StorageConstants.*;

import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.drivers.motor.Spark;
import org.wpilib.command2.SubsystemBase;

public class Stroage extends SubsystemBase {
  private final DigitalOutput storageReverse = new DigitalOutput(kStorageForwardPort);
  private final DigitalOutput storageForward = new DigitalOutput(kStorageReversePort);
  private final DigitalOutput triggerReverse = new DigitalOutput(kTriggerForwardPort);
  private final DigitalOutput triggerForward = new DigitalOutput(kTriggerReversePort);

  private final Spark storage = new Spark(kStorageSpeedPort);
  private final Spark trigger = new Spark(kTriggerSpeedPort);

  public void setStorageSpeed(double speed) {
    storage.setThrottle(speed);
  }

  public void setStorageDirection(boolean forward) {
    storageForward.set(forward);
    storageReverse.set(!forward);
  }

  public void disableStorage() {
    storageForward.set(false);
    storageReverse.set(false);
  }

  public void setTriggerSpeed(double speed) {
    trigger.setThrottle(speed);
  }

  public void setTriggerDirection(boolean forward) {
    triggerForward.set(forward);
    triggerReverse.set(!forward);
  }

  public void disableTrigger() {
    triggerForward.set(false);
    triggerReverse.set(false);
  }
}
