package frc.FSLib.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;

public class PhoenixUntil {
  public static void assertOk(Pigeon2 pigeon, Supplier<StatusCode> command) {
    StatusCode status = command.get();
    if (status != StatusCode.OK) {
      DriverStationErrors.reportError("can't apply configuration to pigeon with ID " + pigeon.getDeviceID(), true);
    }
  }

  public static void assertOk(CANcoder cancoder, Supplier<StatusCode> command) {
    StatusCode status = command.get();
    if (status != StatusCode.OK) {
      DriverStationErrors.reportError("can't apply configuration to CANcoder with ID " + cancoder.getDeviceID(), true);
    }
  }

  public static void assertOk(TalonFX talonFX, Supplier<StatusCode> command) {
    StatusCode status = command.get();
    if (status != StatusCode.OK) {
      DriverStationErrors.reportError("can't apply configuration to talonFX with ID " + talonFX.getDeviceID(), true);
    }
  }
}
