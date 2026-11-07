package frc.robot.gamepiece;

import static org.wpilib.units.Units.Centimeters;
import static org.wpilib.units.Units.Grams;

import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class CargoOnField extends GamePieceOnFieldSimulation {
  public static final GamePieceInfo TEL_10_CARGO_INFO = new GamePieceInfo(
    "Cargo", Geometry.createCircle(0.035), Centimeters.of(7), Grams.of(30), 3.5, 5, 0.5);

  public CargoOnField(Translation2d initialPosition) {
    super(TEL_10_CARGO_INFO, new Pose2d(initialPosition, new Rotation2d()));
  }
}
