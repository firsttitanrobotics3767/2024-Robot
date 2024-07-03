package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double fieldLength = 16.451;
    public static final double fieldWidth = 8.211;

    public static final double noteSpacing = Units.inchesToMeters(64);

      public static final List<Translation2d> notePositions =
      List.of(
          new Translation2d(fieldLength / 2, Units.inchesToMeters(32)),
          new Translation2d(fieldLength / 2, Units.inchesToMeters(32) + noteSpacing),
          new Translation2d(fieldLength / 2, Units.inchesToMeters(32) + noteSpacing * 2),
          new Translation2d(fieldLength / 2, Units.inchesToMeters(32) + noteSpacing * 3),
          new Translation2d(fieldLength / 2, Units.inchesToMeters(32) + noteSpacing * 4));
}
