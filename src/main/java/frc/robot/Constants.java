package frc.robot;

import java.io.File;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.math.SwerveMath;

public class Constants {
    public static class IO {
        public static final double swerveDeadband = 0.04;
        public static final int driveXAxis = 1;
        public static final int driveYAxis = 0;
        public static final int driveOmegaAxis = 2;
        public static final int resetGyroButton = 2;
        public static final int driveModeButton = 7;
    }

    public static class Swerve {
        public static final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
        public static final double maxVelocity = 1;
        public static final double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.12, 1);
        public static final double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(21.4285714285714, 1);
    }
}
