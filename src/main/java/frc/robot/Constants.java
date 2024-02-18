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
        public static final int resetOdometryButton = 3;
        public static final int driveModeButton = 7;
        public static final int faceSpeakerButton = 8;
    }

    public static class Swerve {
        public static final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
        public static final double maxVelocity = 5;
        public static final double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.12, 1);
        public static final double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(21.4285714285714, 1);
    }

    public static class Intake {
        public static final int positionCANID = 13;
        public static final int rollerCANID = 14;

        public static final double conversionFactor = 1;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double gravityFFVolts = 0;
        public static final double maxVel = 1;
        public static final double maxAccel = 1;

        public static final double defaultPosition = 1;
    }

    public static class Elevator {
        public static final int motorID = 15;
        public static final double conversionFactor = 1;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double gravityFFVolts = 0;
        public static final double maxVel = 1;
        public static final double maxAccel = 1;

        public static final double defaultPosition = 1;
    }

    public static class Shooter {
        public static final int feederCANID = 18;
        public static final int topCANID = 17;
        public static final int bottomCANID = 16;
        public static final int rotationCANID = 15;


        public static final double topP = 0;
        public static final double topI = 0;
        public static final double topD = 0;
        public static final double topFF = 0;

        public static final double bottomP = 0;
        public static final double bottomI = 0;
        public static final double bottomD = 0;
        public static final double bottomFF = 0;
        
        public static final double rotationP = 0;
        public static final double rotationI = 0;
        public static final double rotationD = 0;
        public static final double rotationFF = 0;
        public static final double conversionFactor = 1;
        public static final double maxVel = 1;
        public static final double maxAcc = 1;

        public static final double feederP = 0;
        public static final double feederI = 0;
        public static final double feederD = 0;
        public static final double feederFF = 0;
    }
}
