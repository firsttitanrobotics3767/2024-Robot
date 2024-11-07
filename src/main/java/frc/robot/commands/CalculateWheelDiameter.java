package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class CalculateWheelDiameter extends Command {

    Drivetrain drive = Drivetrain.getInstance();

    public static final double SWERVE_WHEEL_DISTANCE_FROM_CENTER = 17.151;
    public static final double SWERVE_GEARING_RATIO = 6.12;
    public static final double TARGET_ROTATIONS = 5;
    public static final double ROTATION_SPEED_RADIANS_PER_SECOND = Math.PI; // 1/2 rotation per second

    public class SwerveInfo {
        private int m_index;
        private DoubleSupplier m_swerveCurrentRotations;

        public SwerveInfo(int index) {
            m_index = index;
            m_swerveCurrentRotations = () -> drive.getModule(index).getDriveMotor().getPosition();
        }

        public void init() {
            drive.getModule(m_index).getDriveMotor().setPosition(0);
        }

        public double getRotations() {
            return m_swerveCurrentRotations.getAsDouble();
        }
    }

  private double m_lastGyroDegrees;
  private double m_robotRotations;
  private SwerveInfo[] m_swerveInfo = {
    new SwerveInfo(0), new SwerveInfo(1), new SwerveInfo(2), new SwerveInfo(3),
  };

  public CalculateWheelDiameter() {
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_robotRotations = 0;
    m_lastGyroDegrees = drive.getHeading().getDegrees();

    for (int i = 0; i < 4; i++) {
      m_swerveInfo[i].init();
    }
  }

  @Override
  public void execute() {
    double newGyroDegrees = drive.getHeading().getDegrees();
    double degreesRotated = newGyroDegrees - m_lastGyroDegrees;
    m_robotRotations += (degreesRotated / 360);
    m_lastGyroDegrees = newGyroDegrees;
    
    drive.driveRobotOriented(new ChassisSpeeds(0, 0, ROTATION_SPEED_RADIANS_PER_SECOND));
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("[CalculateWheelDiameter] interrupted!");
      return;
    }

    System.out.println("-----------------------------------------------------");
    double travelCircleDiameterInches = (SWERVE_WHEEL_DISTANCE_FROM_CENTER * 2);
    double travelCircleCircumferenceInches = Math.PI * travelCircleDiameterInches;
    double totalLinearTravelInches = travelCircleCircumferenceInches * m_robotRotations;

    for (int i = 0; i < 4; i++) {
      double wheelRotations = Math.abs(m_swerveInfo[i].getRotations()) / SWERVE_GEARING_RATIO;
      double wheelCircumference = totalLinearTravelInches / wheelRotations;
      double wheelDiameter = wheelCircumference / Math.PI;

      System.out.println("Wheel " + i + " radius = " + (wheelDiameter / 2));
      drive.log("Wheel " + i + " radius", (wheelDiameter/2));
    }
    System.out.println("-----------------------------------------------------");
  }

  @Override
  public boolean isFinished() {
    return m_robotRotations >= TARGET_ROTATIONS;
  }
}