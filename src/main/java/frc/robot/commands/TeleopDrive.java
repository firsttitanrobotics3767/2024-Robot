package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;
import swervelib.SwerveController;

public class TeleopDrive extends Command{
    private final Drivetrain drivetrain;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY, headingX, headingY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;
    private final Supplier<Translation2d> faceLocation;
    private final PIDController pidController;

    public TeleopDrive(
        Drivetrain drivetrain,
        DoubleSupplier vX,
        DoubleSupplier vY,
        DoubleSupplier omega,
        BooleanSupplier driveMode,
        Supplier<Translation2d> faceLocation,
        DoubleSupplier headingX,
        DoubleSupplier headingY
    ) {
        this.drivetrain = drivetrain;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.controller = drivetrain.getSwerveController();
        this.faceLocation = faceLocation;
        this.headingX = headingX;
        this.headingY = headingY;

        pidController = new PIDController(4.3, 0, 0);
        pidController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xVelocity = vX.getAsDouble();
        double yVelocity = vY.getAsDouble();
        double angVelocity = omega.getAsDouble();
        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);

        if (faceLocation.get().equals(Constants.FieldLocations.none)) {
            drivetrain.drive(new Translation2d(xVelocity * drivetrain.maxSpeed, yVelocity * drivetrain.maxSpeed),
                            angVelocity * controller.config.maxAngularVelocity,
                            driveMode.getAsBoolean());
        } else {
            Translation2d target = faceLocation.get();
            double faceLocationHeading = Math.atan2(drivetrain.getPose().getY() - target.getY(), drivetrain.getPose().getX() - target.getX()) + ((omega.getAsDouble() > 0.1) ? omega.getAsDouble() * 0.1 : 0.0);
            // double faceLocationHeading = Math.atan2(headingY.getAsDouble(), headingX.getAsDouble());
            
            // ChassisSpeeds desiredSpeeds = drivetrain.getTargetSpeeds(
            //     xVelocity * drivetrain.maxSpeed, 
            //     yVelocity * drivetrain.maxSpeed,
            //     new Rotation2d(faceLocationHeading)
            // );

            drivetrain.drive(new Translation2d(xVelocity * drivetrain.maxSpeed, yVelocity * drivetrain.maxSpeed), pidController.calculate(drivetrain.getHeading().getRadians(), faceLocationHeading), driveMode.getAsBoolean());

            System.out.println("facing location " + headingX.getAsDouble() + "   " + headingY.getAsDouble() + " | " + Units.radiansToDegrees(faceLocationHeading) + " : " + drivetrain.getHeading().getDegrees());
        }


    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}
    
}
