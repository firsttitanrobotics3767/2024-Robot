package frc.robot.commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.FieldLocation;
import swervelib.SwerveController;

public class TeleopDrive extends Command{
    private final Drivetrain drivetrain;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;
    private final Supplier<FieldLocation> faceLocation;

    public TeleopDrive(
        Drivetrain drivetrain,
        DoubleSupplier vX,
        DoubleSupplier vY,
        DoubleSupplier omega,
        BooleanSupplier driveMode,
        Supplier<FieldLocation> faceLocation
    ) {
        this.drivetrain = drivetrain;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.controller = drivetrain.getSwerveController();
        this.faceLocation = faceLocation;

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

        if (faceLocation.get() == FieldLocation.NONE) {
            drivetrain.drive(new Translation2d(xVelocity * drivetrain.maxSpeed, yVelocity * drivetrain.maxSpeed),
                         angVelocity * controller.config.maxAngularVelocity,
                         driveMode.getAsBoolean());
        } else {
            double faceLocationHeading = Math.atan2(
                faceLocation.get().xPos - drivetrain.getPose().getX(),
                faceLocation.get().yPos - drivetrain.getPose().getY()
            );
            drivetrain.driveFieldOriented(drivetrain.getTargetSpeeds(
                xVelocity,
                yVelocity,
                Math.sin(faceLocationHeading),
                Math.cos(faceLocationHeading)
            ));
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}
    
}
