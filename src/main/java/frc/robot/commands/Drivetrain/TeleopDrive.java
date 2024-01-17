package frc.robot.commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class TeleopDrive extends Command{
    private final Drivetrain drivetrain;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;

    public TeleopDrive(Drivetrain drivetrain, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode) {
        this.drivetrain = drivetrain;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.controller = drivetrain.getSwerveController();

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

        drivetrain.drive(new Translation2d(xVelocity * drivetrain.maxSpeed, yVelocity * drivetrain.maxSpeed),
                         angVelocity * controller.config.maxAngularVelocity,
                         driveMode.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}
    
}
