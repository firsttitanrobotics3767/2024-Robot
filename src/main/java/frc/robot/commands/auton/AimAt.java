package frc.robot.commands.auton;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class AimAt extends Command{
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    public AimAt(Supplier<Pose2d> pose) {
        
    }

}
