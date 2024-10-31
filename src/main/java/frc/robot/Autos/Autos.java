package frc.robot.Autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoChooser.AutoRoutineGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auton.FirstShot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Autos extends Command{

    private final static Drivetrain drivetrain = Drivetrain.getInstance();
    private final static Shooter shooter = Shooter.getInstance();
    private final static Intake intake = RobotContainer.getIntake();
    private final static Vision vision = RobotContainer.getVision();

    public AutoLoop fourPieceAuto(AutoFactory factory) {
        final AutoLoop routine = factory.newLoop("4-piece");

        final SendableChooser<String> note1 = new SendableChooser<String>();

        final AutoTrajectory front_n2 = factory.trajectory("Front-2", routine);
        final AutoTrajectory n2_frontShoot = factory.trajectory("2-FrontShoot", routine);
        final AutoTrajectory frontShoot_n3 = factory.trajectory("FrontShoot-3", routine);
        final AutoTrajectory n3_frontShoot = factory.trajectory("3-FrontShoot", routine);
        final AutoTrajectory frontShoot_n1 = factory.trajectory("FrontShoot-1", routine);
        final AutoTrajectory n1_frontShoot = factory.trajectory("1-FrontShoot", routine);
        final AutoTrajectory n2_n1 = factory.trajectory("2-1", routine);
        final AutoTrajectory n1_n3 = factory.trajectory("1-3", routine);

        routine.enabled()
            .onTrue(new FirstShot()
                        .alongWith(new InstantCommand(() -> {
                            vision.turnOffAprilTags();
                        }))
                        .andThen(
                            
                        )
                        );

        return routine;
    }
    
}
