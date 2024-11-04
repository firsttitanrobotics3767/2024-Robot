package frc.robot.Autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.FaceLocation;
import frc.robot.commands.auton.DeployIntake;
import frc.robot.commands.auton.FirstShot;
import frc.robot.commands.auton.PrepareShootAutoAim;
import frc.robot.commands.auton.ShootAuton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Autos {

    public static AutoLoop fourPieceAuto(AutoFactory factory) {
        final AutoLoop routine = factory.newLoop("4-piece");

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
                            Drivetrain.getInstance().resetOdometry(front_n2.getInitialPose().get());
                        }))
                        .andThen(new InstantCommand(() -> {
                            Vision.getInstance().turnOffAprilTags();
                        }))
                        .andThen(
                            new ParallelRaceGroup(
                                deployIntake(),
                                front_n2.cmd()
                            ))
            .withName("Four Piece Auto Entry Point"));

        front_n2.done().and(hasGamePiece(routine)).onTrue(
            new ParallelRaceGroup(
                n2_frontShoot.cmd(),
                new WaitUntilCommand(() -> (Shooter.getInstance().getEstimatedShotAngle(DriverStation.getAlliance().orElse(Alliance.Blue)) < 1))
            ).alongWith(
                new InstantCommand(() -> {
                    RobotContainer.setFaceLocation(FaceLocation.Speaker);
                }),
                new PrepareShootAutoAim()
            ).andThen(
                new ShootAuton()
            ).andThen(
                new InstantCommand(() -> {
                    RobotContainer.setFaceLocation(FaceLocation.None);
                })
            ).andThen(
                new ParallelRaceGroup(
                    frontShoot_n1.cmd(),
                    deployIntake()
                )
            )
        );
        
        front_n2.done().and(hasGamePiece(routine).negate()).onTrue(
            new ParallelRaceGroup(
                n2_n1.cmd(),
                deployIntake()
            )
        );

        frontShoot_n1.done().or(n2_n1.done()).and(hasGamePiece(routine)).onTrue(
            new ParallelRaceGroup(
                n1_frontShoot.cmd(),
                new WaitUntilCommand(() -> (Shooter.getInstance().getEstimatedShotAngle(DriverStation.getAlliance().get()) < 1))
            ).alongWith(
                new InstantCommand(() -> {
                    RobotContainer.setFaceLocation(FaceLocation.Speaker);
                }),
                new PrepareShootAutoAim()
            ).andThen(
                new ShootAuton()
            ).andThen(
                new InstantCommand(() -> {
                    RobotContainer.setFaceLocation(FaceLocation.None);
                })
            ).andThen(
                new ParallelRaceGroup(
                    frontShoot_n3.cmd(),
                    deployIntake()
                )
            )
        );

        frontShoot_n1.done().or(n2_n1.done()).and(hasGamePiece(routine).negate()).onTrue(
            new ParallelRaceGroup(
                n1_n3.cmd(),
                deployIntake()
            )
        );
        
        frontShoot_n3.done().or(n1_n3.done()).onTrue(
            new ParallelRaceGroup(
                n3_frontShoot.cmd(),
                new WaitUntilCommand(() -> (Shooter.getInstance().getEstimatedShotAngle(DriverStation.getAlliance().get()) < 1))
            ).alongWith(
                new InstantCommand(() -> {
                    RobotContainer.setFaceLocation(FaceLocation.Speaker);
                }),
                new PrepareShootAutoAim()
            ).andThen(
                new ShootAuton()
            ).andThen(
                new InstantCommand(() -> {
                    RobotContainer.setFaceLocation(FaceLocation.None);
                })
            )
        );

        return routine;
    }

    public static AutoLoop test(AutoFactory factory) {
        final AutoLoop routine = factory.newLoop("test path");

        final AutoTrajectory trajectory = factory.trajectory("test path", routine);

        routine.enabled().onTrue(
            trajectory.cmd()
        );

        return routine;
    }

    private static Command deployIntake() {
        return new DeployIntake()
            .andThen(new InstantCommand(() -> {
                Intake.getInstance().setRollerSpeed(0);
                Intake.getInstance().moveTo(Intake.PositionState.STOW);
            })
        );
    }

    private static Trigger hasGamePiece(AutoLoop routine) {
        Trigger trigger = new Trigger(routine.getLoop(), () -> Intake.getInstance().hasGamePiece() || Shooter.getInstance().hasGamePiece());
        return trigger;
    }
    
}
