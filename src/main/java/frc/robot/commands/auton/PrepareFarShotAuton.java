package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SetShooterPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PrepareFarShotAuton extends SequentialCommandGroup{
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = RobotContainer.getIntake();

    public PrepareFarShotAuton() {
        addCommands(
            new InstantCommand(() -> intake.setRollerSpeed(0.1)),
            new InstantCommand(() -> intake.moveTo(Intake.PositionState.SCORING)),
            new WaitCommand(0.2),
            new SetShooterPosition(Shooter.PositionState.AUTO).withTimeout(0.2),

            new InstantCommand(() -> {shooter.setFeederSpeed(-0.1); shooter.setShootSpeed(-2); intake.setRollerSpeed(0);}),
            new WaitUntilCommand(() -> !shooter.hasGamePiece()),
            new InstantCommand(() -> {shooter.setShootSpeed(90); shooter.setFeederSpeed(0); System.out.println("end shoot");}),
            new WaitUntilCommand(() -> shooter.getWheelSpeed() > 75)
        );
        addRequirements(intake, shooter);
    }
}
