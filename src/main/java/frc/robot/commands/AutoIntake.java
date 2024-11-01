package frc.robot.commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends SequentialCommandGroup {
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    public AutoIntake(PS5Controller controller) {
        
        addCommands(
            new InstantCommand(() -> {intake.setRollerSpeed(0.3); SmartDashboard.putBoolean("Ready to Shoot", false);}),
            new ParallelCommandGroup(
                new SetIntakePosition(Intake.PositionState.GROUND),
                new SetShooterPosition(Shooter.PositionState.HANDOFF)
            ).withTimeout(0),
            new WaitUntilCommand(() -> intake.hasGamePiece()),
            new ParallelCommandGroup(
                new InstantCommand(() -> intake.flashLights(1, 2)),
                new SetIntakePosition(Intake.PositionState.STOW).withTimeout(1),
                new SetShooterPosition(Shooter.PositionState.HANDOFF).withTimeout(0.5),
                new WaitCommand(0.1).andThen(new InstantCommand(() -> {intake.setRollerSpeed(0.0); SmartDashboard.putBoolean("Intake Ring", true);}))
            ),
            new InstantCommand(() -> {intake.setRollerSpeed(0.5); shooter.setFeederSpeed(0.2); shooter.setShootSpeed(-2);}),
            new WaitCommand(0.2),
            new WaitUntilCommand(() -> (shooter.hasGamePiece() || !intake.hasGamePiece())).withTimeout(0.5),
            new InstantCommand(() -> {
                intake.setRollerSpeed(0.0);
                shooter.setFeederSpeed(0.0);
                shooter.setShootSpeed(0);
                intake.flashLights(-1, 10);
                SmartDashboard.putBoolean("Intake Ring", false);
                SmartDashboard.putBoolean("Ready to Shoot", true);
            })

        );
        addRequirements(intake, shooter);
    }
}