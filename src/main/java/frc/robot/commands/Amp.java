package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Amp extends SequentialCommandGroup {
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final BooleanSupplier shootButton;

    public Amp(BooleanSupplier shootButton) {
        this.shootButton = shootButton;
        addCommands(
            new InstantCommand(() -> intake.setRollerSpeed(0.1)),
            new InstantCommand(() -> intake.moveTo(Intake.PositionState.SCORING)),
            new WaitCommand(0.3),
            new SetShooterPosition(Shooter.PositionState.AMP).withTimeout(0.7),
            new InstantCommand(() -> {intake.setRollerSpeed(0); elevator.moveTo(Elevator.PositionState.AMP);}),

            new WaitUntilCommand(shootButton),
            new InstantCommand(() -> {shooter.setFeederSpeed(0.30); shooter.setShootSpeed(20);}),
            new WaitCommand(0.3),
            new InstantCommand(() -> {shooter.setShootSpeed(0); shooter.setFeederSpeed(0);}),
            new ParallelCommandGroup(
                new SetShooterPosition(Shooter.PositionState.HANDOFF),
                new SetIntakePosition(Intake.PositionState.STOW),
                new InstantCommand(() -> elevator.moveTo(Elevator.PositionState.STOW))
            )
        );
        addRequirements(elevator);
    }
}
