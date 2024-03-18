package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class Shoot extends SequentialCommandGroup {
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();
    private final BooleanSupplier shootButton;

    public Shoot(BooleanSupplier shootButton) {
        this.shootButton = shootButton;
        addCommands(
            new InstantCommand(() -> intake.setRollerSpeed(0.1)),
            new InstantCommand(() -> intake.moveTo(Intake.PositionState.SCORING)),
            new WaitCommand(0.3),
            new SetShooterPosition(Shooter.PositionState.SHOOT).withTimeout(1),

            new InstantCommand(() -> {shooter.setFeederSpeed(-0.1); shooter.setShootSpeed(-2); intake.setRollerSpeed(0);}),
            new WaitCommand(0.15),
            new InstantCommand(() -> {shooter.setShootSpeed(80); shooter.setFeederSpeed(0);}),
            new WaitUntilCommand(shootButton),
            new InstantCommand(() -> shooter.setFeederSpeed(0.30)),
            new WaitCommand(0.3),
            new InstantCommand(() -> {shooter.setShootSpeed(0); shooter.setFeederSpeed(0);}),
            // new SetSuperstructureState(Superstructure.SystemState.STOW)
            new ParallelCommandGroup(
                new SetShooterPosition(Shooter.PositionState.HANDOFF),
                new SetIntakePosition(Intake.PositionState.STOW)
            )
        );
        addRequirements(intake, shooter);
    }
}
