package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class Shoot extends SequentialCommandGroup {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();
    private final BooleanSupplier shootButton;

    public Shoot(BooleanSupplier shootButton) {
        this.shootButton = shootButton;
        addCommands(
            new InstantCommand(() -> intake.setRollerSpeed(0.1)),
            new SetSuperstructureState(Superstructure.SystemState.PREPARE_SPEAKER),
            new InstantCommand(() -> {shooter.setFeederSpeed(-0.1); intake.setRollerSpeed(0);}),
            new WaitCommand(0.1),
            new InstantCommand(() -> {shooter.setShootSpeed(80); shooter.setFeederSpeed(0);}),
            new WaitUntilCommand(shootButton),
            new InstantCommand(() -> shooter.setFeederSpeed(0.30)),
            new WaitCommand(0.3),
            new InstantCommand(() -> {shooter.setShootSpeed(0); shooter.setFeederSpeed(0);}),
            new SetSuperstructureState(Superstructure.SystemState.STOW)
        );
    }
}
