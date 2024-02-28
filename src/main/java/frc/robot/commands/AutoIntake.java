package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.SetIntakePosition;
import frc.robot.commands.shooter.SetShooterPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;

public class AutoIntake extends SequentialCommandGroup {
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final GenericHID controller = new GenericHID(0);

    public AutoIntake() {
        addCommands(
            new InstantCommand(() -> intake.setRollerSpeed(0.3)),
            new SetSuperstructureState(Superstructure.SystemState.INTAKE),
            new WaitUntilCommand(() -> intake.hasGamePiece()),
            new InstantCommand(() -> {intake.setRollerSpeed(0.0); SmartDashboard.putBoolean("Intake/ready", true);}),
            // new InstantCommand(() -> superstructure.setGoalState(Superstructure.SystemState.STOW)).withTimeout(2).handleInterrupt(() -> this.end(true)),
            // new InstantCommand(() -> superstructure.setGoalState(Superstructure.SystemState.STOW)),
            new SetSuperstructureState(Superstructure.SystemState.STOW),

            new InstantCommand(() -> {intake.setRollerSpeed(0.2); shooter.setFeederSpeed(0.2); shooter.setShootSpeed(-2); SmartDashboard.putBoolean("Intake/ready", false);}),
            new WaitCommand(0.3),
            new WaitUntilCommand(() -> intake.getTorqueCurrent() < 25),
            // new WaitCommand(1.5),
            new InstantCommand(() -> {intake.setRollerSpeed(0.0); shooter.setFeederSpeed(0.0); shooter.setShootSpeed(0);})

        );
    }
}