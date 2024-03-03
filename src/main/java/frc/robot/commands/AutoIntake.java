package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
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

    public AutoIntake(PS5Controller controller) {
        
        addCommands(
            new InstantCommand(() -> {intake.setRollerSpeed(0.3); SmartDashboard.putBoolean("Ready to Shoot", false);}),
            new ParallelCommandGroup(
                new SetIntakePosition(Intake.PositionState.GROUND).withTimeout(1),
                new SetShooterPosition(Shooter.PositionState.HANDOFF).withTimeout(0.5)
            ),
            new WaitUntilCommand(() -> intake.hasGamePiece()),
            new InstantCommand(() -> {intake.setRollerSpeed(0.0); SmartDashboard.putBoolean("Intake Ring", true);}),
            // new InstantCommand(() -> superstructure.setGoalState(Superstructure.SystemState.STOW)).withTimeout(2).handleInterrupt(() -> this.end(true)),
            // new InstantCommand(() -> superstructure.setGoalState(Superstructure.SystemState.STOW)),
            new ParallelCommandGroup(
                new SetIntakePosition(Intake.PositionState.STOW).withTimeout(1.2),
                new SetShooterPosition(Shooter.PositionState.HANDOFF).withTimeout(0.5)
            ),

            new InstantCommand(() -> {intake.setRollerSpeed(0.2); shooter.setFeederSpeed(0.2); shooter.setShootSpeed(-2);}),
            new WaitCommand(0.4),
            new WaitUntilCommand(() -> intake.getTorqueCurrent() < 25),
            // new WaitCommand(1.5),
            new InstantCommand(() -> {intake.setRollerSpeed(0.0); shooter.setFeederSpeed(0.0); shooter.setShootSpeed(0); SmartDashboard.putBoolean("Intake Ring", false); SmartDashboard.putBoolean("Ready to Shoot", true);})

        );
        addRequirements(intake, shooter);
    }
}