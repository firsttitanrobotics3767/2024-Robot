package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SetShooterPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DeployIntake extends SequentialCommandGroup {
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = RobotContainer.getIntake();

    public DeployIntake() {
        new SequentialCommandGroup(
            new InstantCommand(() -> {intake.setRollerSpeed(0.3); SmartDashboard.putBoolean("Ready to Shoot", false);}),
            new ParallelCommandGroup(
                new SetIntakePosition(Intake.PositionState.GROUND),
                new SetShooterPosition(Shooter.PositionState.HANDOFF)
            ).withTimeout(0),
            new WaitUntilCommand(() -> intake.hasGamePiece())
        );
        addRequirements(intake, shooter);
    }
    
}

