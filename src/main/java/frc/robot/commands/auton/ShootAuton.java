package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SetShooterPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootAuton extends SequentialCommandGroup {
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();

    public ShootAuton() {
        addCommands(
            new InstantCommand(() -> {shooter.setFeederSpeed(0.30); System.out.println("shooot");}),
            new WaitCommand(0.2),
            new WaitUntilCommand(() -> !shooter.hasGamePiece()),
            new InstantCommand(() -> {shooter.setShootSpeed(0); shooter.setFeederSpeed(0); shooter.moveTo(Shooter.PositionState.HANDOFF);})
        );
    }
}
