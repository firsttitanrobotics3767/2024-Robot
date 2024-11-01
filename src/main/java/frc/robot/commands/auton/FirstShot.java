package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SetShooterPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FirstShot extends SequentialCommandGroup {
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();

    public FirstShot() {
        addCommands(
            new InstantCommand(() -> intake.setRollerSpeed(0.1)),
            new InstantCommand(() -> intake.moveTo(Intake.PositionState.SCORING)),
            new WaitCommand(0.3),
            new SetShooterPosition(Shooter.PositionState.SHOOT).withTimeout(1),

            new InstantCommand(() -> {shooter.setFeederSpeed(-0.1); shooter.setShootSpeed(-2); intake.setRollerSpeed(0);}),
            new WaitCommand(0.2),
            new InstantCommand(() -> {shooter.setShootSpeed(80); shooter.setFeederSpeed(0); System.out.println("end shoot");}),
            new WaitCommand(1)
        );
    }
}
