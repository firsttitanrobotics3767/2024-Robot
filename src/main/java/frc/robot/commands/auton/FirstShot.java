package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SetShooterPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.CommandTrigger;

public class FirstShot extends CommandTrigger {
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = RobotContainer.getIntake();

    private boolean hasShot = false;

    public FirstShot() {
        addRequirements(shooter, intake);
        addLoop(new EventLoop());
    }

    @Override
    public void initialize() {
        active(true);
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.setRollerSpeed(0.1)),
            new InstantCommand(() -> intake.moveTo(Intake.PositionState.SCORING)),
            new WaitCommand(0.3),
            new SetShooterPosition(Shooter.PositionState.SHOOT).withTimeout(1),

            new InstantCommand(() -> {shooter.setFeederSpeed(-0.1); shooter.setShootSpeed(-2); intake.setRollerSpeed(0);}),
            new WaitCommand(0.2),
            new InstantCommand(() -> {shooter.setShootSpeed(80); shooter.setFeederSpeed(0); System.out.println("end shoot");}),
            new WaitCommand(1),
            new InstantCommand(() -> {hasShot = true;})
        );
    }

    @Override
    public boolean isFinished() {
        return hasShot;
    }
}
