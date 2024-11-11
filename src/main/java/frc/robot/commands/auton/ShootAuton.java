package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SetShooterPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.CommandTrigger;

public class ShootAuton extends CommandTrigger {
    private final Shooter shooter;

    boolean finished = false;

    public ShootAuton(EventLoop loop, Shooter shooter) {
        this.shooter = shooter;
        addLoop(loop);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new InstantCommand(() -> {shooter.setFeederSpeed(0.30); System.out.println("shoot");}),
            new WaitCommand(0.2),
            new WaitUntilCommand(() -> !shooter.hasGamePiece()),
            new InstantCommand(() -> {shooter.setShootSpeed(0); shooter.setFeederSpeed(0); shooter.moveTo(Shooter.PositionState.HANDOFF);})
        ).finallyDo(() -> {finished = true;});
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
