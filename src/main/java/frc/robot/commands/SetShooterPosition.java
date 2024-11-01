package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SetShooterPosition extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private Shooter.PositionState position;

    public SetShooterPosition(Shooter.PositionState position) {
        this.position = position;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.moveTo(position);
    }

    @Override
    public void end(boolean isInturrepted) {

    }

    @Override
    public boolean isFinished() {
        return shooter.atGoal();
        // return true;
    }
}
