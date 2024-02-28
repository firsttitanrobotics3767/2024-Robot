package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntakePosition extends Command {
    private final Intake intake = Intake.getInstance();
    private Intake.PositionState position;

    public SetIntakePosition(Intake.PositionState position) {
        this.position = position;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.moveTo(position);
    }

    @Override
    public void end(boolean isInturrepted) {

    }

    @Override
    public boolean isFinished() {
        // return intake.atGoal();
        return true;
    }
}
