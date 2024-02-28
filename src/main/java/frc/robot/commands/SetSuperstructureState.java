package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class SetSuperstructureState extends Command {
    private final Superstructure superstructure = Superstructure.getInstance();
    private Superstructure.SystemState state;

    public SetSuperstructureState(Superstructure.SystemState state) {
        this.state = state;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setGoalState(state);
    }

    @Override
    public void end(boolean isInturrepted) {

    }

    @Override
    public boolean isFinished() {
        return superstructure.atGoalState();
    }
}
