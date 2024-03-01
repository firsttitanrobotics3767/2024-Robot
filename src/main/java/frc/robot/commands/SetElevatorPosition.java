package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class SetElevatorPosition extends Command {
    private final Elevator elevator = Elevator.getInstance();
    private Elevator.PositionState position;

    public SetElevatorPosition(Elevator.PositionState position) {
        this.position = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.moveTo(position);
    }

    @Override
    public void end(boolean isInturrepted) {

    }

    @Override
    public boolean isFinished() {
        return elevator.atGoal();
    }
}
