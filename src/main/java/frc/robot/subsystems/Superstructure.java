package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    public enum SystemState {
        PREPARE_SHOOT,
        PREPARE_AMP,
        SCORE,
        INTAKE,
        IDLE,
        PREPARE_CLIMB,
        CLIMB,
        SCORE_TRAP
    }

    private SystemState currentState = SystemState.IDLE;
    private SystemState goalState = SystemState.IDLE;

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    public Superstructure() {

    }

    @Override
    public void periodic() {
        switch(goalState) {
            case IDLE -> {
                
            }
        }
    }

}
