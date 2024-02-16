package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private SystemState lastGoalState = SystemState.STOW;
    private SystemState goalState = SystemState.STOW;

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Elevator elevator = Elevator.getInstance();


    public enum SystemState {
        PREPARE_SHOOT(IntakeState.IDLE),
        PREPARE_AMP(IntakeState.IDLE),
        SCORE(IntakeState.IDLE),
        INTAKE(IntakeState.GROUND),
        STOW(IntakeState.STOW),
        PREPARE_CLIMB(IntakeState.CLIMB),
        CLIMB(IntakeState.CLIMB),
        SCORE_TRAP(IntakeState.CLIMB_LOCK);

        public IntakeState intakeState;
        private SystemState(IntakeState intakeState) {
            this.intakeState = intakeState;
        }
    }

    public enum IntakeState {
        //none of these values are tested
        GROUND(-0.3),
        IDLE(0),
        STOW(-0.1),
        HANDOFF(0.2),
        CLIMB(-0.2),
        CLIMB_LOCK(0);

        public double pos;
        private IntakeState(double pos) {
            this.pos = pos;
        }
    }

    public Superstructure() {

    }

    @Override
    public void periodic() {
        moveToGoalState();
    }

    private void moveToGoalState() {
        // potentially good logic to schedule movement
        // if (lastGoalState == SystemState.STOW && goalState == SystemState.SCORE) {
        //     intake.moveTo(goalState.intakeState.pos);
        //     if (intake.atGoal()) {
        //         shooter.moveTo(goalState.shooterState.pos)
        //     }
        // } else {
            intake.moveTo(goalState.intakeState.pos);
            // shooter.moveTo(goalState.intakeState.pos);
    }

}
