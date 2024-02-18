package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private SystemState lastGoalState = SystemState.IDLE;
    private SystemState goalState = SystemState.IDLE;

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Shooter shooter = Shooter.getInstance();


    public enum SystemState {
        IDLE(IntakeState.IDLE, ShooterState.IDLE),
        PREPARE_SHOOT(IntakeState.HANDOFF, ShooterState.HANDOFF),
        PREPARE_AMP(IntakeState.IDLE, ShooterState.AMP),
        SCORE(IntakeState.IDLE, ShooterState.SHOOT),
        INTAKE(IntakeState.GROUND, ShooterState.IDLE),
        STOW(IntakeState.STOW, ShooterState.IDLE),
        PREPARE_CLIMB(IntakeState.CLIMB, ShooterState.AMP),
        CLIMB(IntakeState.CLIMB, ShooterState.AMP),
        SCORE_TRAP(IntakeState.CLIMB_LOCK, ShooterState.AMP);

        public IntakeState intakeState;
        public ShooterState shooterState;
        private SystemState(IntakeState intakeState, ShooterState shooterState) {
            this.intakeState = intakeState;
            this.shooterState = shooterState;
        }
    }

    public enum IntakeState {
        GROUND(0.02),
        IDLE(0.24),
        STOW(0.25),
        HANDOFF(0.25), 
        CLIMB(0.05), 
        CLIMB_LOCK(.25); 

        public double pos;
        private IntakeState(double pos) {
            this.pos = pos;
        }
    }

    public enum ShooterState {
        IDLE(0.05),
        SHOOT(0.3),
        AMP(0.3),
        HANDOFF(0.12);

        public double pos;
        private ShooterState(double pos) {
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
        // potentially good logic to sequence/schedule movement
        // if (lastGoalState == SystemState.STOW && goalState == SystemState.SCORE) {
        //     intake.moveTo(goalState.intakeState.pos);
        //     if (intake.atGoal()) {
        //         shooter.moveTo(goalState.shooterState.pos)
        //     }
        // } else {
            intake.moveTo(goalState.intakeState.pos);
            if (goalState == SystemState.INTAKE) {
                intake.setRollerSpeed(-0.2);
            } else {
                intake.setRollerSpeed(0);
            }
            
            shooter.moveTo(goalState.shooterState.pos);
    }

    public void setGoalState(SystemState goalState) {
        this.goalState = goalState;
    }

}
