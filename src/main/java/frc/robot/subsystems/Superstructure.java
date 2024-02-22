package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private SystemState lastGoalState = SystemState.IDLE;
    private SystemState goalState = SystemState.IDLE;

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Climber climber = Climber.getInstance();


    public enum SystemState {
        // IDLE(IntakeState.IDLE, ShooterState.HANDOFF),
        IDLE(IntakeState.GROUND, ShooterState.AMP),
        PREPARE_SHOOT(IntakeState.HANDOFF, ShooterState.HANDOFF),
        PREPARE_AMP(IntakeState.IDLE, ShooterState.AMP),
        SCORE(IntakeState.IDLE, ShooterState.HANDOFF),
        INTAKE(IntakeState.GROUND, ShooterState.HANDOFF),
        STOW(IntakeState.STOW, ShooterState.IDLE),
        PREPARE_CLIMB(IntakeState.CLIMB, ShooterState.AMP),
        CLIMB(IntakeState.GROUND, ShooterState.AMP),
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
        IDLE(0.25),
        STOW(0.25),
        HANDOFF(0.26), 
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
        AMP(0.35),
        HANDOFF(0.17);

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
            } else if (goalState == SystemState.PREPARE_SHOOT) {
                intake.setRollerSpeed(-0.2);
                shooter.setFeederSpeed(-0.2);
                shooter.setShootSpeed(1);
            } else if (goalState == SystemState.SCORE) {
                shooter.setFeederSpeed(0);
                shooter.setShootSpeed(1);
            } else {
                shooter.setShootSpeed(0);
                shooter.setFeederSpeed(0);
                intake.setRollerSpeed(0);
            }
            
            shooter.moveTo(goalState.shooterState.pos);
    }

    public void setGoalState(SystemState goalState) {
        this.goalState = goalState;
    }

}
