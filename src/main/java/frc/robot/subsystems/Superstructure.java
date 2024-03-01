package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;
    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }

        return instance;
    }

    private SystemState lastGoalState = SystemState.STOW;
    private SystemState lastState = SystemState.STOW;
    private SystemState goalState = SystemState.STOW;

    public static boolean hasGamePiece = false;

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Climber climber = Climber.getInstance();


    public enum SystemState {
        INTAKE(Intake.PositionState.GROUND, Shooter.PositionState.HANDOFF),
        STOW(Intake.PositionState.STOW, Shooter.PositionState.HANDOFF),
        PREPARE_SPEAKER(Intake.PositionState.SCORING, Shooter.PositionState.SHOOT),
        PREPARE_AMP(Intake.PositionState.SCORING, Shooter.PositionState.AMP);

        Intake.PositionState intakeState;
        Shooter.PositionState shooterState;
        SystemState(Intake.PositionState intakePos, Shooter.PositionState shooterPos) {
            this.intakeState = intakePos;
            this.shooterState = shooterPos;
        }


        // state of the robot, actions determined by this
        // intake and shooter share ring, when preparing to shoot back off intake
        // probably schedule movement in here?
    }

    public Superstructure() {

    }

    @Override
    public void periodic() {
        moveToGoalState();
        lastGoalState = goalState;
        lastState = atGoalState() ? goalState : lastState;

        SmartDashboard.putString("Superstructure/goalState", goalState.toString());
        SmartDashboard.putString("Superstructure/lastState", lastState.toString());
    }

    private void moveToGoalState() {
        // potentially good logic to sequence/schedule movement
        // if (goalState == SystemState.PREPARE_SPEAKER) {
        //     intake.moveTo(goalState.intakeState);
        //     if (intake.atGoal()) {
        //         shooter.moveTo(goalState.shooterState);
        //     }
        // // } else if (lastState == SystemState.PREPARE_SPEAKER && goalState == SystemState.STOW){
        // //     shooter.moveTo(goalState.shooterState);
        // //     if (shooter.atGoal()) {
        // //         intake.moveTo(goalState.intakeState);
        // //     }
        // // } else {
        // //     intake.moveTo(goalState.intakeState);
        // //     shooter.moveTo(goalState.shooterState);
        // // }
        // } else {
        //     intake.moveTo(goalState.intakeState);
        //     shooter.moveTo(goalState.shooterState);
        // }
    }

    public void setGoalState(SystemState goalState) {
        this.goalState = goalState;
    }

    public boolean atGoalState() {
        return (intake.atGoal() && shooter.atGoal());
    }

    public void reset() {
        goalState = SystemState.STOW;
        lastGoalState = SystemState.STOW;
        lastState = SystemState.STOW;
        intake.reset();
        shooter.reset();
    }

}
