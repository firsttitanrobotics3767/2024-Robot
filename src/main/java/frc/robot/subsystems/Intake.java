package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/**
 * Intake class
 */
public class Intake extends SubsystemBase{
    public enum ControlState {
        AUTOMATIC,
        MANUAL;
    }

    public enum PositionState {
        GROUND(0.005),
        SCORING(0.15),
        STOW(0.32);
        // STOW(0.02);

        public double pos;
        private PositionState(double pos) {
            this.pos = pos;
        }
    }

    public enum MovementState {
        MOVING,
        AT_GOAL;
    }

    private static Intake instance = null;
    // private final Superstructure superstructure = Superstructure.getInstance();
    // private final Shooter shooter = Shooter.getInstance();
    private ControlState controlState = ControlState.AUTOMATIC;
    private MovementState movementState = MovementState.MOVING;
    private PositionState lastState = PositionState.STOW;
    private PositionState lastGoalState = PositionState.STOW;
    private PositionState goalState = PositionState.STOW;
    private double targetOpenLoopOutput = 0;
    private double targetRollerSpeed = 0;
    private boolean hasGamePiece = false;
    private double startTime = 0;
    private double travelTime = 1;
    private double sensorDistance = 0;

    
    
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    // Rollers
    private final TalonFX rollerMotor;
    private final TalonFXConfiguration rollerConfig;

    private final DigitalInput sensor = new DigitalInput(0);

    // Pivot
    private final CANSparkMax positionMotor;
    private final RelativeEncoder positionEncoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkPIDController positionController;

    public Intake() {
        rollerMotor = new TalonFX(Constants.Intake.rollerCANID);
        rollerConfig = new TalonFXConfiguration();
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setInverted(false);


        positionMotor = new CANSparkMax(Constants.Intake.positionCANID, MotorType.kBrushless);
        positionMotor.restoreFactoryDefaults();
        positionMotor.setIdleMode(IdleMode.kBrake);
        positionMotor.setInverted(false);
        positionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        positionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        positionMotor.setSoftLimit(SoftLimitDirection.kForward, (float)0.33);
        positionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.0);
        positionMotor.setOpenLoopRampRate(Constants.Intake.openLoopRampRate);
        positionMotor.setSmartCurrentLimit(40);
        
        absoluteEncoder = positionMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Constants.Intake.absoluteConversionFactor);
        absoluteEncoder.setVelocityConversionFactor(Constants.Intake.absoluteConversionFactor);
        absoluteEncoder.setZeroOffset(Constants.Intake.absoluteOffset);
        
        positionEncoder = positionMotor.getEncoder();
        positionEncoder.setPositionConversionFactor(Constants.Intake.conversionFactor);
        positionEncoder.setVelocityConversionFactor(Constants.Intake.conversionFactor);
        resetPosition(absoluteEncoder.getPosition());


        positionController = positionMotor.getPIDController();
        positionController.setFeedbackDevice(absoluteEncoder);
        positionController.setP(Constants.Intake.positionP);
        positionController.setI(Constants.Intake.positionI);
        positionController.setD(Constants.Intake.positionD);
        positionController.setSmartMotionMaxAccel(Constants.Intake.maxAccel, 0);
        positionController.setSmartMotionMaxVelocity(Constants.Intake.maxVel, 0);

        
    }

    @Override
    public void periodic() {
        // sensorDistance = SmartDashboard.getNumber("Intake/sensorDistance", 0);
        hasGamePiece = hasGamePiece();

        if (controlState == ControlState.AUTOMATIC) {
            positionController.setReference(goalState.pos, ControlType.kSmartMotion, 0, 
                Math.cos(getPosition() * Math.PI * 2.0) * Constants.Intake.positionGravityFF);
        } else {
            positionMotor.set(targetOpenLoopOutput + (Math.cos(getPosition() * Math.PI * 2.0) * Constants.Intake.positionGravityFF));
        }
        rollerMotor.set(targetRollerSpeed);

        lastState = atGoal()  ? goalState : lastState;

        // if (!hasGamePiece && lastState == PositionState.STOW && movementState == MovementState.MOVING && goalState == PositionState.STOW) {
        //     startHandoff();
        //     Shooter.getInstance().startHandoff();
        // }

        movementState = atGoal() ? MovementState.AT_GOAL : MovementState.MOVING;

        SmartDashboard.putNumber("Intake/measuredPosition", getPosition());
        SmartDashboard.putNumber("Intake/Absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Intake/measuredRotationVelocity", positionEncoder.getVelocity());
        SmartDashboard.putNumber("Intake/measuredRollerVelocity", rollerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/atGoal", atGoal());
        SmartDashboard.putString("Intake/lastState", lastState.toString());
        SmartDashboard.putString("Intake/goalState", goalState.toString());
        SmartDashboard.putBoolean("Intake/hasGamePiece", hasGamePiece());
        SmartDashboard.putBoolean("intkae sensor", sensor.get());
        SmartDashboard.putNumber("Intake/roller torque", rollerMotor.getTorqueCurrent().getValueAsDouble());
    }

    /**
     * Set the velocity of the intake deploy arms
     * @param speed double of the velocity
     */
    public void setPositionSpeed(double speed) {
        targetOpenLoopOutput = speed * 0.2;
    }

    /**
     * Set the velocity of the intake rollers
     * @param speed double of the velocity
     */
    public void setRollerSpeed(double speed) {
        targetRollerSpeed = speed;
        SmartDashboard.putNumber("Intake/Target Roller Speed", speed);
    }

    /**
     * Set the position of the intake
     * @param position double of the rotational position
     */
    public void moveTo(PositionState positionState) {
        goalState = positionState;
    }
    
    public void setControlState(ControlState controlState) {
        this.controlState = controlState;
    }

    public boolean atGoal() {
        return ((getAbsolutePosition() > (goalState.pos - 0.01)) && (getAbsolutePosition() < (goalState.pos + 0.01)));
    }

    public MovementState getMovementState() {
        return movementState;
    }

    public double getPosition() {
        return positionEncoder.getPosition();
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getPosition();
    }

    public boolean areEncodersSynched() {
        return ((getPosition() > (getAbsolutePosition() - 0.0005)) && (getPosition() < (getAbsolutePosition() + 0.0005)));
    }

    public void resetPosition(double position) {
        positionEncoder.setPosition(absoluteEncoder.getPosition());
    }

    // public double getSensorDistance() {
    //     return 15;
    // }

    public void startHandoff() {
        // startTime = Timer.getFPGATimestamp();
        // hasGamePiece = true;
    }

    public boolean hasGamePiece() {
        // return false;
        // if (Timer.getFPGATimestamp() > startTime + travelTime) {
        //     return false;
        // }
        // return true;
        // if (sensorDistance < Constants.Intake.sensorThreshhold && sensorDistance != 0.0) {
        //     return true;
        // }
        // return sensor.get();
        return  rollerMotor.getTorqueCurrent().getValueAsDouble() > 25;
    }

    public double getTorqueCurrent() {
        return rollerMotor.getTorqueCurrent().getValueAsDouble();
    }

    public void reset() {
        goalState = PositionState.STOW;
        lastGoalState = PositionState.STOW;
        lastState = PositionState.STOW;
        movementState = MovementState.MOVING;
    }
}
