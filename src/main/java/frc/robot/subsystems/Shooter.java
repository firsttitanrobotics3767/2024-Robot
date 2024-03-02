package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/**
 * Shooter class
 */
public class Shooter extends SubsystemBase{
    public enum ControlState {
        AUTOMATIC,
        MANUAL;
    }

    public enum PositionState {
        IDLE(0.19),
        SHOOT(0.12),
        SCORE_3(0.17),
        SIDE_SCORE(0.11),
        AMP(0.37),
        HANDOFF(0.165);

        public double pos;
        private PositionState(double pos) {
            this.pos = pos;
        }
    }

    public enum MovementState {
        MOVING,
        AT_GOAL;
    }

    private static Shooter instance = null;
    private ControlState controlState = ControlState.AUTOMATIC;
    private MovementState movementState = MovementState.MOVING;
    private PositionState lastState = PositionState.HANDOFF;
    private PositionState goalState = PositionState.HANDOFF;
    private PositionState lastGoalState = PositionState.HANDOFF;
    private double targetOpenLoopOutput = 0;
    private double targetSpeed = 0;
    private double targetFeederSpeed = 0;
    public static boolean hasGamePiece = false;
    private boolean ready = false;
    private double startTime = 0;
    private double travelTime = 1;
    private double feederBackOffTime = 0.75;
    private double sensorDistance = 0;
    private boolean auton = false;

    private final TalonFX shooterTop;
    private final TalonFX shooterBottom;
    private final TalonFX feeder;

    private final TalonFXConfiguration shooterTopConfig;
    private final TalonFXConfiguration shooterBottomConfig;
    private final TalonFXConfiguration feederConfig;

    private final VelocityVoltage shooterTopVelocityVolt;
    private final VelocityVoltage shooterBottomVelocityVolt;
    private final VelocityVoltage feederVelocityVolt;

    private final CANSparkMax positionMotor;
    private final RelativeEncoder positionEncoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkPIDController positionController;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }

        return instance;
    }

    public Shooter() {
        shooterTop = new TalonFX(Constants.Shooter.topCANID);
        shooterTop.setNeutralMode(NeutralModeValue.Brake);
        shooterTop.setInverted(true);
        
        shooterBottom = new TalonFX(Constants.Shooter.bottomCANID);
        shooterBottom.setNeutralMode(NeutralModeValue.Brake);
        shooterBottom.setInverted(true);

        feeder = new TalonFX(Constants.Shooter.feederCANID);
        feeder.setNeutralMode(NeutralModeValue.Brake);
        feeder.setInverted(true);

        shooterTopConfig = new TalonFXConfiguration();
        shooterBottomConfig = new TalonFXConfiguration();
        feederConfig = new TalonFXConfiguration();

        shooterBottomConfig.Slot0.kP = Constants.Shooter.bottomP;
        shooterBottomConfig.Slot0.kI = Constants.Shooter.bottomI;
        shooterBottomConfig.Slot0.kD = Constants.Shooter.bottomD;
        shooterBottomConfig.Slot0.kV = Constants.Shooter.bottomFF;

        shooterTopConfig.Slot0.kP = Constants.Shooter.topP;
        shooterTopConfig.Slot0.kI = Constants.Shooter.topI;
        shooterTopConfig.Slot0.kD = Constants.Shooter.topD;
        shooterTopConfig.Slot0.kV = Constants.Shooter.topFF;

        feederConfig.Slot0.kP = Constants.Shooter.feederP;
        feederConfig.Slot0.kI = Constants.Shooter.feederI;
        feederConfig.Slot0.kD = Constants.Shooter.feederD;
        feederConfig.Slot0.kV = Constants.Shooter.feederFF;

        shooterBottom.getConfigurator().apply(shooterBottomConfig.Slot0);
        shooterTop.getConfigurator().apply(shooterBottomConfig.Slot0);
        feeder.getConfigurator().apply(feederConfig.Slot0);

        shooterBottomVelocityVolt = new VelocityVoltage(0).withSlot(0);
        shooterTopVelocityVolt = new VelocityVoltage(0).withSlot(0);
        feederVelocityVolt = new VelocityVoltage(0).withSlot(0);

        positionMotor = new CANSparkMax(Constants.Shooter.rotationCANID, MotorType.kBrushless);
        positionMotor.restoreFactoryDefaults();
        positionMotor.setIdleMode(IdleMode.kBrake);
        positionMotor.setInverted(true);
        positionMotor.setSmartCurrentLimit(40);

        absoluteEncoder = positionMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setZeroOffset(Constants.Shooter.absoluteOffset);
        absoluteEncoder.setInverted(true);
        
        positionEncoder = positionMotor.getEncoder();
        positionEncoder.setPositionConversionFactor(Constants.Shooter.conversionFactor);
        positionEncoder.setVelocityConversionFactor(Constants.Shooter.conversionFactor);

        try {
            Thread.sleep(500);
        } catch (Exception e) {
            e.printStackTrace();
        }

        positionController = positionMotor.getPIDController();
        positionController.setFeedbackDevice(absoluteEncoder);
        positionController.setP(Constants.Shooter.positionP);
        positionController.setI(Constants.Shooter.positionI);
        positionController.setD(Constants.Shooter.positionD);
        positionController.setSmartMotionMaxAccel(Constants.Shooter.maxAcc, 0);
        positionController.setSmartMotionMaxVelocity(Constants.Shooter.maxVel, 0);
        // positionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        // positionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // positionMotor.setSoftLimit(SoftLimitDirection.kForward, (float)0.35);
        // positionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.01);

        resetPosition(absoluteEncoder.getPosition());
    }

    @Override
    public void periodic() {
        sensorDistance = SmartDashboard.getNumber("Shooter/sensorDistance", 0);
        // if (!auton) {
        //     if (lastState == PositionState.HANDOFF && !hasGamePiece && Superstructure.hasGamePiece) {
        //         setFeederSpeed(0.1);
        //     } else if (lastState == PositionState.HANDOFF && hasGamePiece) {
        //         setFeederSpeed(0);
        //     // } else if (goalState == PositionState.SHOOT && (Timer.getFPGATimestamp() < startTime + feederBackOffTime)) {
        //     //     setFeederSpeed(-0.2);
        //     //     setShootSpeed(-5);
        //     // } else if (goalState == PositionState.SHOOT && (Timer.getFPGATimestamp() > startTime + feederBackOffTime) && !ready) {
        //     //     setFeederSpeed(0);
        //     //     setShootSpeed(80);
        //     //     ready = true;
        //     } else if (goalState == PositionState.SHOOT) {
        //         setShootSpeed(80);
        //     } else if (!(goalState == PositionState.SHOOT)){
        //         setShootSpeed(0);
        //     }
        // }

        hasGamePiece = hasGamePiece();

        if (controlState == ControlState.AUTOMATIC) {

            positionController.setReference(goalState.pos, ControlType.kSmartMotion, 0,
                Math.cos((getPosition() - 0.05) * Math.PI * 2.0) * Constants.Shooter.positionFF);
        } else {
            positionMotor.set(targetOpenLoopOutput + (Math.cos((getPosition() - 0.05) * Math.PI * 2.0) * Constants.Shooter.positionFF));
        }

        feeder.set(targetFeederSpeed);

        movementState = atGoal() ? MovementState.AT_GOAL : MovementState.MOVING;
        lastState = atGoal() ? goalState : lastState;
        lastGoalState = goalState;

        SmartDashboard.putNumber("Shooter/measuredPosition", getPosition());
        SmartDashboard.putNumber("Shooter/absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Shooter/position temp", positionMotor.getMotorTemperature());
        SmartDashboard.putNumber("Shooter/positionCurrent", positionMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Shooter/atGoal", atGoal());
        SmartDashboard.putString("Shooter/goalState", goalState.toString());
        SmartDashboard.putString("Shooter/lastState", lastState.toString());
        SmartDashboard.putBoolean("Shooter/hasGamePiece", hasGamePiece);
        SmartDashboard.putNumber("Shooter/voltage", shooterBottom.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/velocity", shooterBottom.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/target velocity", targetSpeed);
        SmartDashboard.putBoolean("Shooter/ready", ready);
        SmartDashboard.putNumber("Shooter/torque", feeder.getTorqueCurrent().getValueAsDouble());
    }
    
    /**
     * Set the velocity of the shooter
     * @param speed double of the velocity
     */
    public void setShootSpeed(double speed) {
        this.targetSpeed = speed;
        shooterBottom.setControl(shooterBottomVelocityVolt.withVelocity(speed));
        shooterTop.setControl(shooterTopVelocityVolt.withVelocity(speed));
    }

    /**
     * Set the velocity of the feeder
     * @param speed double of the velocity
     */
    public void setFeederSpeed(double speed) {
        targetFeederSpeed = speed;
    }

    public void setPositionSpeed(double speed) {
        targetOpenLoopOutput = speed * 0.2;
    }

    /**
     * Set the rotational position of the shooter
     * @param position double of the anglular position
     */
    public void moveTo(PositionState positionState) {
        if (goalState == PositionState.AMP) {
            resetPosition(absoluteEncoder.getPosition());
            positionController.setFeedbackDevice(absoluteEncoder);
        } else {
            positionController.setFeedbackDevice(positionEncoder);
        }
        goalState = positionState;
    }

    public void setControlState(ControlState controlState) {
        this.controlState = controlState;
    }

    public boolean atGoal() {
        return ((getAbsolutePosition() > (goalState.pos - 0.008)) && (getAbsolutePosition() < (goalState.pos + 0.008)));
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

    public void resetPosition(double newPosition) {
        positionEncoder.setPosition(absoluteEncoder.getPosition());
    }

    public void startHandoff() {
        startTime = Timer.getFPGATimestamp();
        hasGamePiece = true;
    }

    public boolean hasGamePiece() {
        // if (Timer.getFPGATimestamp() > startTime + travelTime) {
        //     return false;
        // }
        // return true;
        // if (sensorDistance < Constants.Shooter.sensorThreshhold) {
        //     return true;
        // }
        return (feeder.getTorqueCurrent().getValueAsDouble() > 10);
    }

    public void reset() {
        goalState = PositionState.IDLE;
        lastGoalState = PositionState.IDLE;
        lastState = PositionState.IDLE;
        movementState = MovementState.MOVING;
    }

    public void setAuton(boolean auton) {
        this.auton = auton;
    }
}
