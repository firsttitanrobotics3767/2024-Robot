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
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants;

public class Shooter extends SubsystemBase{
    public enum ControlState {
        AUTOMATIC,
        MANUAL;
    }

    public enum PositionState {
        IDLE(0.19),
        SHOOT(0.12),
        SCORE_3(0.167),
        SIDE_SCORE(0.11),
        AMP(0.37),
        HANDOFF(0.175),
        PASS(0.185);

        public double pos;
        private PositionState(double pos) {
            this.pos = pos;
        }
    }

    private static Shooter instance = null;
    private ControlState controlState = ControlState.AUTOMATIC;
    private PositionState lastState = PositionState.HANDOFF;
    private PositionState goalState = PositionState.HANDOFF;
    private PositionState lastGoalState = PositionState.HANDOFF;
    private double targetOpenLoopOutput = 0;
    private double targetSpeed = 0;
    private double targetFeederSpeed = 0;
    public static boolean hasGamePiece = false;
    private boolean areEncodersSynched = false;

    private final TalonFX shooterTop;
    private final TalonFX shooterBottom;
    private final TalonFX feeder;

    private final TalonFXConfiguration shooterTopConfig;
    private final TalonFXConfiguration shooterBottomConfig;
    private final TalonFXConfiguration feederConfig;

    private final VelocityVoltage shooterTopVelocityVolt;
    private final VelocityVoltage shooterBottomVelocityVolt;

    private final CANSparkMax positionMotor;
    private final RelativeEncoder positionEncoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkPIDController positionController;

    private final DigitalInput sensor = new DigitalInput(0);

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

        positionController = positionMotor.getPIDController();
        positionController.setFeedbackDevice(absoluteEncoder);
        positionController.setP(Constants.Shooter.positionP);
        positionController.setI(Constants.Shooter.positionI);
        positionController.setD(Constants.Shooter.positionD);
        positionController.setSmartMotionMaxAccel(Constants.Shooter.maxAcc, 0);
        positionController.setSmartMotionMaxVelocity(Constants.Shooter.maxVel, 0);

        
    }

    @Override
    public void periodic() {
        if (!areEncodersSynched) {
            resetPosition();
            areEncodersSynched = areEncodersSynched();
        }
        if (controlState == ControlState.AUTOMATIC) {
            positionController.setReference(goalState.pos, ControlType.kSmartMotion, 0,
                Math.cos((getPosition() - 0.05) * Math.PI * 2.0) * Constants.Shooter.positionFF);
        } else {
            positionMotor.set(targetOpenLoopOutput + (Math.cos((getPosition() - 0.05) * Math.PI * 2.0) * Constants.Shooter.positionFF));
        }

        feeder.set(targetFeederSpeed);

        lastState = atGoal() ? goalState : lastState;
        lastGoalState = goalState;

        SmartDashboard.putNumber("Shooter/measuredPosition", getPosition());
        SmartDashboard.putNumber("Shooter/absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Shooter/positionCurrent", positionMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Shooter/atGoal", atGoal());
        SmartDashboard.putString("Shooter/goalState", goalState.toString());
        SmartDashboard.putString("Shooter/lastState", lastState.toString());
        SmartDashboard.putNumber("Shooter/voltage", shooterBottom.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/velocity", shooterBottom.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/target velocity", targetSpeed);
    }
    
    public void setShootSpeed(double speed) {
        this.targetSpeed = speed;
        shooterBottom.setControl(shooterBottomVelocityVolt.withVelocity(speed));
        shooterTop.setControl(shooterTopVelocityVolt.withVelocity(speed));
    }

    public void setFeederSpeed(double speed) {
        targetFeederSpeed = speed;
    }

    public void setPositionSpeed(double speed) {
        targetOpenLoopOutput = speed * 0.2;
    }

    public void moveTo(PositionState positionState) {
        if (positionState == PositionState.AMP) {
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
        return ((getAbsolutePosition() > (goalState.pos - 0.01)) && (getAbsolutePosition() < (goalState.pos + 0.01)));
    }
    
    public double getPosition() {
        return positionEncoder.getPosition();
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getPosition();
    }

    public void resetPosition() {
        positionEncoder.setPosition(absoluteEncoder.getPosition());
    }

    public boolean areEncodersSynched() {
        return ((getPosition() > (getAbsolutePosition() - 0.0005)) && (getPosition() < (getAbsolutePosition() + 0.0005)));
    }

    public void startHandoff() {
        hasGamePiece = true;
    }

    public boolean hasGamePiece() {
        // return (feeder.getTorqueCurrent().getValueAsDouble() > 10);
        return sensor.get();
    }

    public void reset() {
        goalState = PositionState.HANDOFF;
        lastGoalState = PositionState.HANDOFF;
        lastState = PositionState.HANDOFF;
    }

}
