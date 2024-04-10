package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
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
        SCORING(0.1),
        STOW(0.23);

        public double pos;
        private PositionState(double pos) {
            this.pos = pos;
        }
    }

    private static Intake instance = null;
    private ControlState controlState = ControlState.AUTOMATIC;
    private PositionState goalState = PositionState.STOW;
    private PositionState lastState = PositionState.STOW;
    private PositionState lastGoalState = PositionState.STOW;
    private double targetOpenLoopOutput = 0;
    private double targetRollerSpeed = 0;
    private boolean areEncodersSynched = false;
    private boolean hasGamePiece = false; 
    private boolean flashLights = false;
    private boolean lightsOn = true;
    private int t = 0;
    private double startTime = 0; 
    private double runTime = 1; 
    private int runSpeed = 4;
    
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private final TalonFX rollerMotor;
    private final TalonFXConfiguration rollerConfig, positionConfig;

    private final AnalogInput analogSensor = new AnalogInput(3);
    private final DigitalInput digitalSensor = new DigitalInput(3);
    private final PWM lights = new PWM(1);

    private final TalonFX positionLeftMotor, positionRightMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public Intake() {
        rollerMotor = new TalonFX(Constants.Intake.rollerCANID);
        rollerConfig = new TalonFXConfiguration();
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setInverted(false);

        absoluteEncoder = new DutyCycleEncoder(1);
        absoluteEncoder.setDistancePerRotation(Constants.Intake.absoluteConversionFactor);

        positionLeftMotor = new TalonFX(Constants.Intake.positionLeftCANID);
        positionConfig = new TalonFXConfiguration();
        positionConfig.Feedback.SensorToMechanismRatio = Constants.Intake.conversionFactor;
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Intake.positionP;
        slot0Configs.kI = Constants.Intake.positionI;
        slot0Configs.kD = Constants.Intake.positionD;
        slot0Configs.kG = Constants.Intake.positionG;
        slot0Configs.kV = Constants.Intake.positionV;
        slot0Configs.kS = Constants.Intake.positionS;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake.maxVel;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Intake.maxAccel;
        positionLeftMotor.getConfigurator().apply(positionConfig);
        positionLeftMotor.getConfigurator().apply(slot0Configs);
        positionLeftMotor.getConfigurator().apply(motionMagicConfigs);
        positionLeftMotor.setInverted(false);
        positionLeftMotor.setNeutralMode(NeutralModeValue.Brake);

        positionRightMotor = new TalonFX(Constants.Intake.positionRightCANID);
        positionRightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        if (!areEncodersSynched && Timer.getFPGATimestamp() > 10) {
            resetPosition();
            areEncodersSynched = areEncodersSynched();
        }

        hasGamePiece = hasGamePiece();

        if (controlState == ControlState.AUTOMATIC) {
            positionLeftMotor.setControl(new MotionMagicVoltage(goalState.pos));
            positionRightMotor.setControl(new Follower(Constants.Intake.positionLeftCANID, true));
        } else {
            positionLeftMotor.setControl(new DutyCycleOut(targetOpenLoopOutput));
            positionRightMotor.setControl(new Follower(Constants.Intake.positionLeftCANID, true));
        }

        rollerMotor.set(targetRollerSpeed);

        lastState = atGoal()  ? goalState : lastState;

        if (flashLights && Timer.getFPGATimestamp() < (startTime + runTime)) {
            t = (t + 1) % runSpeed;
            if (t == 0) {
                lightsOn = !lightsOn;
            }
        } else {
            lightsOn = true;
            flashLights = false;
        }

        
        if (lightsOn) {
            lights.setSpeed(1);
        } else {
            lights.setSpeed(0);
        }

        SmartDashboard.putNumber("Intake/measuredPosition", getPosition());
        SmartDashboard.putNumber("Intake/Absolute Position", getAbsolutePosition());
        SmartDashboard.putNumber("Intake/measuredRotationVelocity", positionLeftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/targetPos", goalState.pos);
        SmartDashboard.putBoolean("Intake/atGoal", atGoal());
        SmartDashboard.putString("Intake/lastState", lastState.toString());
        SmartDashboard.putString("Intake/goalState", goalState.toString());
        SmartDashboard.putBoolean("Intake/hasGamePiece", hasGamePiece);
        SmartDashboard.putNumber("Intake/Sensor Distance", analogSensor.getValue());
        SmartDashboard.putBoolean("Intake/digitalSensor", digitalSensor.get());
        SmartDashboard.putNumber("Intake/roller torque", rollerMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/positionVolts", positionLeftMotor.getMotorVoltage().getValueAsDouble());
    }

    public void setPositionSpeed(double speed) {
        targetOpenLoopOutput = speed * 1;
    }

    public void setRollerSpeed(double speed) {
        targetRollerSpeed = speed;
    }

    public void moveTo(PositionState positionState) {
        goalState = positionState;
    }
    
    public void setControlState(ControlState controlState) {
        this.controlState = controlState;
    }

    public boolean atGoal() {
        // return ((getAbsolutePosition() > (goalState.pos - 0.01)) && (getAbsolutePosition() < (goalState.pos + 0.01)));
        return ((getPosition() > (goalState.pos - 0.01)) && (getPosition() < (goalState.pos + 0.01)));
    }

    public double getPosition() {
        return positionLeftMotor.getPosition().getValueAsDouble();
    }

    public double getAbsolutePosition() {
        // return absoluteEncoder.getAbsolutePosition() - 0;
        return absoluteEncoder.getAbsolutePosition() - Constants.Intake.absoluteOffset;
    }

    public boolean areEncodersSynched() {
        return ((getPosition() > (getAbsolutePosition() - 0.0005)) && (getPosition() < (getAbsolutePosition() + 0.0005)));
    }

    public void resetPosition() {
        positionLeftMotor.setPosition(getAbsolutePosition());
    }

    public boolean hasGamePiece() {
        // return  rollerMotor.getTorqueCurrent().getValueAsDouble() > 30.0;
        return (analogSensor.getValue() < 2700.0) || !digitalSensor.get();
    }

    public double getTorqueCurrent() {
        return rollerMotor.getTorqueCurrent().getValueAsDouble();
    }

    public void reset() {
        goalState = PositionState.STOW;
        lastGoalState = PositionState.STOW;
        lastState = PositionState.STOW;
    }

    public void flashLights(double time, int speed) {
        flashLights = true;
        startTime = Timer.getFPGATimestamp();
        runTime = time;
        runSpeed = speed;
        if (time == 0) {
            flashLights = false;
        } else if (time == -1) {
            startTime = 99999;
        }
    }


}
