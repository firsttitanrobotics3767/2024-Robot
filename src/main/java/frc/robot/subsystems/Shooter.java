package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Constants;

public class Shooter extends SubsystemBase{
    public enum ControlState {
        AUTOMATIC,
        MANUAL;
    }

    public enum PositionState {
        IDLE(0.045),
        // SHOOT(0.1),
        SHOOT(-0.02),
        AUTO(0),
        SCORE_3(0.05),
        SIDE_SCORE(-0.01),
        AMP(0.25),
        HANDOFF(0.055),
        PASS(0.0),
        CLIMB(0.25);

        public double pos;
        private PositionState(double pos) {
            this.pos = pos;
        }
    }

    private static InterpolatingDoubleTreeMap shotAngle = new InterpolatingDoubleTreeMap();

    static {
        shotAngle.put(1.55, 0.00);
        shotAngle.put(2.0, 0.018);
        shotAngle.put(2.5, 0.04);
        shotAngle.put(3.0, 0.05);
        shotAngle.put(3.5, 0.056);
        shotAngle.put(4.0, 0.062);
        shotAngle.put(4.5, 0.069);
        shotAngle.put(5.0, .05);
    }

    private static Translation2d speaker = new Translation2d(0, 5.50);

    private static Shooter instance = null;
    private ControlState controlState = ControlState.AUTOMATIC;
    private PositionState lastState = PositionState.HANDOFF;
    private PositionState goalState = PositionState.SHOOT;
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
    private final TalonFXConfiguration positionConfig;

    private final VelocityVoltage shooterTopVelocityVolt;
    private final VelocityVoltage shooterBottomVelocityVolt;

    private final TalonFX positionMotor;
    // private final RelativeEncoder positionEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    private final MotionMagicVoltage positionController;

    private final DigitalInput sensor = new DigitalInput(0);


    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }

        return instance;
    }

    // SysIdRoutine routine = new SysIdRoutine(new Config(Units.Volts.of(0.1).per(1), Units.Volts.of(1)), );
    

    public Shooter() {
        shooterTop = new TalonFX(Constants.Shooter.topCANID);
        shooterTop.setNeutralMode(NeutralModeValue.Brake);
        shooterTop.setInverted(true);
        
        shooterBottom = new TalonFX(Constants.Shooter.bottomCANID);
        shooterBottom.setNeutralMode(NeutralModeValue.Brake);
        shooterBottom.setInverted(true);

        feeder = new TalonFX(Constants.Shooter.feederCANID);
        feeder.setNeutralMode(NeutralModeValue.Brake);
        feeder.setInverted(false);

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

        positionMotor = new TalonFX(Constants.Shooter.rotationCANID);

        absoluteEncoder = new DutyCycleEncoder(2);
        absoluteEncoder.setDistancePerRotation(Constants.Shooter.absoluteConversionFactor);
        
        // positionController = new ProfiledPIDController(
        //     Constants.Shooter.positionP,
        //     Constants.Shooter.positionI,
        //     Constants.Shooter.positionD,
        //     new Constraints(Constants.Shooter.maxVel, Constants.Shooter.maxAcc)
        // );

        positionConfig = new TalonFXConfiguration();
        positionConfig.Slot0.kP = Constants.Shooter.positionP;
        positionConfig.Slot0.kI = Constants.Shooter.positionI;
        positionConfig.Slot0.kD = Constants.Shooter.positionD;
        positionConfig.Slot0.kG = Constants.Shooter.positionG;
        positionConfig.Slot0.kV = Constants.Shooter.positionV;
        positionConfig.Slot0.kS = Constants.Shooter.positionS;
        positionConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        positionConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.conversionFactor;
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.maxVel;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.maxAcc;

        positionMotor.getConfigurator().apply(positionConfig.Slot0);
        positionMotor.getConfigurator().apply(motionMagicConfigs);
        positionMotor.getConfigurator().apply(positionConfig.Feedback);

        positionController = new MotionMagicVoltage(goalState.pos);
        
        positionMotor.setNeutralMode(NeutralModeValue.Brake);
        positionMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        if (!areEncodersSynched && Timer.getFPGATimestamp() > 12) {
            resetPosition();
            areEncodersSynched = areEncodersSynched();
            System.out.println("out of sync : " + getPosition() + " | " + getAbsolutePosition());
            SmartDashboard.putBoolean("Shooter/resetEncoder", areEncodersSynched ? true : false);
        }

        // if (lastGoalState == PositionState.HANDOFF && goalState != PositionState.HANDOFF) {
        //     // resetPosition();
        //     System.out.println("reset");
        // }

        if (controlState == ControlState.AUTOMATIC) {
           if (goalState == PositionState.AUTO) {
               positionMotor.setControl(new MotionMagicVoltage(getEstimatedShotAngle(DriverStation.getAlliance().get())));
           } else {
               positionMotor.setControl(new MotionMagicVoltage(goalState.pos));
            //    positionMotor.setPosition(getAbsolutePosition());
           }
        }// } else if (controlState == ControlState.AUTOMATIC) {
        //     positionMotor.set(targetOpenLoopOutput + (Math.cos((getPosition() - 0.05) * Math.PI * 2.0) * Constants.Shooter.positionFF));
        //     // setShootSpeed(80);
        // } else {
        //     positionMotor.set(targetOpenLoopOutput + (Math.cos((getPosition() - 0.05) * Math.PI * 2.0) * Constants.Shooter.positionFF));
        // }

        // if (controlState == ControlState.AUTOMATIC) {
        //     // positionController.Position = goalState.pos;
        //     positionMotor.setControl(new MotionMagicVoltage(goalState.pos));
        //     // double controllerOutput = positionController.calculate(getAbsolutePosition(), goalState.pos);
        //     // positionMotor.setVoltage(controllerOutput + Math.cos(getAbsolutePosition() * Constants.Shooter.positionG) + (Math.signum(controllerOutput) * Constants.Shooter.positionS));
        // } 

        feeder.set(targetFeederSpeed);

        lastState = atGoal() ? goalState : lastState;
        lastGoalState = goalState;

        SmartDashboard.putNumber("Shooter/distanceToSpeaker", Drivetrain.getInstance().getPose().getTranslation().getDistance((DriverStation.getAlliance().get() == Alliance.Blue) ? Constants.FieldLocations.blueSpeaker : Constants.FieldLocations.redSpeaker));
        SmartDashboard.putNumber("Shooter/measuredPosition", getPosition());
        SmartDashboard.putNumber("Shooter/absolute Position", getAbsolutePosition());
        SmartDashboard.putNumber("Shooter/positionVolts", positionMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/atGoal", atGoal());
        SmartDashboard.putString("Shooter/goalState", goalState.toString());
        SmartDashboard.putString("Shooter/lastState", lastState.toString());
        SmartDashboard.putNumber("Shooter/voltage", positionMotor.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/voltage", shooterBottom.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/velocity", shooterBottom.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/target velocity", targetSpeed);
        SmartDashboard.putNumber("Shooter/velocity", getWheelSpeed());
        SmartDashboard.putBoolean("Shooter/sensor", hasGamePiece());
        SmartDashboard.putBoolean("Shooter/encodersSynched", areEncodersSynched());
        SmartDashboard.putBoolean("Shooter/hasGamePiece", hasGamePiece());
        // SmartDashboard.putNumber("Shooter/target velocity", targetSpeed);
    }

    public double getEstimatedShotAngle(Alliance alliance) {
        return shotAngle.get(Drivetrain.getInstance().getPose().getTranslation().getDistance((alliance == Alliance.Blue) ? Constants.FieldLocations.blueSpeaker : Constants.FieldLocations.redSpeaker));
    }
    
    public void setShootSpeed(double speed) {
        this.targetSpeed = speed;
        shooterBottom.setControl(shooterBottomVelocityVolt.withVelocity(speed));
        shooterTop.setControl(shooterTopVelocityVolt.withVelocity(speed - 1));
    }

    public void setFeederSpeed(double speed) {
        targetFeederSpeed = speed;
    }

    public void setPositionSpeed(double speed) {
        positionMotor.set(speed);
    }

    public void moveTo(PositionState positionState) {
        goalState = positionState;
        positionMotor.setPosition(getAbsolutePosition());
    }

    public void setControlState(ControlState controlState) {
        this.controlState = controlState;
    }

    public boolean atGoal() {
        if (goalState == PositionState.AUTO) { 
            return ((getAbsolutePosition() > (getEstimatedShotAngle(DriverStation.getAlliance().get()) - 0.01) && (getAbsolutePosition() > (getEstimatedShotAngle(DriverStation.getAlliance().get()) + 0.01))));
        } else {
            return ((getAbsolutePosition() > (goalState.pos - 0.01)) && (getAbsolutePosition() < (goalState.pos + 0.01)));
        }
    }
    
    public double getPosition() {
        return positionMotor.getPosition().getValueAsDouble();
    }

    public double getAbsolutePosition() {
        return -(absoluteEncoder.getAbsolutePosition() - Constants.Shooter.absoluteOffset);
    }

    public void resetPosition() {
        positionMotor.setPosition(getAbsolutePosition());
    }

    public boolean areEncodersSynched() {
        return ((getPosition() > (getAbsolutePosition() - 0.0005)) && (getPosition() < (getAbsolutePosition() + 0.0005)));
    }

    public void startHandoff() {
        hasGamePiece = true;
    }

    public boolean hasGamePiece() {
        // return (feeder.getTorqueCurrent().getValueAsDouble() > 10);
        return !sensor.get();
    }

    public double getWheelSpeed() {
        return shooterBottom.getVelocity().getValueAsDouble();
    }

    public void reset() {
        goalState = PositionState.HANDOFF;
        lastGoalState = PositionState.HANDOFF;
        lastState = PositionState.HANDOFF;
    }



}
