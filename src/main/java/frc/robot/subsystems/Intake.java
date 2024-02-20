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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake class
 */
public class Intake extends SubsystemBase{
    private static Intake instance = null;
    private boolean openLoopControl = true;
    private double targetOpenLoopOutput = 0;
    private double targetPos = Superstructure.IntakeState.IDLE.pos;
    private double rollerTargetSpeed = 0;
    
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    // Rollers
    private final TalonFX rollerMotor;
    private final TalonFXConfiguration rollerConfig;

    // Pivot
    private final CANSparkMax positionMotor;
    private final RelativeEncoder positionEncoder;
    private final SparkPIDController positionController;

    public Intake() {
        rollerMotor = new TalonFX(Constants.Intake.rollerCANID);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);

        rollerConfig = new TalonFXConfiguration();

        rollerMotor.getConfigurator().apply(rollerConfig);

        positionMotor = new CANSparkMax(Constants.Intake.positionCANID, MotorType.kBrushless);
        positionMotor.restoreFactoryDefaults();
        positionMotor.setIdleMode(IdleMode.kBrake);
        positionMotor.setInverted(false);
        positionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        positionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        positionMotor.setSoftLimit(SoftLimitDirection.kForward, (float)0.26);
        positionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.01);
        positionMotor.setOpenLoopRampRate(Constants.Intake.openLoopRampRate);
        
        positionEncoder = positionMotor.getEncoder();
        positionEncoder.setPositionConversionFactor(Constants.Intake.conversionFactor);
        positionEncoder.setVelocityConversionFactor(Constants.Intake.conversionFactor);

        positionController = positionMotor.getPIDController();
        positionController.setFeedbackDevice(positionEncoder);
        positionController.setP(Constants.Intake.positionP);
        positionController.setI(Constants.Intake.positionI);
        positionController.setD(Constants.Intake.positionD);
        positionController.setSmartMotionMaxAccel(Constants.Intake.maxAccel, 0);
        positionController.setSmartMotionMaxVelocity(Constants.Intake.maxVel, 0);

        
    }

    @Override
    public void periodic() {
        if (!openLoopControl) {
            positionController.setReference(targetPos, ControlType.kSmartMotion, 0, 
                Math.cos(getPosition() * Math.PI * 2.0) * Constants.Intake.positionGravityFF);
        } else {
            positionMotor.set(targetOpenLoopOutput);
        }

        rollerMotor.set(rollerTargetSpeed);

        SmartDashboard.putNumber("Intake/measuredPosition", getPosition());
        SmartDashboard.putNumber("Intake/measuredRotationVelocity", positionEncoder.getVelocity());
        SmartDashboard.putNumber("Intake/measuredRollerVelocity", rollerMotor.getVelocity().getValueAsDouble());
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
        rollerTargetSpeed = speed;
        SmartDashboard.putNumber("Intake/Target Roller Speed", speed);
    }

    /**
     * Set the position of the intake
     * @param position double of the rotational position
     */
    public void moveTo(double position) {
        targetPos = position;
    }

    public boolean atGoal() {
        return true; //check if the intake pid is within acceptable range
    }

    public void setOpenLoopControl(boolean controlType) {
        openLoopControl = controlType;
    }

    public double getPosition() {
        return positionEncoder.getPosition();
    }

    public void resetPosition(double position) {
        positionEncoder.setPosition(position);
    }
}
