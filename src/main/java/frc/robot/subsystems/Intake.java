package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake class
 */
public class Intake extends SubsystemBase{
    private static Intake instance = null;
    private boolean openLoopControl = true;
    private double targetPos = Superstructure.IntakeState.IDLE.pos;
    private double targetOpenLoopOutput;
    
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    // Rollers
    private final TalonFX intakeMotor;
    // private final PhoenixPIDController rollerPID;

    // Pivot
    private final CANSparkMax positionMotor;
    private final AbsoluteEncoder positionEncoder;
    private final SparkPIDController positionController;

    public Intake() {
        intakeMotor = new TalonFX(Constants.Intake.rollerCANID);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        positionMotor = new CANSparkMax(Constants.Intake.positionCANID, MotorType.kBrushless);
        positionMotor.restoreFactoryDefaults();
        positionMotor.setIdleMode(IdleMode.kBrake);
        positionMotor.setInverted(false);
        
        positionEncoder = positionMotor.getAbsoluteEncoder(Type.kDutyCycle);
        positionEncoder.setPositionConversionFactor(Constants.Intake.conversionFactor);
        positionEncoder.setVelocityConversionFactor(Constants.Intake.conversionFactor);

        positionController = positionMotor.getPIDController();
        positionController.setFeedbackDevice(positionEncoder);
        positionController.setP(Constants.Intake.kP);
        positionController.setI(Constants.Intake.kI);
        positionController.setD(Constants.Intake.kD);
        positionController.setSmartMotionMaxAccel(Constants.Intake.maxAccel, 0);
        positionController.setSmartMotionMaxVelocity(Constants.Intake.maxVel, 0);
    }

    @Override
    public void periodic() {
        if (!openLoopControl) {
            positionController.setReference(targetPos, ControlType.kSmartMotion);
        } else {
            intakeMotor.set(targetOpenLoopOutput);
        }
    }

    /**
     * Set the velocity of the intake
     * @param speed double of the velocity
     */
    public void setIntakeSpeed(double speed) {
        targetOpenLoopOutput = speed;
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
}
