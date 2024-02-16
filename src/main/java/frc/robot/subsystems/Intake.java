package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake class
 */
public class Intake extends SubsystemBase{
    private static Intake instance = null;
    
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private double positionSetpoint = 0.0;

    // Rollers
    private final TalonFX intakeMotor;
    private final TalonFXConfiguration rollerConfigs;
    private final PositionVoltage voltageRequest;

    // Pivot
    private final CANSparkMax positionMotor;
    private final AbsoluteEncoder positionEncoder;
    private final SparkPIDController positionController;

    public Intake() {
        intakeMotor = new TalonFX(Constants.Intake.rollerCANID);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        rollerConfigs =  new TalonFXConfiguration();
        rollerConfigs.Slot0.kP = 0;
        rollerConfigs.Slot0.kI = 0;
        rollerConfigs.Slot0.kD = 0;
        rollerConfigs.Slot0.kV = 0;
        intakeMotor.getConfigurator().apply(rollerConfigs);

        voltageRequest = new PositionVoltage(0).withSlot(0);

        positionMotor = new CANSparkMax(Constants.Intake.positionCANID, MotorType.kBrushless);
        positionMotor.restoreFactoryDefaults();
        positionMotor.setIdleMode(IdleMode.kBrake);
        positionMotor.setInverted(false);
        
        positionEncoder = positionMotor.getAbsoluteEncoder(Type.kDutyCycle);
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

    /**
     * Set the velocity of the intake
     * @param speed double of the velocity
     */
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Set the position of the intake
     * @param position double of the rotational position
     */
    public void setPos(double position) {
        if (setpointIsValid(position)) {
            positionController.setReference(Units.degreesToRotations(position), ControlType.kSmartMotion);
        }
    }

    public void controlPos(double volts) {
        positionMotor.set(volts);
    }

    private boolean setpointIsValid(Double setpoint) {
        return true;
    }
}
