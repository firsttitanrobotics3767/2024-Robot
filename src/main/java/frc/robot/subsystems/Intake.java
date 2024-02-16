package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    // Rollers
    private final TalonFX intakeMotor;
    private final PhoenixPIDController rollerPID;

    // Pivot
    private final CANSparkMax positionMotor;
    private final RelativeEncoder positionEncoder;
    private final SparkPIDController positionController;

    public Intake() {
        intakeMotor = new TalonFX(14);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        positionMotor = new CANSparkMax(13, MotorType.kBrushless);
        positionMotor.restoreFactoryDefaults();
        positionMotor.setIdleMode(IdleMode.kBrake);
        positionMotor.setInverted(false);
        
        positionEncoder = positionMotor.getEncoder();
        positionEncoder.setPositionConversionFactor(Constants.Intake.conversionFactor);
        positionEncoder.setVelocityConversionFactor(Constants.Intake.conversionFactor);

        positionController = positionMotor.getPIDController();
        positionController.setP(Constants.Intake.kP);
        positionController.setI(Constants.Intake.kI);
        positionController.setD(Constants.Intake.kD);
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
        //Code
    }
}
