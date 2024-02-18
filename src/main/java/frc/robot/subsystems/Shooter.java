package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Shooter class
 */
public class Shooter extends SubsystemBase{

    private final TalonFX shooterTop;
    private final TalonFX shooterBottom;
    private final TalonFX feeder;

    private final TalonFXConfiguration shooterTopConfig;
    private final TalonFXConfiguration shooterBottomConfig;
    private final TalonFXConfiguration feederConfig;

    private final VelocityVoltage shooterTopVelocityVolt;
    private final VelocityVoltage shooterBottomVelocityVolt;
    private final VelocityVoltage feederVelocityVolt;

    private final CANSparkMax rotationMotor;
    private final AbsoluteEncoder rotationEncoder;
    private final SparkPIDController rotationController;

    public Shooter() {
        shooterTop = new TalonFX(Constants.Shooter.topCANID);
        shooterTop.setNeutralMode(NeutralModeValue.Coast);
        
        shooterBottom = new TalonFX(Constants.Shooter.bottomCANID);
        shooterBottom.setNeutralMode(NeutralModeValue.Coast);

        feeder = new TalonFX(Constants.Shooter.feederCANID);
        feeder.setNeutralMode(NeutralModeValue.Coast);

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

        rotationMotor = new CANSparkMax(Constants.Shooter.rotationCANID, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setInverted(false);

        rotationEncoder = rotationMotor.getAbsoluteEncoder(Type.kDutyCycle);
        rotationEncoder.setPositionConversionFactor(Constants.Shooter.conversionFactor);
        rotationEncoder.setVelocityConversionFactor(Constants.Shooter.conversionFactor);

        rotationController = rotationMotor.getPIDController();
        rotationController.setP(Constants.Shooter.rotationP);
        rotationController.setI(Constants.Shooter.rotationI);
        rotationController.setD(Constants.Shooter.rotationD);
        rotationController.setFF(Constants.Shooter.rotationFF);
        rotationController.setSmartMotionMaxAccel(Constants.Shooter.maxAcc, 0);
        rotationController.setSmartMotionMaxVelocity(Constants.Shooter.maxVel, 0);


    }

    /**
     * Set the rotational position of the shooter
     * @param position double of the anglular position
     */
    public void setPos(double position) {
        if (isPoseValid(position)) {
            rotationController.setReference(Units.degreesToRotations(position), ControlType.kSmartMotion);
        }
    }

    /**
     * Set the velocity of the shooter
     * @param speed double of the velocity
     */
    public void setShootSpeed(double speed) {
        shooterBottom.setControl(shooterBottomVelocityVolt.withVelocity(speed));
        shooterTop.setControl(shooterTopVelocityVolt.withVelocity(speed));
    }

    /**
     * Set the velocity of the feeder
     * @param speed double of the velocity
     */
    public void setFeederSpeed(double speed) {
        feeder.setControl(feederVelocityVolt.withVelocity(speed));
    }

    private boolean isPoseValid(double position) {
        return true;
    }
}
