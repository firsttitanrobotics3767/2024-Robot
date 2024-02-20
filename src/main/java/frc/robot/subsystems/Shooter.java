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
 * Shooter class
 */
public class Shooter extends SubsystemBase{
    private static Shooter instance = null;
    private boolean positionOpenLoopControl = true;
    private double positionOpenLoopOutput = 0;
    private double targetPosition = Superstructure.ShooterState.IDLE.pos;

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
    private final SparkPIDController positionController;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }

        return instance;
    }

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

        positionMotor = new CANSparkMax(Constants.Shooter.rotationCANID, MotorType.kBrushless);
        positionMotor.restoreFactoryDefaults();
        positionMotor.setIdleMode(IdleMode.kBrake);
        positionMotor.setInverted(true);

        positionEncoder = positionMotor.getEncoder();
        positionEncoder.setPositionConversionFactor(Constants.Shooter.conversionFactor);
        positionEncoder.setVelocityConversionFactor(Constants.Shooter.conversionFactor);

        positionController = positionMotor.getPIDController();
        positionController.setP(Constants.Shooter.positionP);
        positionController.setI(Constants.Shooter.positionI);
        positionController.setD(Constants.Shooter.positionD);
        positionController.setSmartMotionMaxAccel(Constants.Shooter.maxAcc, 0);
        positionController.setSmartMotionMaxVelocity(Constants.Shooter.maxVel, 0);
        positionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        positionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        positionMotor.setSoftLimit(SoftLimitDirection.kForward, (float)0.3);
        positionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.01);


    }

    @Override
    public void periodic() {
        if (!positionOpenLoopControl) {
            positionController.setReference(targetPosition, ControlType.kSmartMotion, 0,
                Math.cos((getPosition() - 0.05) * Math.PI * 2.0) * Constants.Shooter.positionFF);
        } else {
            positionMotor.set(positionOpenLoopOutput);
        }

        SmartDashboard.putNumber("Shooter/measuredPosition", getPosition());
    }

    /**
     * Set the rotational position of the shooter
     * @param position double of the anglular position
     */
    public void moveTo(double position) {
        targetPosition = position;
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

    public void setPositionSpeed(double speed) {
        positionOpenLoopOutput = speed * 0.2;
    }

    public void resetPosition(double newPosition) {
        positionEncoder.setPosition(newPosition);
    }

    public double getPosition() {
        return positionEncoder.getPosition();
    }

    public void setOpenLoopControl(boolean controlMode) {
        this.positionOpenLoopControl = controlMode;
    }
}
