package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Elevator extends SubsystemBase{
    public enum PositionState {
        STOW(0),
        AMP(6.5),
        SAFE_SHOOT(10);

        public double pos;
        private PositionState(double pos) {
            this.pos = pos;
        }
    }

    private static Elevator instance = null;
    
    private final TalonFX elevatorMotor;
    private final TalonFXConfiguration elevatorConfig;

    private PositionState goalState = PositionState.STOW;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }
    
    public Elevator() {
        // elevatorMotor = new CANSparkMax(Constants.Elevator.motorID, MotorType.kBrushless);
        // elevatorMotor.restoreFactoryDefaults();
        // elevatorMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        // elevatorMotor.setInverted(false);
        // elevatorMotor.setSmartCurrentLimit(80);
        // elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        // elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        // elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 8);
        // elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0);

        // elevatorEncoder = elevatorMotor.getEncoder();
        // elevatorEncoder.setPositionConversionFactor(Constants.Elevator.conversionFactor);
        // elevatorEncoder.setVelocityConversionFactor(Constants.Elevator.conversionFactor / 60);

        // pidController = elevatorMotor.getPIDController();
        // pidController.setP(Constants.Elevator.kP);
        // pidController.setI(Constants.Elevator.kI);
        // pidController.setD(Constants.Elevator.kD);
        // pidController.setSmartMotionMaxAccel(Constants.Elevator.maxAccel, 0);
        // pidController.setSmartMotionMaxVelocity(Constants.Elevator.maxVel, 0);
        elevatorMotor = new TalonFX(Constants.Elevator.motorID);
        elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.2;
        elevatorMotor.getConfigurator().apply(elevatorConfig);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setInverted(false);
        elevatorMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Elevator/goalState", goalState.toString());
        SmartDashboard.putNumber("Elevator/targetPos", goalState.pos);
        SmartDashboard.putNumber("Elevator/measuredPos", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/velocity", elevatorMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator/at goal", atGoal());


        // pidController.setReference(goalState.pos, ControlType.kSmartMotion, 0, 2.1);
    }

    public void moveTo(PositionState position) {
        goalState = position;
    }

    public void setSpeed(double speed) {
        elevatorMotor.set(speed * 0.5);
    }

    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public void resetPosition() {
        elevatorMotor.setPosition(0);
    }

    public boolean atGoal() {
        return ((getPosition() > (goalState.pos - 0.01)) && (getPosition() < (goalState.pos + 0.01)));
    }
}
