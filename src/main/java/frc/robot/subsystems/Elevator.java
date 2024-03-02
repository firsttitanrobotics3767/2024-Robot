package frc.robot.subsystems;

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
    
    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final SparkPIDController pidController;

    private PositionState goalState = PositionState.STOW;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }
    
    public Elevator() {
        elevatorMotor = new CANSparkMax(Constants.Elevator.motorID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        elevatorMotor.setInverted(false);
        elevatorMotor.setSmartCurrentLimit(80);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 6);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0);

        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(Constants.Elevator.conversionFactor);
        elevatorEncoder.setVelocityConversionFactor(Constants.Elevator.conversionFactor / 60);

        pidController = elevatorMotor.getPIDController();
        pidController.setP(Constants.Elevator.kP);
        pidController.setI(Constants.Elevator.kI);
        pidController.setD(Constants.Elevator.kD);
        pidController.setSmartMotionMaxAccel(Constants.Elevator.maxAccel, 0);
        pidController.setSmartMotionMaxVelocity(Constants.Elevator.maxVel, 0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Elevator/goalState", goalState.toString());
        SmartDashboard.putNumber("Elevator/targetPos", goalState.pos);
        SmartDashboard.putNumber("Elevator/measuredPos", elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Elevator/velocity", elevatorEncoder.getVelocity());
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
        return elevatorEncoder.getPosition();
    }

    public void resetPosition() {
        elevatorEncoder.setPosition(0);
    }

    public boolean atGoal() {
        return ((getPosition() > (goalState.pos - 0.01)) && (getPosition() < (goalState.pos + 0.01)));
    }
}
