package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private static Elevator instance = null;
    
    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final SparkPIDController pidController;

    private double targetPos = Constants.Elevator.defaultPosition;

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
        elevatorMotor.setInverted(true);

        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(Constants.Elevator.conversionFactor);
        elevatorEncoder.setVelocityConversionFactor(Constants.Elevator.conversionFactor);

        pidController = elevatorMotor.getPIDController();
        pidController.setP(Constants.Elevator.kP);
        pidController.setI(Constants.Elevator.kI);
        pidController.setD(Constants.Elevator.kD);
        pidController.setSmartMotionMaxAccel(Constants.Elevator.maxAccel, 0);
        pidController.setSmartMotionMaxVelocity(Constants.Elevator.maxVel, 0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator/targetPos", targetPos);
        SmartDashboard.putNumber("elevator/measuredPos", elevatorEncoder.getPosition());
        SmartDashboard.putNumber("elevator/velocity", elevatorEncoder.getVelocity());

        pidController.setReference(targetPos, ControlType.kSmartMotion, 0, Constants.Elevator.gravityFFVolts);
    }

    public void moveTo(double pos) {
        targetPos = pos;
    }
}
