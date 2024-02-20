package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final AbsoluteEncoder encoder;

    private final SparkPIDController pidController;

    public Climber() {
        leftMotor = new CANSparkMax(Constants.Climber.leftMotorCANID, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setInverted(false);

        rightMotor = new CANSparkMax(Constants.Climber.rightMotorCANID, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor);

        encoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(Constants.Climber.conversionfactor);
        encoder.setVelocityConversionFactor(Constants.Climber.conversionfactor);

        pidController = leftMotor.getPIDController();
        pidController.setFeedbackDevice(encoder);
        pidController.setP(Constants.Climber.kP);
        pidController.setI(Constants.Climber.kI);
        pidController.setD(Constants.Climber.kD);
        pidController.setSmartMotionMaxAccel(Constants.Climber.maxAccel, 0);
        pidController.setSmartMotionMaxVelocity(Constants.Climber.maxVel, 0);

    }

    public void setPosition(double position) {
        pidController.setReference(position, ControlType.kSmartMotion);
    }

}
