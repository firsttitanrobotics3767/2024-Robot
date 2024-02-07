package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    
    private final CANSparkMax leftMotor, rightMotor;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private double targetVolts = 0.0;

    private final double maxHeightMeters = 1;
    private final double minHeightMeters = 0.1;

    public Elevator() {
        // Left motor setup
        leftMotor = new CANSparkMax(5, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        leftMotor.setInverted(true);

        // Right motor setup
        rightMotor = new CANSparkMax(51, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.follow(leftMotor, true);
        

        // Encoder setup
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

    }

    @Override
    public void periodic() {
        leftMotor.set(targetVolts);
    }

    public void setVolts(double volts) {
        targetVolts = volts;
    }
}
