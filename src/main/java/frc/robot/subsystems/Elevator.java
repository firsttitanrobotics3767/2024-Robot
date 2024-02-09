package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private double targetPosition = 0;

    public Elevator() {
        // Left motor setup
        elevatorMotor = new CANSparkMax(5, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        elevatorMotor.setInverted(true);
        

        // Encoder setup
        elevatorEncoder = elevatorMotor.getEncoder();

    }

    @Override
    public void periodic() {}

    public void set(double speed) {}
}
