package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax positionMotor;

    public Intake() {
        positionMotor = new CANSparkMax(13, MotorType.kBrushless);
        positionMotor.setInverted(false);
        positionMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {}

    public void set(double speed) {
        positionMotor.set(speed);
    }
}
