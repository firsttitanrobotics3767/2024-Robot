package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final TalonFX leftMotor = new TalonFX(6);
    private final TalonFX rightMotor = new TalonFX(7);

    public Shooter() {
        leftMotor.getConfigurator().apply(new TalonFXConfiguration());
        rightMotor.getConfigurator().apply(new TalonFXConfiguration());
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
    }

    public void set(double speed) {
        leftMotor.set(speed * 0.5);
        rightMotor.set(speed);
    }
}
