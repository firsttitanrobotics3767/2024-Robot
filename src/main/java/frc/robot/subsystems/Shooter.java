package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Dashboard.Entry;

public class Shooter extends SubsystemBase{
    private final TalonFX leftMotor = new TalonFX(6);
    private final TalonFX rightMotor = new TalonFX(7);
    private final Entry<Double> speedControl = Entry.getDoubleEntry("Shooter Speed", 0);
    private double targetSpeed = 0;

    public Shooter() {
        leftMotor.getConfigurator().apply(new TalonFXConfiguration());
        rightMotor.getConfigurator().apply(new TalonFXConfiguration());
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        leftMotor.set(targetSpeed);
        rightMotor.set(targetSpeed);
    }

    public void on() {
        targetSpeed = speedControl.get();
    }

    public void off() {
        targetSpeed = 0;
    }
}
