package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        leftMotor.set(targetSpeed);
        rightMotor.set(targetSpeed);
        SmartDashboard.putNumber("Top Velocity", leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Velocity", rightMotor.getVelocity().getValueAsDouble());
    }

    public void on() {
        targetSpeed = speedControl.get();
    }

    public void off() {
        targetSpeed = 0;
    }
}
