package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Dashboard.Entry;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final Entry<Double> speedControl = Entry.getDoubleEntry("Intake Speed", 0);
    private double targetSpeed = 0;

    public Intake() {
        intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        intakeMotor.set(targetSpeed);
    }

    public void on() {
        targetSpeed = speedControl.get();
    }

    public void off() {
        targetSpeed = 0;
    }
}
