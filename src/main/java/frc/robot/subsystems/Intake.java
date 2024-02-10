package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax positionMotor;
    private final TalonFX rollerMotor;

    public Intake() {
        positionMotor = new CANSparkMax(13, MotorType.kBrushless);
        positionMotor.setIdleMode(IdleMode.kBrake);

        rollerMotor = new TalonFX(14);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {}

    public void set(double pivotSpeed, double rollerSpeed) {
        positionMotor.set(pivotSpeed);
        rollerMotor.set(rollerSpeed);
    }
}
