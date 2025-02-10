package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivetrain extends SubsystemBase {

private CANSparkMax rightFront, rightBack, leftFront, leftBack;

    public drivetrain() {

        rightFront = new CANSparkMax(1, MotorType.kBrushless);
        rightFront.restoreFactoryDefaults();
        rightFront.setIdleMode(IdleMode.kBrake);

        rightBack = new CANSparkMax(2, MotorType.kBrushless);
        rightBack.restoreFactoryDefaults();
        rightBack.setIdleMode(IdleMode.kBrake);

        leftFront = new CANSparkMax(3, MotorType.kBrushless);
        leftFront.restoreFactoryDefaults();
        leftFront.setIdleMode(IdleMode.kBrake);

        leftBack = new CANSparkMax(4, MotorType.kBrushless);
        leftBack.restoreFactoryDefaults();
        leftBack.setIdleMode(IdleMode.kBrake);

        rightFront.setInverted(true);
        rightBack.setInverted(true);

        rightBack.follow(rightFront);
        leftBack.follow(leftFront);

    }
    public void setRightSpeed(double speed) {
        rightFront.set(speed);

    }

    public void setLeftSpeed(double speed) {
        leftFront.set(speed);
    }
}