package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SensorsSubsystem;
import com.revrobotics.CANSparkMax;

public class neoMotor extends SubsystemBase {

    private CANSparkMax sparkMax;
    private SensorsSubsystem sensor = new SensorsSubsystem();
    private boolean dead = false;

    public neoMotor (CANSparkMax sparkMax) {
        this.sparkMax = sparkMax;
    }

    public void setSpeed (double speed) {
        if (!dead) {
        sparkMax.set(speed);
        } else {
            sparkMax.set(0);
        }

        if (sensor.dataPublish()) {
            dead = true;
        }
    }

}
