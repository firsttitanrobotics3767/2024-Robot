package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.neoMotor;

public class setMotorSpeed extends Command{
    private neoMotor motor;
    private double speed;
    private Joystick joystick;

    public setMotorSpeed (neoMotor motor, double speed) {
        this.motor = motor;
        this.speed = speed;
        addRequirements(motor);
    }

    @Override
    public void initialize() {
        motor.setSpeed(speed);
    }

    @Override
    public void execute() {
        motor.setSpeed(joystick.getY());
    }

    @Override
    public void end(boolean interrupted) {
        motor.setSpeed(0);
    }

}
