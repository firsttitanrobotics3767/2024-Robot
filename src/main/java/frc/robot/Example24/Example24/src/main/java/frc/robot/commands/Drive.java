package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain;

public class Drive extends Command {
    private final drivetrain Drivetrain;
    private final Supplier<Double> rightSpeed, leftSpeed;

    public Drive(Supplier<Double> left, Supplier<Double> right, drivetrain Drivetrain) {
        this.leftSpeed = left;
        this.rightSpeed = right;
        this.Drivetrain = Drivetrain;

        addRequirements (Drivetrain);

    }

    @Override
    public void execute() {
        Drivetrain.setLeftSpeed(leftSpeed.get());
        Drivetrain.setRightSpeed(rightSpeed.get());
    }
     @Override
     public void end(boolean isInterrupted) {
        Drivetrain.setLeftSpeed(0);
        Drivetrain.setRightSpeed(0);
     }

     @Override
     public boolean isFinished(){
        return false;
     }
}