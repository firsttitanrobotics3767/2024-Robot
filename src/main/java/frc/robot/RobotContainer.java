// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IO;
import frc.robot.commands.Drivetrain.TeleopDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();

  CommandJoystick driver = new CommandJoystick(0);

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> driver.getRawAxis(IO.driveXAxis),
      () -> -driver.getRawAxis(IO.driveYAxis),
      () -> driver.getRawAxis(IO.driveOmegaAxis),
      () -> driver.button(IO.driveModeButton).getAsBoolean()));
    

    autoChooser = AutoBuilder.buildAutoChooser();
  }


  private void configureBindings() {
    driver.button(IO.resetGyroButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
