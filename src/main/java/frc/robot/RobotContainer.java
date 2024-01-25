// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveXAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveYAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(driver.getRawAxis(IO.driveOmegaAxis), Constants.IO.swerveDeadband),
      () -> !driver.button(IO.driveModeButton).getAsBoolean()));
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  private void configureBindings() {
    driver.button(IO.resetGyroButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
