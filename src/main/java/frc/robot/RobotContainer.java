// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IO;
import frc.robot.commands.Drivetrain.SupplyElevator;
import frc.robot.commands.Drivetrain.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  public final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();

  CommandJoystick driver = new CommandJoystick(0);

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveXAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveYAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveOmegaAxis), Constants.IO.swerveDeadband),
      () -> !driver.button(IO.driveModeButton).getAsBoolean()));
    

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  private void configureBindings() {
    driver.button(IO.resetGyroButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
    driver.button(3).whileTrue(drivetrain.driveToPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180))));
    driver.povUp().whileTrue(new SupplyElevator(() -> -0.3, elevator));
    driver.povDown().whileTrue(new SupplyElevator(() -> 0.3, elevator));
  }

  public Command getAutonomousCommand() {
    // //return autoChooser.getSelected();
                        
    return new PathPlannerAuto("New Auto");
  }
}
