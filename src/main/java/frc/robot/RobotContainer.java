// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IO;
import frc.robot.commands.Drivetrain.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.PathBuilder;

public class RobotContainer {

  public final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Elevator elevator = Elevator.getInstance();
  private final Shooter shooter = Shooter.getInstance();

  CommandJoystick driver = new CommandJoystick(0);
  CommandJoystick operator = new CommandJoystick(1);

  SendableChooser<Command> autoChooser;

  Drivetrain.FieldLocation faceLocation = Drivetrain.FieldLocation.SPEAKER;

  public RobotContainer() {
    configureBindings();

    // drivetrain.setDefaultCommand(new TeleopDrive(
    //   drivetrain,
    //   () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveXAxis), Constants.IO.swerveDeadband),
    //   () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveYAxis), Constants.IO.swerveDeadband),
    //   () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveOmegaAxis), Constants.IO.swerveDeadband),
    //   () -> !driver.button(IO.driveModeButton).getAsBoolean(),
    //   () -> faceLocation
    // ));
    
    intake.setDefaultCommand(new RunCommand(() -> intake.setIntakeSpeed(operator.getRawAxis(1)), intake));
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setPositionSpeed(operator.getRawAxis(5)), shooter));

    autoChooser = AutoBuilder.buildAutoChooser();

    PathBuilder.setupQuestions();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  private void configureBindings() {
    driver.button(IO.resetGyroButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
    driver.button(IO.resetOdometryButton).onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())));
    driver.button(IO.faceSpeakerButton).onTrue(new InstantCommand(() -> faceLocation = Drivetrain.FieldLocation.SPEAKER));
    driver.button(IO.faceSpeakerButton).onFalse(new InstantCommand(() -> faceLocation = Drivetrain.FieldLocation.NONE));

    operator.button(IO.resetIntakePosition).onTrue(new InstantCommand(() -> intake.setPosition(0)));
    operator.button(IO.testIntakePosition).onTrue(new InstantCommand(() -> {intake.moveTo(0.02); intake.setOpenLoopControl(false);}));
    // operator.button(IO.testIntakePosition).onFalse(new InstantCommand(() -> {intake.setOpenLoopControl(true);}));
    operator.button(IO.testStowPosition).onTrue(new InstantCommand(() -> {intake.moveTo(0.25); intake.setOpenLoopControl(false);}));
    // operator.button(IO.testStowPosition).onFalse(new InstantCommand(() -> {intake.setOpenLoopControl(true);}));
    operator.button(1).onTrue(new InstantCommand(() -> intake.setOpenLoopControl(true)));
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return PathBuilder.getFullPathCommand();
  }
}
