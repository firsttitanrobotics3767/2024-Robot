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
import frc.robot.commands.Drivetrain.TeleopDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SystemState;
import frc.robot.utils.Constants;
import frc.robot.utils.PathBuilder;
import frc.robot.utils.Constants.IO;
import frc.robot.subsystems.NetworkTables;

public class RobotContainer {

  public final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Climber climber = Climber.getInstance();

  private final Superstructure superstructure = new Superstructure();

  CommandJoystick driver = new CommandJoystick(0);
  CommandJoystick operator = new CommandJoystick(1);
  NetworkTables networkTables = new NetworkTables();

  SendableChooser<Command> autoChooser;

  Drivetrain.FieldLocation faceLocation = Drivetrain.FieldLocation.NONE;

  public RobotContainer() {
    configureBindings();
    networkTables.reader();
    // networkTables.heartbeatReader();

    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveXAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveYAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveOmegaAxis), Constants.IO.swerveDeadband),
      () -> !driver.button(IO.driveModeButton).getAsBoolean(),
      () -> faceLocation
    ));
    
    // intake.setDefaultCommand(new RunCommand(() -> intake.setPositionSpeed(operator.getRawAxis(1)), intake));
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.setPositionSpeed(operator.getRawAxis(5)), shooter));
    // climber.setDefaultCommand(new RunCommand(() -> climber.setArmSpeed(operator.getRawAxis(2)), climber));
    elevator.setDefaultCommand(new RunCommand(() -> elevator.setSpeed(operator.getRawAxis(2)), elevator));
    autoChooser = AutoBuilder.buildAutoChooser();

    PathBuilder.setupQuestions();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  private void configureBindings() {
    driver.button(IO.resetGyroButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
    driver.button(IO.resetOdometryButton).onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())));
    // driver.button(IO.faceSpeakerButton).onTrue(new InstantCommand(() -> faceLocation = Drivetrain.FieldLocation.SPEAKER));
    // driver.button(IO.faceSpeakerButton).onFalse(new InstantCommand(() -> faceLocation = Drivetrain.FieldLocation.NONE));

    driver.button(IO.resetIntakePositionButton).onTrue(new InstantCommand(() -> {shooter.resetPosition(0); intake.resetPosition(0);}));
    driver.button(IO.intakeButton).whileTrue(new InstantCommand(() -> superstructure.setGoalState(SystemState.INTAKE))).onFalse(new InstantCommand(() -> superstructure.setGoalState(SystemState.STOW)));
    driver.button(5).whileTrue(new InstantCommand(() -> superstructure.setGoalState(SystemState.PREPARE_SPEAKER))).onFalse(new InstantCommand(() -> superstructure.setGoalState(SystemState.STOW)));
    operator.button(1).onTrue(new InstantCommand(() -> {shooter.setControlState(Shooter.ControlState.MANUAL); intake.setControlState(Intake.ControlState.MANUAL);}));
    operator.button(3).onTrue(new InstantCommand(() -> {shooter.setControlState(Shooter.ControlState.AUTOMATIC); intake.setControlState(Intake.ControlState.AUTOMATIC);}));
    driver.button(9).whileTrue(new InstantCommand(() -> {shooter.setFeederSpeed(0);})).onFalse(new InstantCommand(() -> {shooter.setFeederSpeed(0.0);}));
    driver.button(14).whileTrue(new InstantCommand(() -> {shooter.setFeederSpeed(0.3); shooter.setShootSpeed(80);})).onFalse(new InstantCommand(() -> {shooter.setFeederSpeed(0.0); shooter.setShootSpeed(0);}));
    driver.button(6).onTrue(new InstantCommand(() -> {intake.startHandoff(); shooter.startHandoff();}));
    driver.button(10).onTrue(new InstantCommand(() -> superstructure.reset()));
    
  }


  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return PathBuilder.getFullPathCommand();
  }
}
