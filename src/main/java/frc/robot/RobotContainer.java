// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.SetSuperstructureState;
import frc.robot.commands.Shoot;
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
import frc.robot.utils.Constants.Sensors;
import frc.robot.subsystems.SensorSubsystem;

public class RobotContainer {

  public final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  public final Shooter shooter = Shooter.getInstance();
  private final Climber climber = Climber.getInstance();
  private final SensorSubsystem sensors = SensorSubsystem.getInstance();

  private final Superstructure superstructure = new Superstructure();

  CommandJoystick driver = new CommandJoystick(0);
  // PS5Controller driverRumble = new PS5Controller(0);
  // GenericHID driverRumble = new GenericHID(0);
  CommandJoystick operator = new CommandJoystick(1);

  SendableChooser<Command> autoChooser;

  Drivetrain.FieldLocation faceLocation = Drivetrain.FieldLocation.NONE;

  public RobotContainer() {
    configureBindings();
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
    climber.setDefaultCommand(new RunCommand(() -> climber.setArmSpeed(operator.getRawAxis(1)), climber));

    NamedCommands.registerCommand("Start", new InstantCommand(() -> {shooter. setShootSpeed(80); shooter.setFeederSpeed(0.3); shooter.setAuton(true);}));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> superstructure.setGoalState(SystemState.INTAKE)));
    autoChooser = AutoBuilder.buildAutoChooser();
    PathBuilder.setupQuestions();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }


  private void configureBindings() {
    Command intakeCommand = new AutoIntake();

    driver.button(IO.resetGyroButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
    driver.button(IO.resetOdometryButton).onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())));
    // driver.button(IO.faceSpeakerButton).onTrue(new InstantCommand(() -> faceLocation = Drivetrain.FieldLocation.SPEAKER));
    // driver.button(IO.faceSpeakerButton).onFalse(new InstantCommand(() -> faceLocation = Drivetrain.FieldLocation.NONE));

    // driver.button(IO.resetIntakePositionButton).onTrue(new InstantCommand(() -> {shooter.resetPosition(0); intake.resetPosition(0);}));
    // driver.button(IO.intakeButton).whileTrue(new InstantCommand(() -> superstructure.setGoalState(SystemState.INTAKE))).onFalse(new InstantCommand(() -> superstructure.setGoalState(SystemState.STOW)));
    // driver.button(IO.intakeButton).whileTrue(new AutoIntake()).onFalse(new InstantCommand(() -> {superstructure.setGoalState(Superstructure.SystemState.STOW); intake.setRollerSpeed(0); shooter.setFeederSpeed(0);}, superstructure, intake, shooter));
    operator.button(IO.intakeButton).onTrue(intakeCommand);
    operator.button(4).onTrue(new SetSuperstructureState(Superstructure.SystemState.STOW).alongWith(new InstantCommand(() -> {intakeCommand.cancel(); intake.setRollerSpeed(0); shooter.setFeederSpeed(0); shooter.setShootSpeed(0);})));
    operator.button(5).onTrue(new Shoot(() -> driver.button(14).getAsBoolean()));
    operator.button(2).whileTrue(new InstantCommand(() -> intake.setRollerSpeed(-0.3))).onFalse(new InstantCommand(() -> intake.setRollerSpeed(0)));
    // operator.button(1).onTrue(new InstantCommand(() -> {shooter.setControlState(Shooter.ControlState.MANUAL); intake.setControlState(Intake.ControlState.MANUAL);}));
    // operator.button(3).onTrue(new InstantCommand(() -> {shooter.setControlState(Shooter.ControlState.AUTOMATIC); intake.setControlState(Intake.ControlState.AUTOMATIC);}));
    operator.button(10).onTrue(new InstantCommand(() -> superstructure.reset()));
    // driver.pov(0).onTrue(new InstantCommand(() -> elevator.moveTo(-5)));
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return PathBuilder.getFullPathCommand();
    // return AutoBuilder.buildAuto("Front 3-2-1");
  }
}
