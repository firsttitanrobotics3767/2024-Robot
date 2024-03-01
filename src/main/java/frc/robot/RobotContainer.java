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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Amp;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.SetSuperstructureState;
import frc.robot.commands.Shoot;
import frc.robot.commands.Drivetrain.TeleopDrive;
import frc.robot.commands.intake.SetIntakePosition;
import frc.robot.commands.shooter.SetShooterPosition;
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

  // CommandJoystick driver = new CommandJoystick(0);
  // CommandPS5Controller driver = new CommandPS5Controller(0);
  PS5Controller driver = new PS5Controller(0);
  // PS5Controller driverRumble = new PS5Controller(0);
  // GenericHID driverRumble = new GenericHID(0);
  // CommandJoystick operator = new CommandJoystick(1);
  PS5Controller operator = new PS5Controller(1);

  SendableChooser<Command> autoChooser;

  Drivetrain.FieldLocation faceLocation = Drivetrain.FieldLocation.NONE;

  public RobotContainer() {
    configureBindings();
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveXAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveYAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveOmegaAxis), Constants.IO.swerveDeadband),
      () -> !driver.getRawButton(IO.driveModeButton),
      () -> faceLocation
    ));
    // intake.setDefaultCommand(new RunCommand(() -> intake.setPositionSpeed(operator.getRawAxis(1)), intake));
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.setPositionSpeed(operator.getRawAxis(5)), shooter));
    // climber.setDefaultCommand(new RunCommand(() -> climber.setArmSpeed(operator.getRawAxis(2)), climber));
    elevator.setDefaultCommand(new RunCommand(() -> elevator.setSpeed(-operator.getRawAxis(1)), elevator));
    climber.setDefaultCommand(new RunCommand(() -> climber.setArmSpeed(operator.getRawAxis(5)), climber));

    NamedCommands.registerCommand("Start", new InstantCommand(() -> {shooter.moveTo(Shooter.PositionState.SHOOT); shooter. setShootSpeed(80); intake.moveTo(Intake.PositionState.GROUND); intake.setRollerSpeed(0.3);}).andThen(new WaitCommand(0.3)).andThen(new InstantCommand(() -> shooter.setFeederSpeed(0.3))).andThen(new InstantCommand(() -> {shooter.moveTo(Shooter.PositionState.SCORE_3);})));
    NamedCommands.registerCommand("Wait For Ring", new InstantCommand(() -> {intake.moveTo(Intake.PositionState.GROUND); intake.setRollerSpeed(0.3);}).andThen(new WaitUntilCommand(() -> intake.hasGamePiece()).andThen(new InstantCommand(() -> {intake.setRollerSpeed(0);})).andThen(new SetIntakePosition(Intake.PositionState.STOW)).andThen(new InstantCommand(() -> {intake.setRollerSpeed(0.3); shooter.setFeederSpeed(0.3);}))).andThen(new WaitCommand(0.5)));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> {intake.moveTo(Intake.PositionState.GROUND); intake.setRollerSpeed(0.3);}));
    autoChooser = AutoBuilder.buildAutoChooser();
    PathBuilder.setupQuestions();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }


  private void configureBindings() {
    Command intakeCommand = new AutoIntake(operator);
    Trigger resetGyroButton = new Trigger(() -> driver.getRawButton(IO.resetGyroButton));

    Trigger intakeButton = new Trigger(() -> operator.getRawButton(IO.intakeButton));
    Trigger cancelIntakeButton = new Trigger(() -> operator.getRawButton(IO.cancelIntakeButton));
    Trigger prepareSpeakerButton = new Trigger(() -> operator.getRawButton(IO.prepareSpeakerButton));
    Trigger prepareSafeShootButton = new Trigger(() -> {return operator.getPOV() == IO.safeShootPOV;});
    Trigger prepareAmpButton = new Trigger(() -> operator.getRawButton(IO.prepareAmpButton));
    Trigger shootButton = new Trigger(() -> operator.getRawButton(IO.shootButton));
    Trigger resetSuperstructureButton = new Trigger(() -> operator.getRawButton(IO.resetSuperstructureButton));

    resetGyroButton.onTrue(new InstantCommand(drivetrain::zeroGyro));

    intakeButton.onTrue(intakeCommand);
    cancelIntakeButton.onTrue(new SetIntakePosition(Intake.PositionState.STOW).alongWith(new SetShooterPosition(Shooter.PositionState.HANDOFF)).alongWith(new InstantCommand(() -> {intakeCommand.cancel(); intake.setRollerSpeed(0); shooter.setFeederSpeed(0); shooter.setShootSpeed(0);})));
    prepareSpeakerButton.onTrue(new Shoot(() -> shootButton.getAsBoolean()));
    prepareAmpButton.onTrue(new Amp(() -> shootButton.getAsBoolean()));
    resetSuperstructureButton.onTrue(new InstantCommand(() -> superstructure.reset()));
    new Trigger(() -> operator.getRawButton(3)).onTrue(new InstantCommand(() -> elevator.moveTo(Elevator.PositionState.AMP)));
    new Trigger(() -> operator.getRawButton(10)).onTrue(new InstantCommand(() -> elevator.resetPosition()));
    new Trigger(() -> operator.getRawButton(14)).onTrue(new SetIntakePosition(Intake.PositionState.GROUND).alongWith(new SetShooterPosition(Shooter.PositionState.AMP)));
    new Trigger(() -> operator.getRawButton(9)).onTrue(new InstantCommand(() -> {shooter.setShootSpeed(20); shooter.setFeederSpeed(0.3);}));
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return PathBuilder.getFullPathCommand();
    // return AutoBuilder.buildAuto("Front 3-2-1");
  }
}
