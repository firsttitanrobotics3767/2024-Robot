// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Amp;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.Pass;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SetShooterPosition;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAutoAim;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.IO;

public class RobotContainer {
  
  public final Vision vision = new Vision();
  public final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  public final Shooter shooter = Shooter.getInstance();
  private final Climber climber = Climber.getInstance();

  PS5Controller driver = new PS5Controller(0);
  PS5Controller operator = new PS5Controller(1);

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveXAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveYAxis), Constants.IO.swerveDeadband),
      () -> MathUtil.applyDeadband(-driver.getRawAxis(IO.driveOmegaAxis), Constants.IO.swerveDeadband),
      () -> !driver.getRawButton(IO.driveModeButton)
    ));
    //intake.setDefaultCommand(new RunCommand(() -> intake.setPositionSpeed(operator.getRawAxis(1)), intake));
    //shooter.setDefaultCommand(new RunCommand(() -> shooter.setPositionSpeed(operator.getRawAxis(1)), shooter));
    elevator.setDefaultCommand(new RunCommand(() -> elevator.setSpeed(MathUtil.applyDeadband(-operator.getRawAxis(5), Constants.IO.elevatorDeadband)), elevator));
    //climber.setDefaultCommand(new RunCommand(() -> climber.setArmSpeed(MathUtil.applyDeadband(-operator.getRawAxis(1), Constants.IO.climberDeadband)), climber));

    NamedCommands.registerCommand("Start", new InstantCommand(() -> {shooter.moveTo(Shooter.PositionState.SHOOT); shooter. setShootSpeed(80); intake.moveTo(Intake.PositionState.GROUND); intake.setRollerSpeed(0.3);}).andThen(new WaitCommand(0.5)).andThen(new InstantCommand(() -> shooter.setFeederSpeed(0.3))).andThen(new InstantCommand(() -> {shooter.moveTo(Shooter.PositionState.SCORE_3);})));
    NamedCommands.registerCommand("Side Score", new InstantCommand(() -> {shooter.moveTo(Shooter.PositionState.SIDE_SCORE); shooter. setShootSpeed(80); intake.moveTo(Intake.PositionState.STOW); intake.setRollerSpeed(0.3);}).andThen(new WaitCommand(0.3)).andThen(new InstantCommand(() -> shooter.setFeederSpeed(0.3))).andThen(new InstantCommand(() -> {shooter.moveTo(Shooter.PositionState.HANDOFF);})));
    NamedCommands.registerCommand("Wait For Ring", new InstantCommand(() -> {intake.moveTo(Intake.PositionState.GROUND); intake.setRollerSpeed(0.3);}).andThen(new WaitUntilCommand(() -> intake.hasGamePiece()).andThen(new InstantCommand(() -> {intake.setRollerSpeed(0);})).andThen(new SetIntakePosition(Intake.PositionState.STOW)).andThen(new InstantCommand(() -> {intake.setRollerSpeed(0.4); shooter.setFeederSpeed(0.3);}))).andThen(new WaitCommand(1)));
    NamedCommands.registerCommand("Wait For Far Ring", new InstantCommand(() -> {intake.moveTo(Intake.PositionState.GROUND); intake.setRollerSpeed(0.3);}).andThen(new WaitUntilCommand(() -> intake.hasGamePiece()).andThen(new InstantCommand(() -> {intake.setRollerSpeed(0);})).andThen(new SetIntakePosition(Intake.PositionState.STOW))));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> {intake.moveTo(Intake.PositionState.GROUND); intake.setRollerSpeed(0.3);}));
    NamedCommands.registerCommand("End", new InstantCommand(() -> {shooter.setShootSpeed(0); shooter.setFeederSpeed(0); shooter.moveTo(Shooter.PositionState.HANDOFF); intake.setRollerSpeed(0); intake.moveTo(Intake.PositionState.STOW);}));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }


  private void configureBindings() {
    Command intakeCommand = new AutoIntake(operator);
    Trigger resetGyroButton = new Trigger(() -> driver.getRawButton(IO.resetGyroButton));
    Trigger faceLocationButton = new Trigger(() -> driver.getRawButton(5));

    Trigger intakeButton = new Trigger(() -> operator.getRawButton(IO.intakeButton));
    Trigger cancelIntakeButton = new Trigger(() -> operator.getRawButton(IO.cancelIntakeButton));
    Trigger reverseIntakeButton = new Trigger(() -> operator.getRawButton(2));
    Trigger manualIntake = new Trigger(() -> {return operator.getPOV() == 0;});
    Trigger prepareSpeakerButton = new Trigger(() -> operator.getRawButton(IO.prepareSpeakerButton));
    Trigger passButton = new Trigger(() -> operator.getRawButton(1));
    Trigger prepareAmpButton = new Trigger(() -> operator.getRawButton(IO.prepareAmpButton));
    Trigger shootButton = new Trigger(() -> operator.getRawButton(IO.shootButton));

    resetGyroButton.onTrue(new InstantCommand(drivetrain::zeroGyro));
    faceLocationButton.whileTrue(drivetrain.aimChassis((DriverStation.getAlliance().get() == Alliance.Blue) ? Constants.fieldLocations.blueSpeaker : Constants.fieldLocations.redSpeaker));

    intakeButton.onTrue(new InstantCommand(() -> intakeCommand.cancel()).andThen(intakeCommand));
    cancelIntakeButton.onTrue(new SetIntakePosition(Intake.PositionState.STOW).alongWith(new SetShooterPosition(Shooter.PositionState.HANDOFF)).alongWith(new InstantCommand(() -> {intakeCommand.cancel(); intake.setRollerSpeed(0); shooter.setFeederSpeed(0); shooter.setShootSpeed(0);})));
    reverseIntakeButton.onTrue(new InstantCommand(() -> intake.setRollerSpeed(-0.3)));
    reverseIntakeButton.onFalse(new InstantCommand(() -> intake.setRollerSpeed(0)));
    manualIntake.onTrue(new InstantCommand(() -> {intake.setRollerSpeed(0.2); shooter.setFeederSpeed(0.2);}));
    manualIntake.onFalse(new InstantCommand(() -> {intake.setRollerSpeed(0); shooter.setFeederSpeed(0);}));
    prepareSpeakerButton.onTrue(new ShootAutoAim(() -> shootButton.getAsBoolean()));
    //prepareSpeakerButton.onTrue(new SetShooterPosition(Shooter.PositionState.AUTO).alongWith(new SetIntakePosition(Intake.PositionState.SCORING)).alongWith(new SequentialCommandGroup(new InstantCommand(() -> {shooter.setFeederSpeed(-0.2); shooter.setShootSpeed(-2);}), new WaitCommand(0.1))).andThen(new InstantCommand(() -> {shooter.setShootSpeed(0); shooter.setFeederSpeed(0);})).andThen(new RunCommand(() -> shooter.setPositionSpeed(operator.getRawAxis(1)))).alongWith(new RunCommand(() -> shooter.setShootSpeed(80))));
    //shootButton.onTrue(new RunCommand(() -> shooter.setFeederSpeed(0.3)));
    passButton.onTrue(new Pass(() -> shootButton.getAsBoolean()));
    prepareAmpButton.onTrue(new Amp(() -> shootButton.getAsBoolean()));
    new Trigger(() -> operator.getRawButton(9)).onTrue(new InstantCommand(() -> elevator.resetPosition()));
    new Trigger(() -> operator.getRawButton(14)).onTrue(new SetIntakePosition(Intake.PositionState.GROUND).alongWith(new SetShooterPosition(Shooter.PositionState.AMP)));
    new Trigger(() -> operator.getRawButton(3)).onTrue(new InstantCommand(() -> {intake.setRollerSpeed(0.2); shooter.setFeederSpeed(0.2); shooter.setShootSpeed(-2);}).andThen(new WaitCommand(0.4)).andThen(new WaitUntilCommand(() -> intake.getTorqueCurrent() < 25)).andThen(new InstantCommand(() -> {intake.setRollerSpeed(0.0); shooter.setFeederSpeed(0.0); shooter.setShootSpeed(0); SmartDashboard.putBoolean("Intake Ring", false); SmartDashboard.putBoolean("Ready to Shoot", true);})));
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
