// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  private RobotContainer m_robotContainer;

  private boolean debug = false;
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    m_robotContainer.loggingPeriodic(debug);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Vision.getInstance().turnOffAprilTags();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    RobotContainer.drivetrain.setHeadingCorrection(false);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    // Vision.getInstance().turnOnAprilTags();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Vision.getInstance().turnOnAprilTags();

    RobotContainer.drivetrain.setHeadingCorrection(true);
    RobotContainer.shooter.moveTo(Shooter.PositionState.HANDOFF);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    SignalLogger.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    debug = true;
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    debug = false;
  }
}
