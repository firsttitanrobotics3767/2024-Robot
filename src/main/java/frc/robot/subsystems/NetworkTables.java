package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.CanandcolorProximityConfig.SamplingPeriod;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class NetworkTables extends SubsystemBase {
  // test variables
  double x;
  double y;
  // needed variables
  double intake;
  double shooter;
  // topic name variables
  String intakeTopic = "RING IN INTAKE";
  String shooterTopic = "RING IN SHOOTER";

  public NetworkTables() {}

  public void reader() {
    // testing
    x += 1;
    y += 1;
    //

    // reading values
    
    SmartDashboard.putNumber("intake", x);
    SmartDashboard.putNumber("shooter", y);
    intake = SmartDashboard.getNumber("intake", intake);
    shooter = SmartDashboard.getNumber("shooter", shooter);

    // determening if ring is in intke

    if (intake <= 20) {
      SmartDashboard.putBoolean(intakeTopic, true);
    } else {
      SmartDashboard.putBoolean(intakeTopic, false);
    }

    if (shooter <= 20) {
      SmartDashboard.putBoolean(shooterTopic, true);
    } else {
      SmartDashboard.putBoolean(shooterTopic, false);
    }

    //

  }

}
