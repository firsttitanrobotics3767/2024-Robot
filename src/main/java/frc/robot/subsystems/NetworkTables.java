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
  // heartbeat variables
  double heartbeat1 = 1;
  double heartbeat2 = 0;
  // topic name variables
  String intakeTopic = "READY TO FIRE";
  String shooterTopic = "HAS RING";

  public NetworkTables () {}

  public void reader() {
    // testing
    // x += 1;
    // y += 1;
    // if (intake >= 20 && shooter >= 20) {
    //   x = 0;
    //   y = 0;
    // }
    //
    
    // reading values
    // teasing
    // SmartDashboard.putNumber("intake", x);
    // SmartDashboard.putNumber("shooter", y);
    //

    // reading the intake and shooter
    intake = SmartDashboard.getNumber("intake", intake);
    shooter = SmartDashboard.getNumber("shooter", shooter);

    // reading heartbeat
    // SmartDashboard.getNumber("orangepi_heartbet", heartbeat1);
    // SmartDashboard.getNumber("orangepi_heartbeat", heartbeat2);

    // intake and sooter alive


    /* 
     * make a reader for 
     * orangepi_heartbeat
     *  double
     *  do a pull 1 and pull 2 20 sec apart
     *  compair if = the do things
     * intake_alive
     *  boolean
     * shooter_alive
     *  boolean
     */

    // determening if ring is in intke
    // intake

    if (intake <= 30 && intake != 0) {
      SmartDashboard.putBoolean(intakeTopic, true);
    } else {
      SmartDashboard.putBoolean(intakeTopic, false);
    }
    // shooter
    if (shooter <= 30 && shooter != 0) {
      SmartDashboard.putBoolean(shooterTopic, true);
    } else {
      SmartDashboard.putBoolean(shooterTopic, false);
    }
    //

  }

  // public void heartbeatReader() {
  //   heartbeat2 = heartbeat1;
  //   SmartDashboard.getNumber("orangepi_heartbeat", heartbeat1);
  //   if (heartbeat1 != heartbeat2 && heartbeat1 > 600) {
  //     SmartDashboard.putBoolean("ORANGE PI IS ALIVE", true);
  //   } else {
  //     SmartDashboard.putBoolean("ORANGE PI IS ALIVE", false);
  //   }
  // }

    @Override
    public void periodic() {
      heartbeat2 = heartbeat1;
      heartbeat1 = SmartDashboard.getNumber("orangepi_heartbeat", heartbeat1);
      if (heartbeat1 != heartbeat2 && heartbeat1 > 600) {
        SmartDashboard.putBoolean("ORANGE PI IS ALIVE", true);
      } else {
        SmartDashboard.putBoolean("ORANGE PI IS ALIVE", false);
      }
    }

}
