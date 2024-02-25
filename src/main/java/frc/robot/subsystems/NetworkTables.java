package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTables extends SubsystemBase {
  // test variables
  double x;
  double y;
  // needed variables
  double intakeDistance = 0;
  double shooterDistance = 0;
  // heartbeat variables
  double heartbeat1 = 1;
  double heartbeat2;
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
    intakeDistance = SmartDashboard.getNumber("Intake/sensorDistance", intakeDistance);
    shooterDistance = SmartDashboard.getNumber("Shooter/sensorDistance", shooterDistance);

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

    if (intakeDistance <= 30 && intakeDistance != 0) {
      SmartDashboard.putBoolean(intakeTopic, true);
    } else {
      SmartDashboard.putBoolean(intakeTopic, false);
    }
    // shooter
    if (shooterDistance <= 30 && shooterDistance != 0) {
      SmartDashboard.putBoolean(shooterTopic, true);
    } else {
      SmartDashboard.putBoolean(shooterTopic, false);
    }
    //

  }

  public void heartbeatReader() {
    heartbeat2 = heartbeat1;
    SmartDashboard.getNumber("orangepi_heartbeat", heartbeat1);
    if (heartbeat1 != heartbeat2 && heartbeat1 > 600) {
      SmartDashboard.putBoolean("ORANGE PI IS ALIVE", true);
    } else {
      SmartDashboard.putBoolean("ORANGE PI IS ALIVE", false);
    }
  }

  public double getIntakeDistance() {
    return intakeDistance;
  }

  public double getShooterDistance() {
    return shooterDistance;
  }

}
