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

//   DoublePublisher data1;
//   DoublePublisher data2;
  // double x;
  // double y;
  double intake;
  double shooter;
  Boolean intakeDetect;
  // Boolean shooterDitect

  public NetworkTables() {
    // NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // NetworkTable table = inst.getTable("sensorData");
    // data1 = table.getDoubleTopic("data1").publish();
    // data2 = table.getDoubleTopic("data2").publish();
  }

  public void reader() {
    // data1.set(x);
    // data2.set(y);
    intake += 0.05;
    shooter += 1;
    
    // SmartDashboard.putNumber("intake", x);
    // SmartDashboard.putNumber("shooter", y);
    intake = SmartDashboard.getNumber("intake", intake);
    shooter = SmartDashboard.getNumber("shooter", shooter);
    // determening if ring is in intke
    if (intake <= 20) {
      SmartDashboard.putBoolean("RING IN INTAKE", true);
    } else {
      SmartDashboard.putBoolean("RING IN INTAKE ", false);
    }

    if (shooter <= 20) {
      SmartDashboard.putBoolean("RING IN SHOOTER", true);
    } else {
      SmartDashboard.putBoolean("RING IN SHOOTER", false);
    }

    // SmartDashboard.putNumber("read1", read1);
    // SmartDashboard.putNumber("read2", read2);
  }

}
