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
  double x;
  double y;
  double read1;
  double read2;

  public NetworkTables() {
    // NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // NetworkTable table = inst.getTable("sensorData");
    // data1 = table.getDoubleTopic("data1").publish();
    // data2 = table.getDoubleTopic("data2").publish();
  }

  public void publish() {
    // data1.set(x);
    // data2.set(y);
    x += 0.05;
    y += 1;
    SmartDashboard.putNumber("data1", x);
    SmartDashboard.putNumber("data2", y);
    read1 = SmartDashboard.getNumber("data1", read1);
    read2 = SmartDashboard.getNumber("data2", read2);
    System.out.println("read1 = " + read1);
    System.out.println("read2 = " + read2);
    System.out.println("");
  }

}
