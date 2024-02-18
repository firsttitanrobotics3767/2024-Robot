package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class NetworkTables extends SubsystemBase {


    // final DoubleSubscriber dataSub;
    double prev;

    // public NetworkTables() {
    //     dataSub = sensorData.getDefaul();
    // }

    // public void getData() {
    //     double value = dataSub.get();
    //     if (value != prev) {
    //         prev = value;
    //         System.out.println(value);
    //     }

    // }

}
