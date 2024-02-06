package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class SensorsSubsystem extends SubsystemBase {
    private SerialPort teensy;
    private Timer timer;
    private StringPublisher luxPub;
    private double luxData;
    private String dataString;
    private boolean alive;

    public SensorsSubsystem() {
        try {
            teensy = new SerialPort(9600, SerialPort.Port.kUSB1);
            System.out.println("Connected to Teensy");
            alive = true;
        } catch (Exception e) {
            System.out.println("Failed to connect to Teensy");
            alive = false;
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("sensorData");
        luxPub = table.getStringTopic("lux").publish();

        timer = new Timer();
        timer.start();
    }

    public boolean dataPublish() {
        //if (alive) {
            dataString = teensy.readString();
            System.out.println(dataString);
            
            if (dataString.contains("12345")) {
                return true;
            } else {
                return false;
            }
            //luxData = Double.valueOf(dataString);
        //}

        // if (dataString != null && dataString != "" && dataString != "\n") { 
        //     luxPub.set(dataString);
        // }
    }
}
