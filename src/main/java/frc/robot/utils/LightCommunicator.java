package frc.robot.utils;

import edu.wpi.first.wpilibj.SerialPort;

public class LightCommunicator {

    private SerialPort arduino;
    private int numberOfLights;

    // Buffer format: 0xFF | <number of lights being set> | <light number> | <red> | <green> | <blue> | repeat for each led being set
    // Example (2 lights being set to red, green): 0xFF 0x02 | 0x00 0xFF 0x00 0x00 | 0x01 0x00 0xFF 0x00
    private byte[] buffer;

    public LightCommunicator(int numberOfLights) {
        this.numberOfLights = numberOfLights;
    }

    public boolean tryConnectPort1() {
        try {
            arduino = new SerialPort(9600,SerialPort.Port.kUSB1);
            return true;
        }
        catch (Exception e)
        {
          System.out.println("Failed to connect to Pos Light Arduino" + e.getMessage());
          return false;
        }
    }

    public boolean tryConnectPort2() {
        try {
            arduino = new SerialPort(9600,SerialPort.Port.kUSB2);
            return true;
        }
        catch (Exception e)
        {
          System.out.println("Failed to connect to Pos Light Arduino" + e.getMessage());
          return false;
        }
    }

    public void setLightColor(int ledNumber, int r, int g, int b) {

    }

    public void setRangeColor() {}

    public void clear() {}

    public void update() {}


}
