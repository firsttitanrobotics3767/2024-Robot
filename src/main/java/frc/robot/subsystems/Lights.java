package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private SerialPort microcontroller;

    // Control
    private byte[] LightsOff = {0x00};

    // Solid
    private byte[] SolidRed = {0x11};
    private byte[] SolidGreen = {0x12};
    private byte[] SolidBlue = {0x13};
    private byte[] SolidWhite = {0x14};
    private byte[] SolidTitan = {0x15};

    // Cylon
    private byte[] CylonRed = {0x21};
    private byte[] CylonGreen = {0x22};
    private byte[] CylonBlue = {0x23};
    private byte[] CylonWhite = {0x24};
    private byte[] CylonTitan = {0x25};


    public Lights() {
        try {
            microcontroller = new SerialPort(9600, SerialPort.Port.kUSB2);
            System.out.println("Connected to lighting microcontroller.");
        } catch (Exception e) {
            System.out.println("Failed to connect to lighting microcontroller.");
        }
    }

    public void solidWhite() {
        microcontroller.write(SolidWhite, 1);
    }
}
