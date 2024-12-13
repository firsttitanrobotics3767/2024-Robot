package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.StartPoses;

public class PosLights extends SubsystemBase {

    private SerialPort posLight;
    
    // Buffer format: https://docs.google.com/spreadsheets/d/1EYpQI5S_EywiqFBN-zN1tecQcnKyOR3YnvDpMJLEPE4/edit?gid=0#gid=0
    private byte[] buffer = {127, 1, 0, 0, 0, -128}; 

    private byte toMoveX;
    private byte toMoveY;
    private byte toRotate;

    // NOTE: Currents are rounded to the maximum of whatever their buffer values can handle.
    private int currentX;
    private int currentY;
    private int currentRotation;
    private int tempInt;

    private int correctX;
    private int correctY;
    private int correctRotation;

    private boolean isEnabled;

    private final SendableChooser<String> posChooser = new SendableChooser<>();

    public PosLights() {
        try {
            posLight = new SerialPort(9600,SerialPort.Port.kUSB1);
        }
        catch (Exception e)
        {
          System.out.println("Failed to connect to Pos Light Arduino" + e.getMessage());
        }
        
        try {
            posLight = new SerialPort(9600,SerialPort.Port.kUSB2);
        }
        catch (Exception e)
        {
            System.out.println("Failed to connect to Pos Light Arduino" + e.getMessage());
        }

        SmartDashboard.putNumber("toMoveX", toMoveX);
        SmartDashboard.putNumber("toMoveY", toMoveY);
        SmartDashboard.putNumber("toRotate", toRotate);

        posChooser.setDefaultOption("test", "test");
        SmartDashboard.putData("Robot Position", posChooser);
    } 

    @Override
    public void periodic() {
        if (isEnabled) {
            // Get pos
            tempInt = (int) Math.round(Drivetrain.getInstance().getPose().getX());
            if (tempInt > 12) {
                tempInt = 12;
            } else if (tempInt < -12) {
                tempInt = -12;
            } else {
                currentX = tempInt;
            }

            tempInt = (int) Math.round(Drivetrain.getInstance().getPose().getY());
            if (tempInt > 12) {
                tempInt = 12;
            } else if (tempInt < -12) {
                tempInt = -12;
            } else {
                currentY = tempInt;
            }

            tempInt = (int) Math.round((Drivetrain.getInstance().getPose().getRotation().getDegrees() - 180) / 2);
            if (tempInt > 90) {
                tempInt = 90;
            } else if (tempInt == -90) {
                tempInt = 90;
            } else {
                currentRotation = tempInt;
            }

            // Get correct pos
            switch (SmartDashboard.getData("Robot Position").toString()) {
                case "test":
                    // Pos collection
                    correctX = StartPoses.test[0];
                    correctY = StartPoses.test[1];
                    correctRotation = StartPoses.test[2];
                    break;
                default:
                    correctX = StartPoses.test[0];
                    correctY = StartPoses.test[1];
                    correctRotation = StartPoses.test[2];
                    break;
            }

            // Update
            toMoveX = (byte) (correctX - currentX);
            toMoveY = (byte) (correctY - currentY);
            toRotate = (byte) (correctRotation - currentRotation);

            SmartDashboard.putNumber("toMoveX", toMoveX);
            SmartDashboard.putNumber("toMoveY", toMoveY);
            SmartDashboard.putNumber("toRotate", toRotate);

            // Edit buffer
            buffer[2] = toMoveX;
            buffer[3] = toMoveY;
            buffer[4] = toRotate; 

            // Write buffer
            posLight.write(buffer, 6);
        }
    }

    public void enable() {
        isEnabled = true;
    }

    public void disable() {
        isEnabled = false;
    }
}
