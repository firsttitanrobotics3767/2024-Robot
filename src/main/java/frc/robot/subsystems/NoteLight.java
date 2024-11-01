package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NoteLight extends SubsystemBase {
  
  private SerialPort noteLight;

  private String color = "Orange";
  private String lastColor = "Orange";

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public NoteLight() {
        try {
            noteLight = new SerialPort(9600,SerialPort.Port.kUSB1);
      }
      catch (Exception e)
        {
          System.out.println("Failed to connect to Note Light Arduino" + e.getMessage());
      }
      
      try {
            noteLight = new SerialPort(9600,SerialPort.Port.kUSB2);
      }
      catch (Exception e)
        {
          System.out.println("Failed to connect to Note Light Arduino" + e.getMessage());
      }

      SmartDashboard.putBoolean("Color/Red", false);
      SmartDashboard.putBoolean("Color/Orange", true);
      SmartDashboard.putBoolean("Color/Yellow", false);
      SmartDashboard.putBoolean("Color/Green", false);
      SmartDashboard.putBoolean("Color/Blue", false);
      SmartDashboard.putBoolean("Color/Violet", false);

      m_chooser.setDefaultOption("Orange", "Orange");
      m_chooser.addOption("Red", "Red");
      m_chooser.addOption("Yellow", "Yellow");
      m_chooser.addOption("Green", "Green");
      m_chooser.addOption("Blue", "Blue");
      m_chooser.addOption("Violet", "Violet");
      SmartDashboard.putData("Note Color", m_chooser);
    }

    @Override
    public void periodic() {
      String colorCode = "";

      switch (color) {
        case "Red":
          colorCode = "R";
          break;
         case "Orange":
          colorCode = "O";
          break;
          case "Yellow":
          colorCode = "Y";
          break;
          case "Green":
          colorCode = "G";
          break;
          case "Blue":
          colorCode = "B";
          break;
          case "Violet":
          colorCode = "P";
          break;
        default:
          colorCode = "O";
          break;
      }

      
      noteLight.writeString("P");
      
      SmartDashboard.putBoolean("Color/" + color, true);
      SmartDashboard.putBoolean("Color/" + lastColor, false);
      lastColor = color;
    }

    public void setColor(String color) {
      this.color = color;
      System.out.println("changing Color to " + color);
    }
}