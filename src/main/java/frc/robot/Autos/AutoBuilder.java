package frc.robot.Autos;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class AutoBuilder {
    private static final SendableChooser<String> StartingQuestion = new SendableChooser<String>();
    private static final SendableChooser<String> question1 = new SendableChooser<String>();
    private static final SendableChooser<String> question2 = new SendableChooser<String>();
    private static final SendableChooser<String> question3 = new SendableChooser<String>();
    private static final SendableChooser<String> question4 = new SendableChooser<String>();
    private static final SendableChooser<String> question5 = new SendableChooser<String>();

    private static final Drivetrain drivetrain = Drivetrain.getInstance();

    public static void setupQuestions() {
        SmartDashboard.putData("Starting Position", StartingQuestion);
        SmartDashboard.putData("note 1", question1);
        SmartDashboard.putData("note 2", question2);
        SmartDashboard.putData("note 3", question3);
        SmartDashboard.putData("note 4", question4);
        SmartDashboard.putData("note 5", question5);

        ArrayList<String> startingPoses = new ArrayList<String>();
        ArrayList<String> notes = new ArrayList<String>();

        startingPoses.add("S1");
        startingPoses.add("S2");
        startingPoses.add("Front");

        for (int i = 1; i < 4; i++) {
            notes.add("C" + String.valueOf(i));
        }

        for (int i = 1; i < 6; i++) {
            notes.add("C" + String.valueOf(i));
        }
    }
}
