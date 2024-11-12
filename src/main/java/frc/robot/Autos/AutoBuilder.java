package frc.robot.Autos;

import java.lang.invoke.ClassSpecializer.Factory;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import choreo.auto.AutoTrajectory;
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

    final AutoTrajectory startTo1 = factory.trajectory(StartingQuestion.getSelected() + "-" + question1.getSelected(), routine);
    final AutoTrajectory n1_n2 = factory.trajectory(question1.getSelected() + "-" + question2.getSelected(), routine);
    final AutoTrajectory n2_n3 = factory.trajectory(question2.getSelected() + "-" + question3.getSelected(), routine);
    final AutoTrajectory n3_n4 = factory.trajectory(question3.getSelected() + "-" + question4.getSelected(), routine);
    final AutoTrajectory n4_n5 = factory.trajectory(question4.getSelected() + "-" + question5.getSelected(), routine);

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

        // for (int i = 1; i < 4; i++) {
        //     notes.add("C" + String.valueOf(i));
        // }

        // for (int i = 1; i < 6; i++) {
        //     notes.add("C" + String.valueOf(i));
        // }
    }
}
