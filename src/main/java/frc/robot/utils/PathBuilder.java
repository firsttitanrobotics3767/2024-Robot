package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathBuilder {
    private static final SendableChooser<String> StartingQuestion = new SendableChooser<String>();
    private static final SendableChooser<String> question2 = new SendableChooser<String>();
    private static final SendableChooser<String> question1 = new SendableChooser<String>();
    private static final SendableChooser<String> question3 = new SendableChooser<String>();
    private static final SendableChooser<String> question4 = new SendableChooser<String>();
    private static final SendableChooser<String> question5 = new SendableChooser<String>();

    public static List<PathPlannerPath> getPathList() {
        ArrayList<String> autoPoses = new ArrayList<String>();
        List<PathPlannerPath> pathList = new ArrayList<PathPlannerPath>();

        autoPoses.add(StartingQuestion.getSelected());
        autoPoses.add(question1.getSelected());
        autoPoses.add(question2.getSelected());
        autoPoses.add(question3.getSelected());
        autoPoses.add(question4.getSelected());
        autoPoses.add(question5.getSelected());

        for (int i = 0; i < 5; i++) {
            String pathName = autoPoses.get(i) + "-" + autoPoses.get(i + 1);
            System.out.println(pathName);
            if (pathName.contains("null")) {continue;}
            pathList.add(PathPlannerPath.fromPathFile(pathName));
        }
        return pathList;
    }

    public static List<Command> getCommandList() {
        List<Command> pathCommandList = new ArrayList<Command>();
        for (PathPlannerPath path : getPathList()) {
            pathCommandList.add(AutoBuilder.followPath(path));
        }
        return pathCommandList;
    }

    public static Command getFullPathCommand() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        for (PathPlannerPath path : getPathList()) {
            commandGroup.addCommands(AutoBuilder.followPath(path));
        }
        return commandGroup;
    }

    public static void setupQuestions() {
        SmartDashboard.putData("Starting Position", StartingQuestion);
        SmartDashboard.putData("Note 1", question1);
        SmartDashboard.putData("Note 2", question2);
        SmartDashboard.putData("Note 3", question3);
        SmartDashboard.putData("Note 4", question4);
        SmartDashboard.putData("Note 5", question5);

        ArrayList<String> startingPoses = new ArrayList<String>();
        ArrayList<String> notes = new ArrayList<String>();

        startingPoses.add("S1");
        startingPoses.add("S2");
        startingPoses.add("Front");

        for (int i = 1; i < 4; i++) {
            notes.add(String.valueOf(i));
        }

        for (int i = 1; i < 6; i++) {
            notes.add("C" + String.valueOf(i));
        }

        StartingQuestion.addOption(startingPoses.get(0), startingPoses.get(0));
        StartingQuestion.addOption(startingPoses.get(1), startingPoses.get(1));
        StartingQuestion.addOption(startingPoses.get(2), startingPoses.get(2));

        for (int i = 0; i < notes.size(); i++) {
                question1.addOption(notes.get(i), notes.get(i));
                question2.addOption(notes.get(i), notes.get(i));
                question3.addOption(notes.get(i), notes.get(i));
                question4.addOption(notes.get(i), notes.get(i));
                question5.addOption(notes.get(i), notes.get(i));
        }

        question1.setDefaultOption("null", "null");
        question2.setDefaultOption("null", "null");
        question3.setDefaultOption("null", "null");
        question4.setDefaultOption("null", "null");
        question5.setDefaultOption("null", "null");
    }
}
