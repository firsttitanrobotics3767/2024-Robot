package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonBuilder extends SubsystemBase{

    Field2d autoPreview = new Field2d();
    
    SendableChooser<String> StartingQuestion = new SendableChooser<String>();
    SendableChooser<String> question1 = new SendableChooser<String>();
    SendableChooser<String> question2 = new SendableChooser<String>();
    SendableChooser<String> question3 = new SendableChooser<String>();
    SendableChooser<String> question4 = new SendableChooser<String>();
    SendableChooser<String> question5 = new SendableChooser<String>();

    ArrayList<String> startingQuestion = new ArrayList<String>(3);
    ArrayList<String> notes = new ArrayList<String>(8);

    public AutonBuilder() {
        setupQuestions();

        for (int i = 0; i < 9; i++) {
            if (i >= 1) {
                question1.addOption(notes.get(i-1), notes.get(i-1));
                question2.addOption(notes.get(i-1), notes.get(i-1));
                question3.addOption(notes.get(i-1), notes.get(i-1));
                question4.addOption(notes.get(i-1), notes.get(i-1));
                question5.addOption(notes.get(i-1), notes.get(i-1));
            } else {
                StartingQuestion.addOption(startingQuestion.get(0), startingQuestion.get(0));
                StartingQuestion.addOption(startingQuestion.get(1), startingQuestion.get(1));
                StartingQuestion.addOption(startingQuestion.get(2), startingQuestion.get(2));
            }

            SmartDashboard.putData("Starting Position", StartingQuestion);
            SmartDashboard.putData("question 1", question1);
            SmartDashboard.putData("question 2", question2);
            SmartDashboard.putData("question 3", question3);
            SmartDashboard.putData("question 4", question4);
            SmartDashboard.putData("question 5", question5);

        }

    }

    public Command buildAuto() {
        ArrayList<String> autoPoses = new ArrayList<String>(6);

        autoPoses.add(StartingQuestion.getSelected());
        autoPoses.add(question1.getSelected());
        autoPoses.add(question2.getSelected());
        autoPoses.add(question3.getSelected());
        autoPoses.add(question4.getSelected());
        autoPoses.add(question5.getSelected());

        ArrayList<String> pathList = new ArrayList<>(6);

        for (int i = 0; i < 5; i++) {
            pathList.add(autoPoses.get(i) + "-" + autoPoses.get(i + 1));
            System.out.println(pathList.get(i));
        }
        return new SequentialCommandGroup(
            pathList.get(0) != null ? AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathList.get(0))) : new InstantCommand(),
            pathList.get(1) != null ? AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathList.get(1))) : new InstantCommand(),
            pathList.get(2) != null ? AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathList.get(2))) : new InstantCommand(),
            pathList.get(3) != null ? AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathList.get(3))) : new InstantCommand(),
            pathList.get(4) != null ? AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathList.get(4))) : new InstantCommand()
        );
    }

    private void setupQuestions() {
        startingQuestion.add("S1");
        startingQuestion.add("S2");
        startingQuestion.add("Front");

        for (int i = 1; i < 4; i++) {
            notes.add(String.valueOf(i));
        }

        for (int i = 1; i < 6; i++) {
            notes.add("C" + String.valueOf(i));
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Auto Preview", autoPreview);
    }

}
