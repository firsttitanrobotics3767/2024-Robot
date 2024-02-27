package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    private final Drivetrain drivetrain;

    boolean hasTargets;

    private final PhotonCamera aprilTagCam;
    private final AprilTagFieldLayout aprilTagField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonPoseEstimator photonPoseEstimator;
    Optional<EstimatedRobotPose> estimatedPose;
    
    public Vision() {

        this.drivetrain = Drivetrain.getInstance();

        aprilTagCam = new PhotonCamera("Arducam_OV9281_USB_Camera");
        //Transform3d robotToCam = new Transform3d(new Translation3d(-0.298, 0, 0.205), new Rotation3d(0, Units.degreesToRadians(45), 0));
        Transform3d robotToCamNeptune = new Transform3d(new Translation3d(-Units.inchesToMeters(13.25), 0, Units.inchesToMeters(5)), new Rotation3d(0, Units.degreesToRadians(3.5), Units.degreesToRadians(180)));

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.AVERAGE_BEST_TARGETS, aprilTagCam, robotToCamNeptune);

    }

    @Override
    public void periodic() {
        var result = aprilTagCam.getLatestResult();
        hasTargets = result.hasTargets();

        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.targets;
            for (PhotonTrackedTarget target : targets) {
                if (target.getPoseAmbiguity() >= 0.08) {
                    System.out.println("not estimating");
                    break;
                } else {
                    estimatedPose = getEstimatedGlobalPose(drivetrain.getPose());
                    System.out.println(estimatedPose.get().estimatedPose);
        
                    if (estimatedPose.isPresent()) {
                        drivetrain.addVisionMeasurement(estimatedPose.get().estimatedPose);
                    }
                }
            }

        }

    }   

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}
