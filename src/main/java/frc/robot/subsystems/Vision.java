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
        Transform3d robotToCam = new Transform3d(new Translation3d(-0.298, 0, 0.205), new Rotation3d(0, Units.degreesToRadians(45), 0));

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.AVERAGE_BEST_TARGETS, aprilTagCam, robotToCam);

    }

    @Override
    public void periodic() {
        var result = aprilTagCam.getLatestResult();
        hasTargets = result.hasTargets();

        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.targets;
            for (PhotonTrackedTarget target : targets) {
                if (target.getPoseAmbiguity() >= 0.08) {
                    SmartDashboard.putBoolean("vision/isEstimating", false);
                    break;
                } else {
                    estimatedPose = getEstimatedGlobalPose(drivetrain.getPose());
                    SmartDashboard.putBoolean("vision/isEstimating", true);
                    SmartDashboard.putString("vision/estimatedPose", estimatedPose.get().estimatedPose.toString());
        
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
