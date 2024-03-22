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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    EstimatedRobotPose previousPose;
    LinearFilter xFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter yFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter zFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter radFilter = LinearFilter.singlePoleIIR(0.1, 0.2);

    
    public Vision() {

        this.drivetrain = Drivetrain.getInstance();

        aprilTagCam = new PhotonCamera("Arducam_OV9281_USB_Camera");
        Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-11), Units.inchesToMeters(-7.25), Units.inchesToMeters(8.75)), new Rotation3d(0, Units.degreesToRadians(180), Units.degreesToRadians(0)));

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.AVERAGE_BEST_TARGETS, aprilTagCam, robotToCam);

    }

    @Override
    public void periodic() {
        var result = aprilTagCam.getLatestResult();
        hasTargets = result.hasTargets();

        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.targets;
            for (PhotonTrackedTarget target : targets) {
                if (target.getPoseAmbiguity() >= 0.03) {
                    SmartDashboard.putBoolean("vision/isEstimating", false);
                    break;
                } else {
                    estimatedPose = getEstimatedGlobalPose(drivetrain.getPose());
                    previousPose = estimatedPose.isPresent() ? estimatedPose.get() : previousPose;
                    double x = xFilter.calculate(estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.getX() : previousPose.estimatedPose.getX());
                    double y = yFilter.calculate(estimatedPose.isPresent() ?  estimatedPose.get().estimatedPose.getY() : previousPose.estimatedPose.getY());
                    double z = zFilter.calculate(estimatedPose.isPresent() ?  estimatedPose.get().estimatedPose.getZ() : previousPose.estimatedPose.getZ());
                    double rad = radFilter.calculate(estimatedPose.isPresent() ?  estimatedPose.get().estimatedPose.getRotation().toRotation2d().getRadians() : previousPose.estimatedPose.getRotation().toRotation2d().getRadians());
                    Pose3d averagedPose = new Pose3d(x, y, z, new Rotation3d(0, 0, rad));
                    SmartDashboard.putBoolean("vision/isEstimating", true);
                    SmartDashboard.putString("vision/estimatedPose", estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.toString() : "no pose");
        
                    if (estimatedPose.isPresent()) {
                        drivetrain.addVisionMeasurement(averagedPose);
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
