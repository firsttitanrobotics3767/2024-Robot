package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RingPoseEstimator;

public class Vision extends SubsystemBase{

    private static Vision instance = null;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }

        return instance;
    }

    private final Drivetrain drivetrain;

    private boolean doEstimation = true;

    boolean hasTargets = false;
    boolean hasRingTargets = false;
    Transform3d robotToRingCam = new Transform3d(new Translation3d(Units.inchesToMeters(21), 0, Units.inchesToMeters(10.25)), new Rotation3d(0, Units.degreesToRadians(35), 0));
    List<Translation2d> ringPoses = new ArrayList<Translation2d>();

    private final PhotonCamera aprilTagCam;
    private final PhotonCamera ringCam;
    private final AprilTagFieldLayout aprilTagField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonPoseEstimator photonPoseEstimator;
    Optional<EstimatedRobotPose> estimatedPose = Optional.of(new EstimatedRobotPose(new Pose3d(), 0, null, null));
    
    EstimatedRobotPose previousPose = new EstimatedRobotPose(new Pose3d(), 0, null, null);
    double lastUpdateTimestamp = 0;
    LinearFilter xFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter yFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter zFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter radFilter = LinearFilter.singlePoleIIR(0.1, 0.2);

    
    public Vision() {

        this.drivetrain = Drivetrain.getInstance();

        aprilTagCam = new PhotonCamera("Arducam_OV9281_USB_Camera");
        Transform3d robotToCam = new Transform3d(new Translation3d(-0.289857, 0.031749, 0.171914), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(135), Units.degreesToRadians(0)));

        ringCam = new PhotonCamera("HD_USB_Camera");

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagCam, robotToCam);

    }

    @Override
    public void periodic() {
        var result = aprilTagCam.getLatestResult();
        hasTargets = result.hasTargets();

        var ringResult = ringCam.getLatestResult();
        hasRingTargets = ringResult.hasTargets();
        List<PhotonTrackedTarget> rings = (hasRingTargets ? ringResult.targets : new ArrayList<PhotonTrackedTarget>(0));

        SmartDashboard.putBoolean("vision/hasRingTarget", hasRingTargets);

        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.targets;

            boolean estimate = false;

            for (PhotonTrackedTarget target : targets) {
                if (target.getPoseAmbiguity() >= 0.01) {
                    estimate = false;
                } else {
                    estimate = true;
                }
            }

            for (PhotonTrackedTarget target : targets) {
                if (!estimate) {
                    SmartDashboard.putBoolean("vision/isEstimating", false);
                    break;
                } else {

                    if (timeSinceLastUpdate() > 0.8) {
                        xFilter.reset();
                        yFilter.reset();
                        zFilter.reset();
                        radFilter.reset();
                    }
                    
                    estimatedPose = getEstimatedGlobalPose(drivetrain.getPose());
                    previousPose = estimatedPose.isPresent() ? estimatedPose.get() : previousPose;
                    double x = xFilter.calculate(estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.getX() : previousPose.estimatedPose.getX());
                    double y = yFilter.calculate(estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.getY() : previousPose.estimatedPose.getY());
                    double z = zFilter.calculate(estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.getZ() : previousPose.estimatedPose.getZ());
                    double rad = radFilter.calculate(estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.getRotation().toRotation2d().getRadians() : previousPose.estimatedPose.getRotation().toRotation2d().getRadians());
                    
                    Pose3d averagedPose = new Pose3d(x, y, z, new Rotation3d(0, 0, rad));
                    
                    if (estimatedPose.isPresent() && doEstimation) {
                        SmartDashboard.putString("vision/estimatedPose", estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.toString() : "no pose");
                        SmartDashboard.putBoolean("vision/isEstimating", true);
                        lastUpdateTimestamp = Timer.getFPGATimestamp();
                        drivetrain.addVisionMeasurement(averagedPose);
                    }
                }
            }
            
        }

        if (hasRingTargets) {
            for (PhotonTrackedTarget ring : rings) {
                // RingPoseEstimator.calculatePose(getMinMaxCorners(ring), ring., lastUpdateTimestamp, null, lastUpdateTimestamp, null, null)
            }
        }

    }  
    
    public int[][] getMinMaxCorners(PhotonTrackedTarget ring) {
        List<Integer> cornersX = new ArrayList<Integer>();
        List<Integer> cornersY = new ArrayList<Integer>();

        for (TargetCorner corner : ring.getDetectedCorners()) {
            cornersX.add((int) corner.x);
            cornersY.add((int) corner.y);
        }
        
        cornersX.sort(Comparator.naturalOrder());
        cornersY.sort(Comparator.naturalOrder());

        int cornersMinMax[][] = {
            {cornersX.get(0), cornersX.get(3)},
            {cornersY.get(3), cornersX.get(0)}
        };

        return cornersMinMax;
    }

    public boolean hasRingTarget() {
        return hasRingTargets;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public double timeSinceLastUpdate() {
        return Timer.getFPGATimestamp() - lastUpdateTimestamp;
    }

    public void turnOffAprilTags() {
        doEstimation = false;
    }

    public void turnOnAprilTags() {
        doEstimation = true;
    }

}
