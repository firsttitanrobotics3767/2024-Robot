package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

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
import monologue.Logged;

import java.util.HashMap;

public class Vision extends SubsystemBase implements Logged{

    public enum Type {
        AprilTag,
        ObjectDetection,
        DriveCam
    }

    private static HashMap<String, Vision> cameras = new HashMap<String, Vision>();

    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final Type cameraType;
    private static boolean doEstimation = true;
    private boolean cameraValid = true;
    private final String cameraName;

    boolean hasTargets = false;
    boolean hasRingTargets = false;
    Transform3d robotToRingCam = new Transform3d(new Translation3d(Units.inchesToMeters(21), 0, Units.inchesToMeters(10.25)), new Rotation3d(0, Units.degreesToRadians(35), 0));
    List<Translation2d> ringPoses = new ArrayList<Translation2d>();

    private final PhotonCamera camera;
    private final AprilTagFieldLayout aprilTagField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonPoseEstimator photonPoseEstimator;
    Optional<EstimatedRobotPose> estimatedPose = Optional.of(new EstimatedRobotPose(new Pose3d(), 0, null, null));
    
    EstimatedRobotPose previousPose = new EstimatedRobotPose(new Pose3d(), 0, null, null);
    double lastUpdateTimestamp = 0;
    LinearFilter xFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter yFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter zFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    LinearFilter radFilter = LinearFilter.singlePoleIIR(0.1, 0.2);

    private Pose3d mostRecentPose = new Pose3d();
    private boolean outdatedPose = true;
    private Timer timeSinceLastUpdate = new Timer();
    
    public Vision(String cameraName, Vision.Type cameraType, Transform3d robotToCam) {
        camera = new PhotonCamera(cameraName);
        if (cameraType == Type.AprilTag) {
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        } else if (cameraType == Type.DriveCam) {
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.AVERAGE_BEST_TARGETS, robotToCam);
            camera.setDriverMode(true);
        } else {
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.AVERAGE_BEST_TARGETS, robotToCam);
        }
        this.cameraName = cameraName;
        this.cameraType = cameraType;
        cameras.put(cameraName, this);
    }

    @Override
    public void periodic() {
        if (cameraValid) {
            if (cameraType == Type.AprilTag) {
                var result = camera.getLatestResult();
                hasTargets = result.hasTargets();

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

                    
                    if (!estimate) {
                        SmartDashboard.putBoolean("vision/isEstimating", false);
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

                        mostRecentPose = new Pose3d(x, y, z, new Rotation3d(0, 0, rad));

                        if (estimatedPose.isPresent() && doEstimation) {
                            SmartDashboard.putString("vision/estimatedPose", estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.toString() : "no pose");
                            SmartDashboard.putBoolean("vision/isEstimating", true);
                            lastUpdateTimestamp = Timer.getFPGATimestamp();
                            timeSinceLastUpdate.restart();
                        }
                    }
                }
            }
        }

        if (timeSinceLastUpdate.hasElapsed(0.05)) {
            outdatedPose = true;
        } else {
            outdatedPose = false;
        }  

        if (cameraType == Type.AprilTag) {
            log(cameraName + "/Most Recent Pose", getEstimatedPose());
            log(cameraName + "/Raw Pose Estimation", estimatedPose.get().estimatedPose);
            log(cameraName + "/Is Estimating", estimatedPose.isPresent() && doEstimation);
            log(cameraName + "/Pose Outdated", poseOutadated());
            log(cameraName + "/Time Since Last Update", timeSinceLastUpdate());
        }
        log(cameraName + "/Is Valid", cameraValid);
    }   

    public static Vision getCamera(String cameraName) {
        return cameras.get(cameraName);
    }

    public static HashMap<String, Vision> getCameras() {
        return cameras;
    }
    
    public Pose3d getEstimatedPose() {
        return mostRecentPose;
    }

    public boolean poseOutadated() {
        return outdatedPose;
    }

    public double[][] getMinMaxCorners(PhotonTrackedTarget ring) {
        List<Double> cornersX = new ArrayList<Double>();
        List<Double> cornersY = new ArrayList<Double>();

        for (TargetCorner corner : ring.getMinAreaRectCorners()) {
            cornersX.add(corner.x);
            cornersY.add(corner.y);
        }

        cornersX.sort(Comparator.naturalOrder());
        cornersY.sort(Comparator.naturalOrder());

        double cornersMinMax[][] = {
            {cornersX.get(0), cornersX.get(3)},
            {cornersY.get(3), cornersX.get(0)}
        };

        SmartDashboard.putString("vision/ring bounding corners", cornersMinMax.toString());

        return cornersMinMax;
    }

    // public List<Double> sortList(List<Double> list) {
    //     List<Double> result = new ArrayList<Double>(list.size());
    //     double tempNum;
    //     for (double item : list) {
    //         for (int i = list.size() - list.indexOf(item); i < list.size(); i++) {
    //             if (item > list.get(i)) {
    //                 tempNum = list.get(i);
    //                 result.set(i, null);
    //             }
    //         }
    //     }
    // } 

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

    public static void turnOffAprilTags() {
        doEstimation = false;
    }

    public static void turnOnAprilTags() {
        doEstimation = true;
    }

    public void setValidaty(boolean isValid) {
        cameraValid = isValid;
    }

    public boolean isValid() {
        return cameraValid;
    }

}
