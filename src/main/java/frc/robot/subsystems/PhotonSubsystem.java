package frc.robot.subsystems;

import java.io.UncheckedIOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonSubsystem extends SubsystemBase {
    private PhotonCamera cam0; // this is private to ensure we don't waste resources grabbing a new image elsewhere
    private NetworkTable NT;
    public PhotonTrackedTarget bestTarget;

    public PhotonPipelineResult cameraImage;
    public double distanceToOptimalTag;
    public int optimalTagID;

    /* pose estimation variables */
    public GenericEntry pivotEncoderShuffleBoard;
    public AprilTagFieldLayout aprilTagFieldLayout;
    public PhotonPoseEstimator poseEstimator;
    public Transform3d robotToCam;

    public ShuffleboardTab tab;

    public PhotonSubsystem() {
        NT = NetworkTableInstance.getDefault().getTable("photonvision");
        if (NT != null) {
            cam0 = new PhotonCamera(Constants.Vision.camera0);
        } else {
            cam0 = null;
        }

        /* pose estimation using photon vision */
        try {
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (UncheckedIOException e) {
            System.out.println("Requested field layout doesn't exist :(");
            e.printStackTrace();
        }

        /* where is the camera places relative to the robot */
        // TODO: figure the coordinates that we need to put here
        robotToCam = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, Units.degreesToRadians(180)));

        /* start estimating the pose of the robot */
        poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cam0,
                robotToCam);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        tab = Shuffleboard.getTab("PhotonSubsystem");
        pivotEncoderShuffleBoard = tab.add("Pivot Encoder", 0.0).getEntry();
    }

    /**
     * return the estimated pose
     *
     * @return Optional<EstimatedRobotPose>
     */
    public EstimatedRobotPose getEstimatedPose() {
        if (cam0 == null || cameraImage == null) {
            return null;
        }
        if (cam0 == null || !cam0.isConnected()
            || cameraImage.getTargets().size() < 2) {
            return null;
        }
        // if (poseEstimator.update().isPresent() && poseEstimator.update() != null) {
        //     return poseEstimator.update().get();
        // }
        return null;
    }

    /**
     * Get any selected tag from the camera image
     *
     * @param tagID tag id
     * @return PhotonTrackedTarget of the requested target or null if not found
     */
    public PhotonTrackedTarget getTag(long tagID) {
        if (cam0 == null || cameraImage == null) {
            return null;
        }
        for (PhotonTrackedTarget target : cameraImage.getTargets()) {
            if (target.getFiducialId() == tagID) {
                return target;
            }
        }
        return null;
    }

    @Override
    public void periodic() {
        if (cam0 == null) {
            return;
        }
        cameraImage = cam0.getLatestResult();

        if (cameraImage.hasTargets()) {
            bestTarget = cameraImage.getBestTarget();
            distanceToOptimalTag = bestTarget.getBestCameraToTarget().getX();
            optimalTagID = bestTarget.getFiducialId();
        }
    }
}
