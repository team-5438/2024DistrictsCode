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
import edu.wpi.first.math.geometry.Pose3d;
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
    public PhotonTrackedTarget tag;

    public boolean isAligned;

    public PhotonPipelineResult cameraImage;
    public double distanceToOptimalTag;
    public int optimalTagID;

    /* pose estimation variables */
    public AprilTagFieldLayout aprilTagFieldLayout;
    public PhotonPoseEstimator poseEstimator;
    public Transform3d robotToCam;

    public ShuffleboardTab tab;
    public GenericEntry updatedPoseShuffleBoard;
    public GenericEntry alignedWithSpeakerShuffleBoard;
    public GenericEntry tagX, tagY, robotX, robotY, setAngle;

    public PhotonSubsystem() {
        NT = NetworkTableInstance.getDefault().getTable("photonvision");
        if (NT != null) {
            cam0 = new PhotonCamera(Constants.Vision.camera0);
        } else {
            cam0 = null;
        }
        isAligned = false;

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
                new Translation3d(Units.inchesToMeters(-11), 0, 0),
                new Rotation3d(0, Units.degreesToRadians(13), Units.degreesToRadians(180)));

        /* start estimating the pose of the robot */
        poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cam0,
                robotToCam);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        tab = Shuffleboard.getTab("PhotonSubsystem");
        updatedPoseShuffleBoard = tab.add("Updated Pose", "").getEntry();
        alignedWithSpeakerShuffleBoard = tab.add("Aligned With Speaker", false).getEntry();
        tagX = tab.add("tag x", 0.0).getEntry();
        tagY = tab.add("tag y", 0.0).getEntry();
        robotX = tab.add("robot x", 0.0).getEntry();
        robotY = tab.add("robot y", 0.0).getEntry();
        setAngle = tab.add("set angle", 0.0).getEntry();
    }

    /**
     * return the estimated pose
     *
     * @return EstimatedRobotPose
     */
    public EstimatedRobotPose getEstimatedPose() {
        if (cam0 == null || cameraImage == null || !cam0.isConnected()
            || cameraImage.getTargets().size() < 2) {
            return null;
        }
        Optional<EstimatedRobotPose> pose = poseEstimator.update();
        if (pose != null && pose.isPresent()) {
            try {
                if (pose.get() != null) {
                    updatedPoseShuffleBoard.setString(pose.get().estimatedPose.toString());
                    return pose.get();
                }
                // not even sure we need this here, but just incase:
                return null;
            } catch (NoSuchElementException e) {
                return null;
            }
        }
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

    public Pose3d positionToTag(Pose3d currentPose, int tagID) {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagID);
        
        if (tagPose == null || tagPose.isEmpty()) { // check if tag is valid
            return null;
        }

        Rotation3d robotRotation = currentPose.getRotation();
        Rotation3d tagRotation = tagPose.get().getRotation();

        double differenceX = currentPose.getX() - tagPose.get().getX();
        double differenceY = currentPose.getY() - tagPose.get().getY();
        double differenceZ = currentPose.getZ() - tagPose.get().getZ();

        Rotation3d differenceRotation = robotRotation.minus(tagRotation);

        return new Pose3d(differenceX, differenceY, differenceZ, differenceRotation);

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

        tag = getTag(Constants.AprilTags.speakerCentral);
        if (tag != null && Math.abs(tag.getYaw()) < Constants.Vision.alignedTolerance) {
            isAligned = true;
            alignedWithSpeakerShuffleBoard.setBoolean(true);
        } else {
            isAligned = false;
            alignedWithSpeakerShuffleBoard.setBoolean(false);
        }
    }
}