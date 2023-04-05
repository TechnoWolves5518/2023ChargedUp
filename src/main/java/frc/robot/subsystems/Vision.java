package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Wrapper for PhotonCamera class */
public class Vision extends PhotonCamera {

    private static final String DEFAULT_CAM_NAME = "AprilTagCamera";
    private static final double DEFAULT_CAM_X = Units.inchesToMeters(-2); // .5m forward of center
    private static final double DEFAULT_CAM_Y = Units.inchesToMeters(8); // 8 in left of center
    private static final double DEFAULT_CAM_Z = Units.inchesToMeters(16.5); // 52in up from center
    private final double CAMERA_HEIGHT = DEFAULT_CAM_Z; // height on robot (meters)
    private final double TARGET_HEIGHT = Units.inchesToMeters(28); // may need to change 
    private final double CAMERA_PITCH = Units.degreesToRadians(5); // tilt of our camera (radians)

    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator estimator;

    public Vision() {
        super(DEFAULT_CAM_NAME);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            fieldLayout = null;
        }
        Transform3d robotToCam = new Transform3d(
            new Translation3d(DEFAULT_CAM_X, DEFAULT_CAM_Y, DEFAULT_CAM_Z), new Rotation3d(0, 0, 0)
        );
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, this, robotToCam);
    }

    public double getDistanceToTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, 
                Units.degreesToRadians(getPitch())
            );
            return range;
        }
        return 0.0;
    }

    public Optional<EstimatedRobotPose> getGlobalPose() {
        return estimator.update();
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        return getLatestResult().getBestTarget().getYaw();
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        return getLatestResult().getBestTarget().getPitch();
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        return getLatestResult().getBestTarget().getSkew();
    }

    public double getApriltagID() {
        return getLatestResult().getBestTarget().getFiducialId();
    }
}