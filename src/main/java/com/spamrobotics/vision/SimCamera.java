package com.spamrobotics.vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

/**
 * Simulates a Limelight 4 using PhotonVision.
 */
public class SimCamera {

    static final RawFiducial[] EMPTY_FIDUCIALS = new RawFiducial[0];

    final PhotonCamera camera;
    final PhotonPoseEstimator photonPoseEstimator;

    VisionSystemSim visionSim = null;
    PhotonCameraSim cameraSim = null;

    public SimCamera(String name, Transform3d cameraPosition, AprilTagFieldLayout apriltagLayout) {
        camera = new PhotonCamera(name);
        photonPoseEstimator = new PhotonPoseEstimator(apriltagLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraPosition);

        // Everything past this point is simulation only
        if (Robot.isReal()) return;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(apriltagLayout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        // TODO: set LL4 diagonal FOV instead of horizontal (82)
        cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(45);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, cameraPosition);

        cameraSim.enableDrawWireframe(true);
    }

    PoseEstimate poseEstimate = null;
    RawFiducial[] rawFiducials = null;
    double timestamp = 0;

    public void update() {
        visionSim.update(RobotContainer.instance.drivetrain.getSimPose());

        PhotonPipelineResult latestResult = null;
        EstimatedRobotPose latestEstimate = null;
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> poseEst = photonPoseEstimator.update(result);
            if (poseEst.isPresent()) {
                latestEstimate = poseEst.get();
                latestResult = result;
            }
        }
        List<PhotonTrackedTarget> targets = latestResult != null ? latestResult.getTargets() : null;

        poseEstimate = toPoseEstimate(latestEstimate, targets);
        if (poseEstimate != null) timestamp = poseEstimate.timestampSeconds;
        
        if (targets != null) {
            rawFiducials = toRawFiducials(targets);
        } else {
            rawFiducials = EMPTY_FIDUCIALS;
        }
    }

    public PoseEstimate getPoseEstimate() {
        return poseEstimate;
    }

    public RawFiducial[] getRawFiducials() {
        return rawFiducials;
    }

    /**
     * Converts a PhotonVision EstimatedRobotPose to a Limelight PoseEstimate
     */
    private PoseEstimate toPoseEstimate(EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets) {
        if (estimate == null) return null;

        PoseEstimate est = new PoseEstimate();
        est.pose = estimate.estimatedPose.toPose2d();
        est.timestampSeconds = estimate.timestampSeconds;
        est.tagCount = targets.size();
        est.avgTagDist = targets.stream().mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getDistance(Translation3d.kZero)).average().orElse(0);
        est.avgTagArea = 99; //targets.stream().mapToDouble(t -> t.area).average().orElse(0);
        return est;
    }

    /**
     * Converts a PhotonTrackedTarget list to a Limelight RawFiducials array
     */
    private RawFiducial[] toRawFiducials(List<PhotonTrackedTarget> targets) {
        RawFiducial[] fiducials = new RawFiducial[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            double dist = target.getBestCameraToTarget().getTranslation().getDistance(Translation3d.kZero);

            fiducials[i] = new RawFiducial(
                target.getFiducialId(),
                target.getYaw(),
                target.getPitch(),
                target.area,
                dist,
                dist,
                target.poseAmbiguity
            );
        }
        return fiducials;
    }
}
