package frc.robot.subsystems.vision;

import com.spamrobotics.vision.SimCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class VisionIOPhoton implements VisionIO {
    final SimCamera scoringCamera, frontCamera;

    public VisionIOPhoton(AprilTagFieldLayout apriltagLayout) {
        scoringCamera = new SimCamera("scoring", VisionSubsystem.ROBOT_TO_SCORING_CAMERA, apriltagLayout);
        frontCamera = new SimCamera("front", VisionSubsystem.ROBOT_TO_FRONT_CAMERA, apriltagLayout);
    }

    @Override
    public void update(VisionIOInputs inputs) {
        inputs.scoringCameraConnected = true;
        inputs.frontCameraConnected = true;
        inputs.backCameraConnected = true;

        inputs.scoringPoseEstimate = scoringCamera.getPoseEstimate();
        if (inputs.scoringPoseEstimate != null) {
            inputs.scoringTimestamp = inputs.scoringPoseEstimate.timestampSeconds;
            // We're not fully simulating the other camera, but we can use the scoring camera's timestamp
            inputs.backTimestamp = inputs.scoringTimestamp;
        }
        inputs.scoringFiducials = scoringCamera.getRawFiducials();
        inputs.frontPoseEstimate = frontCamera.getPoseEstimate();
    }

    @Override
    public void simulationPeriodic() {
        scoringCamera.update();
        frontCamera.update();
    }
}
