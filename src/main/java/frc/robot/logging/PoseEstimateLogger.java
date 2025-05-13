package frc.robot.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers.PoseEstimate;

@CustomLoggerFor(PoseEstimate.class)
public class PoseEstimateLogger extends ClassSpecificLogger<PoseEstimate> {

    public PoseEstimateLogger() {
        super(PoseEstimate.class);
    }

    @Override
    public void update(EpilogueBackend backend, PoseEstimate estimate) {
        if (estimate == null) {
            backend.log("Exists", false);
            return;
        }
        backend.log("Exists", true);
        backend.log("Tag Count", estimate.tagCount);
        backend.log("Avg Tag Area", estimate.avgTagArea);
        backend.log("Avg Tag Dist", estimate.avgTagDist);
        backend.log("Pose", estimate.pose, Pose2d.struct);
    }
}