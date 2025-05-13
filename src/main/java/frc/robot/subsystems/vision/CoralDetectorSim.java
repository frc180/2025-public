package frc.robot.subsystems.vision;

import org.ironmaple.simulation.SimulatedArena;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers.RawDetection;
import frc.robot.util.simulation.SimLogic;

/**
 * A coral detector that returns poses based off simulation, ignoring any detections passed to it.
 */
public class CoralDetectorSim implements CoralDetector {

    private final double detectionDistance;
    private final boolean useFOV;

    private double fovDegrees = 82; // Limelight 4 FOV

    /**
     * Create a new CoralDetectorSim with the given detection distance.
     * @param detectionMeters The maximum distance in meters the robot can be from the coral and still detect it.
     * @param useFOV Whether to use the robot's field of view to determine if the coral is in sight.
     */
    public CoralDetectorSim(double detectionMeters, boolean useFOV) {
        detectionDistance = detectionMeters;
        this.useFOV = useFOV;
    }

    @Override
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] _detections) {
        if (robotPose == null) return null;

        if (RobotContainer.MAPLESIM) {
            return getCoralPoseMapleSim(robotPose);
        } else {
            return getCoralPoseBasic(robotPose);
        }
    }

    private Pose2d getCoralPoseBasic(Pose2d robotPose) {
        Pose2d coralPose = Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose;

        if (useFOV && poseWithinPOV(robotPose, coralPose)) {
            return coralPose;
        } else if (!useFOV && robotPose.getTranslation().getDistance(coralPose.getTranslation()) <= detectionDistance) {
            return coralPose;
        }

        return null;
    }

    /**
     * Selects the closest coral to the robot within its field of view.
     */
    private Pose2d getCoralPoseMapleSim(Pose2d robotPose) {
        Pose3d[] corals = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        Pose2d bestCoral = null;
        double bestDistance = Double.MAX_VALUE;
        for (int i = 0; i < corals.length; i++) {
            // Filter out upright coral
            if (corals[i].getRotation().getY() > 0.2) continue;

            Pose2d coral = corals[i].toPose2d();
            if (poseWithinPOV(robotPose, coral)) {
                double distance = robotPose.getTranslation().getDistance(coral.getTranslation());
                if (distance < bestDistance) {
                    if (!CoralDetector.isValid(coral)) {
                        continue;
                    }

                    bestCoral = coral;
                    bestDistance = distance;
                }
            }
        }

        if (bestCoral == null) return null;
        double noise = 0.1;
        return bestCoral.transformBy(new Transform2d(randomRange(-noise, noise), randomRange(-noise, noise), Rotation2d.kZero));
    }

    private double randomRange(double min, double max) {
        return Math.random() * (max - min) + min;
    }

    // =========== Helper methods to calculate robot POV ===========

    private boolean poseWithinPOV(Pose2d robotPose, Pose2d coralPose) {
        // Define the triangle vertices based on the robot's pose
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation().rotateBy(Rotation2d.k180deg);

        // Define the field of view (FOV) angle and distance
        double fovAngle = Math.toRadians(fovDegrees);
        double fovDistance = detectionDistance;

        // Calculate the vertices of the triangle
        Translation2d vertex1 = robotTranslation;
        Translation2d vertex2 = robotTranslation.plus(new Translation2d(fovDistance, fovDistance * Math.tan(fovAngle / 2)).rotateBy(robotRotation));
        Translation2d vertex3 = robotTranslation.plus(new Translation2d(fovDistance, -fovDistance * Math.tan(fovAngle / 2)).rotateBy(robotRotation));

        // Check if the coralPose is within the triangle
        return isPointInTriangle(coralPose.getTranslation(), vertex1, vertex2, vertex3);
    }

    private boolean isPointInTriangle(Translation2d pt, Translation2d v1, Translation2d v2, Translation2d v3) {
        double d1 = sign(pt, v1, v2);
        double d2 = sign(pt, v2, v3);
        double d3 = sign(pt, v3, v1);

        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }

    private double sign(Translation2d p1, Translation2d p2, Translation2d p3) {
        return (p1.getX() - p3.getX()) * (p2.getY() - p3.getY()) - (p2.getX() - p3.getX()) * (p1.getY() - p3.getY());
    }
}
