package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import java.util.function.Function;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Helper class to simplify generating DriveToPose commands for coral scoring positions.
 */
public class DriveToCoralPose extends DriveToPose {

    /**
     * Create a new DriveToCoralPose command.
     * @param tagSupplier Supplier for the ID of the AprilTag to target.
     * @param tagToPoseFunction Function to convert an AprilTag ID to a Pose2d.
     */
    public DriveToCoralPose(Supplier<Integer> tagSupplier, Function<Integer, Pose2d> tagToPoseFunction) {
        super(RobotContainer.instance.drivetrain, tagToPoseFunction);
        withPoseTargetType(PoseTarget.REEF);
        withTargetPoseTag(tagSupplier);
    }

    final static double ALGAE_OFFSET = -Inches.of(6).in(Meters);

    public static Function<Pose2d, Pose2d> ALGAE_INTERMEDIATE = (Pose2d target) -> {
        if (!RobotContainer.instance.driverAlgaeDescore.getAsBoolean()) {
            return target;
        }

        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        double offset = 0;
        if (!elevator.isElevatorInScoringPosition()) {
            offset = ALGAE_OFFSET;
        }

        if (offset != 0) {
            target = target.transformBy(new Transform2d(offset, 0, Rotation2d.kZero));
        }

        return target;
    };

    private static final double Y_DIAGONAL_THRESHOLD = Inches.of(6).in(Meters);
    private static final double X_DIAGONAL_THRESHOLD = Meters.of(1.3).in(Meters);

    public static final Function<Pose2d, Pose2d> AVOID_BIG_DIAGONAL = (Pose2d target) -> {
        double frontOffset = 0;
        double sideOffset = 0;
        
        Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();
        double xDiff = robotPose.getX() - target.getX();
        double yDiff = robotPose.getY() - target.getY();
        if (Math.abs(yDiff) > Y_DIAGONAL_THRESHOLD && Math.abs(xDiff) > X_DIAGONAL_THRESHOLD) {
            sideOffset = yDiff;
        }

        if (frontOffset != 0 || sideOffset != 0) {
            target = new Pose2d(new Translation2d(target.getX() + frontOffset, target.getY() + sideOffset), target.getRotation());
        }

        return target;
    };

    public static final Function<Pose2d, Pose2d> AVOID_FAR_DIAGONAL = (Pose2d target) -> {
        double frontOffset = 0;
        double sideOffset = 0;
        
        Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();
        double xDiff = robotPose.getX() - target.getX();
        double yDiff = robotPose.getY() - target.getY();

        boolean far = Robot.isBlue() ? xDiff > 0 : xDiff < 0;

        if (far) {
            sideOffset = yDiff;
        }

        if (frontOffset != 0 || sideOffset != 0) {
            target = new Pose2d(new Translation2d(target.getX() + frontOffset, target.getY() + sideOffset), target.getRotation());
        }

        return target;
    };

    final static double reefMarginAdjust = 0.5;
    final static double reefMinY = 2.25 - reefMarginAdjust;
    final static double reefMaxY = 5.79 + reefMarginAdjust;


    public static final Function<Pose2d, Pose2d> AVOID_REEF_Y = avoidReefY(0.6);
    public static final Function<Pose2d, Pose2d> AVOID_REEF_Y_TIGHT = avoidReefY(0.2);

    public static Function<Pose2d, Pose2d> avoidReefY(double xThreshold) {
        return (Pose2d target) -> {
            double sideOffset = 0;
            
            Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();
            double xDiff = robotPose.getX() - target.getX();
            if (Math.abs(xDiff) > xThreshold) {

                // Check if targetY is within reef bounds
                if (target.getY() > reefMinY && target.getY() < reefMaxY) {
                    // if targetY is within reef bounds, move to the edge of the reef
                    if (target.getY() < reefMinY + (reefMaxY - reefMinY) / 2) {
                        sideOffset = reefMinY - target.getY();
                    } else {
                        sideOffset = reefMaxY - target.getY();
                    }
                }
            }

            if (sideOffset != 0) {
                target = new Pose2d(new Translation2d(target.getX(), target.getY() + sideOffset), target.getRotation());
            }

            return target;
        };
    }
}
