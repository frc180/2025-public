package frc.robot.subsystems.vision;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Robot;
import frc.robot.util.LimelightHelpers.RawDetection;

public interface CoralDetector {

    public static final double MARGIN = 0.1;

    public static final double AUTO_BLUE_X = 4.8;
    public static final double AUTO_RED_X = 12.9;
    
    public Pose2d getCoralPose(Pose2d robotPose, RawDetection[] detections);

    public default void reset() {};

    public static boolean isValid(Pose2d coralPose) {
        // Reject coral detections that are outside the field 
        boolean auto = RobotState.isAutonomous();
        double currMargin = auto ? MARGIN : 0;
        if (coralPose.getX() < currMargin || 
            coralPose.getX() > FlippingUtil.fieldSizeX - currMargin || 
            coralPose.getY() < currMargin || 
            coralPose.getY() > FlippingUtil.fieldSizeY - currMargin) {
           return false;
        }

        if (auto) {
            boolean blue = Robot.isBlue();
            if (blue && coralPose.getX() > AUTO_BLUE_X) {
                return false;
            } else if (!blue && coralPose.getX() < AUTO_RED_X) {
                return false;
            }
        }

        return true;
    }
}
