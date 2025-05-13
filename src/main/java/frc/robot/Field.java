package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.HashMap;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

public abstract class Field {

    private static HashMap<Integer, Integer> ALGAE_HEIGHTS, CURRENT_ALGAE_HEIGHTS;
    private static Pose3d[] reefAlgaePoses = null;

    // Game Manual Page 24 - "...pipes on the same face are 1 ft. 1 in. (~33 cm) apart (center to center)."
    public static final Distance REEF_BRANCH_SEPARATION = Inches.of(13);

    // Game Manual Page 33 - "A CORAL is a 11 ⅞ in. long (~30 cm) piece of..."
    public static final Distance CORAL_LENGTH = Centimeters.of(30);


    public static void init() {
        ALGAE_HEIGHTS = new HashMap<>();
        // Red reef
        ALGAE_HEIGHTS.put(6, 2);
        ALGAE_HEIGHTS.put(7, 3);
        ALGAE_HEIGHTS.put(8, 2);
        ALGAE_HEIGHTS.put(9, 3);
        ALGAE_HEIGHTS.put(10, 2);
        ALGAE_HEIGHTS.put(11, 3);

        // Blue reef
        for (int i = 6; i <= 11; i++) {
            ALGAE_HEIGHTS.put(redReefTagToBlue(i), ALGAE_HEIGHTS.get(i));
        }

        CURRENT_ALGAE_HEIGHTS = new HashMap<>(ALGAE_HEIGHTS);
    }

    public static Pose3d[] getReefAlgaePoses() {
        if (reefAlgaePoses == null) {
            AprilTagFieldLayout layout = RobotContainer.instance.vision.aprilTagFieldLayout;
            reefAlgaePoses = new Pose3d[CURRENT_ALGAE_HEIGHTS.size()];
            int index = 0;
            for (var entry : CURRENT_ALGAE_HEIGHTS.entrySet()) {
                int tag = entry.getKey();
                int level = entry.getValue();
                Optional<Pose3d> optionalTagPose = layout.getTagPose(tag);
                if (optionalTagPose.isPresent()) {
                    Pose3d tagPose = optionalTagPose.get();
                    reefAlgaePoses[index] = tagPose.transformBy(new Transform3d(-0.15, 0, level == 3 ? 1 : 0.6, Rotation3d.kZero));
                } else {
                    reefAlgaePoses[index] = Pose3d.kZero;
                }
                index += 1;
            }
        }
        return reefAlgaePoses;
    }
    
    public static boolean hasReefAlgae(int tag) {
        return CURRENT_ALGAE_HEIGHTS.containsKey(tag);
    }

    public static Pose3d getReefAlgaePose(int tag) {
        Pose3d[] poses = getReefAlgaePoses();
        int index = 0;
        for (var entry : CURRENT_ALGAE_HEIGHTS.entrySet()) {
            int algaeTag = entry.getKey();
            if (tag == algaeTag) {
                return poses[index];
            }
            index++;
        }
        return null;
    }

    public static void removeReefAlgae(int tag) {
        CURRENT_ALGAE_HEIGHTS.remove(tag);
        // Reset reefAlgaePoses so it will be recalculated next time it is requested
        reefAlgaePoses = null;
    }

    public static void resetReefAlgae() {
        CURRENT_ALGAE_HEIGHTS = new HashMap<>(ALGAE_HEIGHTS);
        reefAlgaePoses = null;
    }

    public static int getAlgaeLevel(int tag) {
        Integer level = ALGAE_HEIGHTS.get(tag);
        return level != null ? level : -1;
    }

    public static int blueReefTagToRed(int blueTag) {
        switch(blueTag) {
            case 17:
                return 8;
            case 18:
                return 7;
            case 19:
                return 6;
            case 20:
                return 11;
            case 21:
                return 10;
            case 22:
                return 9;
            default:
                return -1;
        }
    }

    public static int redReefTagToBlue(int redTag) {
        switch(redTag) {
            case 6:
                return 19;
            case 7:
                return 18;
            case 8:
                return 17;
            case 9:
                return 22;
            case 10:
                return 21;
            case 11:
                return 20;
            default:
                return -1;
        }
    }
}
