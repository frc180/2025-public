package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.Map.Entry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.ReefProximity;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

@Logged
public class VisionSubsystem extends SubsystemBase {

    /**
     * The source of a pose estimate, used to determine the standard deviation of the pose estimate
     * (i.e. how much we trust the pose estimate). The lower the number, the more we trust the pose estimate,
     * and the more it'll affect the robot's position.
     */
    enum PoseEstimateSource {
        SCORING_CAMERA(0.025),
        FRONT_CAMERA(0.15),
        BACK_CAMERA(0.3),
        NONE(99);

        private Matrix<N3, N1> stdDev;

        PoseEstimateSource(double dev) {
            this(dev, dev, dev);
        }

        PoseEstimateSource(double xDev, double yDev, double thetaDev) {
            stdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), xDev, yDev, thetaDev);
        }
    } 

    private static final List<Integer> redTags = List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    private static final List<Integer> blueTags = List.of(12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22);
    public static final List<Integer> redReefTags = List.of(6,7,8,9,10,11);
    public static final List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22);
    public static final List<Integer> allReefTags = new ArrayList<>();

    // Used to correct for horizontal offset of tags on the field
    private static final Map<Integer, Distance> TAG_OFFSETS = Map.of(
        // 6, Inches.of(24)
    );

    public static final Transform3d ROBOT_TO_SCORING_CAMERA = new Transform3d(
        Inches.of(9.757).in(Meters), // forward
        Inches.of(-7.2).in(Meters), // right
        Inches.of(31.296).in(Meters),
        new Rotation3d(0, Units.degreesToRadians(50), 0) // downward tilt
    );

    // ORIGINAL FRONT CAMERA MOUNT (Orlando & South Florida)
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
        Inches.of(13.998 + 3.5 - 1).in(Meters), // forward
        Inches.of(-7 + 1 - 1).in(Meters), // right
        Inches.of(6.767).in(Meters),
        new Rotation3d(0, Units.degreesToRadians(-15), 0) // upward tilt
    );
    
    public static final Transform3d ROBOT_TO_INTAKE_CAMERA = new Transform3d(
        Inches.of(-8.7).in(Meters), // forward
        Inches.of(-5.22).in(Meters), // left
        Inches.of(30.049).in(Meters),
        new Rotation3d(0, Units.degreesToRadians(50), Units.degreesToRadians(180)) // downward tilt
    );

    public static final Transform2d ROBOT_TO_INTAKE_CAMERA_2D = new Transform2d(
        ROBOT_TO_INTAKE_CAMERA.getTranslation().getX(),
        ROBOT_TO_INTAKE_CAMERA.getTranslation().getY(),
        Rotation2d.k180deg
    );
    public static final Transform2d INTAKE_CAMERA_TO_ROBOT_2D = ROBOT_TO_INTAKE_CAMERA_2D.inverse();

    private static final int RED_PROCESSOR_TAG = 3;
    private static final int BLUE_PROCESSOR_TAG = 16;

    private static final double BAD_CAMERA_TEMP = 55;

    private final VisionIO io;
    private final VisionIOInputs inputs;

    private final ReefProximity reefProximity;
    private final CoralDetector coralDetector;
    private final CoralDetectorReal coralDetectorReal;

    private final Distance reefBackDistance = Meters.of(0.55).plus(Inches.of(0.5 - 1));
    private final Distance reefSideDistance = Field.REEF_BRANCH_SEPARATION.div(2); // field measurement based

    private final Transform2d leftReefTransform = new Transform2d(reefBackDistance.in(Meters), -reefSideDistance.in(Meters), Rotation2d.k180deg);
    private final Transform2d rightReefTransform = new Transform2d(reefBackDistance.in(Meters), reefSideDistance.in(Meters), Rotation2d.k180deg);
    private final Transform2d processorTransform = new Transform2d(0.55, 0.0, Rotation2d.fromDegrees(90));

    // The "front lay down"
    private final double l1BackDistance = 0.62 - Inches.of(1.5).in(Meters);

    private final Transform2d leftL1ReefTransform = new Transform2d(l1BackDistance, -Inches.of(12).in(Meters), Rotation2d.k180deg);
    private final Transform2d rightL1ReefTransform = new Transform2d(l1BackDistance, 0, Rotation2d.k180deg);
    
    private final Transform2d leftL1ReefRotation = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
    private final Transform2d rightL1ReefRotation = new Transform2d(0, 0, Rotation2d.fromDegrees(0));

    // 1.25 inches closer (forward) than standard, applied on top of left/right reef transforms
    private final Transform2d algaeReefTransform = new Transform2d(Inches.of(0.75 + 0.5 - 1).in(Meters), 0, Rotation2d.kZero);

    private final Transform2d algaeReefBackup = new Transform2d(-0.3, 0, Rotation2d.kZero);

    private final Pose2d redProcessorPose;
    private final Pose2d blueProcessorPose;

    public final HashMap<Integer, Pose2d> tagPoses2d = new HashMap<>();
    public final HashMap<Integer, Pose2d> leftReefHashMap = new HashMap<>();
    public final HashMap<Integer, Pose2d> rightReefHashMap = new HashMap<>();
    private final HashMap<Integer, Pose2d> leftL1ReefHashMap = new HashMap<>();
    private final HashMap<Integer, Pose2d> rightL1ReefHashMap = new HashMap<>();
    private final HashMap<Integer, Pose2d> leftReefAlgaePoses = new HashMap<>();
    private final HashMap<Integer, Pose2d> rightReefAlgaePoses = new HashMap<>();
    private final HashMap<Integer, Pose2d> leftReefAlgaeBackupPoses = new HashMap<>();
    private final HashMap<Integer, Pose2d> rightReefAlgaeBackupPoses = new HashMap<>();

    public int bestReefID = -1;
    public int lastReefID = -1;

    private Pose2d coralPose = Pose2d.kZero;
    private boolean coralPoseValid = false;
    private Pose2d coralPickupPose = null;

    private boolean closestReefPoseValid = false;
    private Pose2d closestReefPose = Pose2d.kZero;

    private Pose2d singleTagPose = Pose2d.kZero;

    private PoseEstimate poseEstimate = null;
    private PoseEstimateSource poseEstimateSource = PoseEstimateSource.NONE;
    private boolean allowPoseEstimates = true;
    private Pose3d scoringCameraPosition = Pose3d.kZero;
    private Pose3d frontCameraPosition = Pose3d.kZero;
    private Pose3d backCameraPosition = Pose3d.kZero;
    private double poseEstimateDiffX, poseEstimateDiffY, poseEstimateDiffTheta;
    private double lastPoseEstimateTime = 0;
    
    private Alert scoringCameraDisconnectedAlert = new Alert("Scoring Camera disconnected!", AlertType.kError);
    private Alert frontCameraDisconnectedAlert = new Alert("Front Camera disconnected!", AlertType.kError);
    private Alert backCameraDisconnectedAlert = new Alert("Back Camera disconnected!", AlertType.kError);

    private Alert scoringCameraTempAlert = new Alert("", AlertType.kWarning);
    private Alert frontCameraTempAlert = new Alert("", AlertType.kWarning);

    public AprilTagFieldLayout aprilTagFieldLayout;
    public final Trigger poseEstimateDiffLow;
    @NotLogged
    public final Trigger scoringCameraConnected;
    public final Trigger hasPoseEstimates = new Trigger(()-> poseEstimate != null).debounce(0.5);

    @SuppressWarnings("unused")
    public VisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        inputs = new VisionIOInputs();
        if (Robot.isReal()) {
            io = new VisionIOLimelight();
        } else {
            io = new VisionIOPhoton(aprilTagFieldLayout);
        }

        allReefTags.addAll(redReefTags);
        allReefTags.addAll(blueReefTags);

        for (var allianceTags : List.of(redTags, blueTags)) {
            for (int tagID : allianceTags) {
                Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID);
                if (pose3d.isPresent()) {
                    tagPoses2d.put(tagID, pose3d.get().toPose2d());
                }
            }
        }

        // Pre-calculate reef poses for all reef tags
        for (int i : redReefTags) {
            leftReefHashMap.put(i, calculateReefPose(i, true));
            rightReefHashMap.put(i, calculateReefPose(i, false));

            leftL1ReefHashMap.put(i, calculateL1ReefPose(i, true));
            rightL1ReefHashMap.put(i, calculateL1ReefPose(i, false));
        }
        for (int i : blueReefTags) {
            leftReefHashMap.put(i, calculateReefPose(i, true));
            rightReefHashMap.put(i, calculateReefPose(i, false));

            leftL1ReefHashMap.put(i, calculateL1ReefPose(i, true));
            rightL1ReefHashMap.put(i, calculateL1ReefPose(i, false));
        }

        // Pre-calculate reef algae poses
        leftReefHashMap.keySet().forEach(i -> {
            leftReefAlgaePoses.put(i, calculateReefAlgaePose(i, true));
            leftReefAlgaeBackupPoses.put(i, calculateReefAlgaeBackupPose(i, true));
        });
        rightReefHashMap.keySet().forEach(i -> {
            rightReefAlgaePoses.put(i, calculateReefAlgaePose(i, false));
            rightReefAlgaeBackupPoses.put(i, calculateReefAlgaeBackupPose(i, false));
        });
        
        // Pre-calculate processor poses
        redProcessorPose = calculateProcessorPose(false);
        blueProcessorPose = calculateProcessorPose(true);

        reefProximity = new ReefProximity(leftReefHashMap, rightReefHashMap);
        coralDetectorReal = new CoralDetectorReal();
        coralDetector = Robot.isReal() ? coralDetectorReal : new CoralDetectorSim(4.0, true);

        double diffMeters = Inches.of(1.5).in(Meters);
        poseEstimateDiffLow = new Trigger(() -> {
            return Math.abs(poseEstimateDiffX) <= diffMeters && 
                   Math.abs(poseEstimateDiffY) <= diffMeters && 
                   Math.abs(poseEstimateDiffTheta) < 5;
        });
        scoringCameraConnected = new Trigger(() -> inputs.scoringCameraConnected);
    }

    boolean wasEnabled = false;

    Pose2d futureRobotPose;

    @Override
    public void periodic() {
        io.update(inputs);

        scoringCameraDisconnectedAlert.set(!inputs.scoringCameraConnected);
        frontCameraDisconnectedAlert.set(!inputs.frontCameraConnected);
        backCameraDisconnectedAlert.set(!inputs.backCameraConnected);

        // cameraTemperatureAlert(scoringCameraTempAlert, "Scoring", inputs.scoringTemp);
        // cameraTemperatureAlert(frontCameraTempAlert, "Front", inputs.frontTemp);

        // If the scoring camera is connected, use its pose estimate
        if (inputs.scoringCameraConnected) {
            poseEstimate = validatePoseEstimate(inputs.scoringPoseEstimate);
            poseEstimateSource = PoseEstimateSource.SCORING_CAMERA;
        }

        boolean invalidScoring = inputs.scoringPoseEstimate == null || inputs.scoringPoseEstimate.tagCount == 0;

        // If we didn't get a pose estimate from the scoring camera, use the front camera's pose estimate
        if (poseEstimate == null && invalidScoring && inputs.frontCameraConnected) {
            poseEstimateSource = PoseEstimateSource.FRONT_CAMERA;

            if (Robot.isReal()) {
                poseEstimate = validatePoseEstimate(inputs.frontPoseEstimate);
            }
            // if (RobotState.isEnabled() && Robot.isSimulation()) {
            //     poseEstimate = validateMT2PoseEstimate(inputs.frontPoseEstimateMT2);
            // } else {
            //     poseEstimate = validatePoseEstimate(inputs.frontPoseEstimate);
            // }
        }
     
        Pose2d robotPose = null;
        if (poseEstimate != null) {
            RobotContainer.instance.drivetrain.addVisionMeasurement(
                poseEstimate.pose,
                Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds),
                poseEstimateSource.stdDev
            );
            // Calculate the difference between the updated robot pose and the scoring pose estimate, to get an idea
            // of how closely we are tracking the robot's actual position
            robotPose = RobotContainer.instance.drivetrain.getPose();
            // Note - the goal of this if statement is to stop "bad" data from non-scoring cameras from allowing
            // a coral to be scored. Unknown if this is working as intended
            if (poseEstimateSource == PoseEstimateSource.SCORING_CAMERA) {
                poseEstimateDiffX = robotPose.getX() - poseEstimate.pose.getX();
                poseEstimateDiffY = robotPose.getY() - poseEstimate.pose.getY();
                poseEstimateDiffTheta = robotPose.getRotation().getDegrees() - poseEstimate.pose.getRotation().getDegrees();
                lastPoseEstimateTime = Timer.getFPGATimestamp();
            } else {
                poseEstimateDiffX = 99;
                poseEstimateDiffY = 99;
                poseEstimateDiffTheta = 99;
            }
        } else {
            poseEstimateSource = PoseEstimateSource.NONE;
        }

        if (robotPose == null) robotPose = RobotContainer.instance.drivetrain.getPose();

        // calculate scoring camera in 3D space, for previewing in AdvantageScope
        if (Robot.isSimulation()) {
            Pose3d robotPose3d = new Pose3d(RobotContainer.instance.drivetrain.getSimPose());
            scoringCameraPosition = robotPose3d.transformBy(ROBOT_TO_SCORING_CAMERA);
            frontCameraPosition = robotPose3d.transformBy(ROBOT_TO_FRONT_CAMERA);
            backCameraPosition = robotPose3d.transformBy(ROBOT_TO_INTAKE_CAMERA);
        }

        ChassisSpeeds speeds = RobotContainer.instance.drivetrain.getCachedState().Speeds;
        futureRobotPose = robotPose.plus(new Transform2d(speeds.vxMetersPerSecond * 0.3, speeds.vyMetersPerSecond * 0.3, Rotation2d.kZero));

        // Allow targeting opponent's reef tags, which is needed for stealing algae
        // Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(futureRobotPose, Robot.isBlue());
        Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(futureRobotPose, allReefTags);
        if (closestTagAndPose == null) {
            closestReefPose = Pose2d.kZero;
            closestReefPoseValid = false;
        } else {
            closestReefPose = closestTagAndPose.getValue();
            closestReefPoseValid = true;
        }

        Pose2d latencyCompensatedRobotPose;
        if (Robot.isReal()) {
            latencyCompensatedRobotPose = RobotContainer.instance.drivetrain.getBufferPose(inputs.backTimestamp);
        } else {
            latencyCompensatedRobotPose = robotPose;
        }
        coralPose = coralDetector.getCoralPose(latencyCompensatedRobotPose, inputs.backDetections);
        if (coralPose == null) {
            coralPose = Pose2d.kZero;
            coralPoseValid = false;
        } else {
            coralPoseValid = true;
        }
        // Invalidate any previously stored coralPickUpPose - this will be recalculated if getCoralPickupPose() is called
        coralPickupPose = null;

        if (closestTagAndPose == null) {
            bestReefID = -1;
        } else {
            bestReefID = closestTagAndPose.getKey();
        }

        if (bestReefID != -1) lastReefID = bestReefID;
    }

    
    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void resetCoralDetector() {
        coralDetector.reset();
    }

    private Pose2d getReefTagPose(int tagID) {
        Pose2d pose = tagPoses2d.get(tagID);
        if (pose == null) return null;

        if (TAG_OFFSETS.containsKey(tagID)) {
            pose = pose.transformBy(new Transform2d(0, TAG_OFFSETS.get(tagID).in(Meters), Rotation2d.kZero));
        }
        return pose;
    }

    /**
     * Generates the scoring pose of the robot relative to a reef AprilTag. This is used to pre-calculate and store all
     * positions to prevent duplicate object creation. To access these pre-calculated poses, use {@link #getReefPose(int, boolean)}.
     */
    private Pose2d calculateReefPose(int tagID, boolean left) {
        Pose2d pose = getReefTagPose(tagID);
        if (pose == null) return null;

        return pose.transformBy(left ? leftReefTransform : rightReefTransform);
    }

    /**
     * Generates the L1 scoring pose of the robot relative to a reef AprilTag. This is used to pre-calculate and store all
     * positions to prevent duplicate object creation. To access these pre-calculated poses, use {@link #getL1ReefPose(int, boolean)}.
     */
    private Pose2d calculateL1ReefPose(int tagID, boolean left) {
        Pose2d pose = getReefTagPose(tagID);
        if (pose == null) return null;

        return pose.transformBy(left ? leftL1ReefTransform : rightL1ReefTransform)
                   .transformBy(left ? leftL1ReefRotation : rightL1ReefRotation);
    }

    /**
     * Generates the right branch scoring pose of the robot relative to a reef AprilTag, closer than
     * the standard reef pose in order to faciliate grabbing an algae. This is used to pre-calculate and store all
     * positions to prevent duplicate object creation. To access these pre-calculated poses, use {@link #getReefAlgaePose(int)}.
     */
    private Pose2d calculateReefAlgaePose(int tagID, boolean left) {
        return getReefPose(tagID, left).transformBy(algaeReefTransform);
    }

    private Pose2d calculateReefAlgaeBackupPose(int tagID, boolean left) {
        return calculateReefAlgaePose(tagID, left).transformBy(algaeReefBackup);
    }

    /**
     * Generates the scoring pose of the robot relative to the processor AprilTag. This is used to pre-calculate and store all 
     * positions to prevent duplicate object creation. To access these pre-calculated poses, use {@link #getProcessorPose(boolean)}.
     */
    private Pose2d calculateProcessorPose(boolean blue) {
        int tagID = blue ? BLUE_PROCESSOR_TAG : RED_PROCESSOR_TAG;

        Pose2d pose = getReefTagPose(tagID);
        if (pose == null) return null;

        return pose.transformBy(processorTransform);
    }

    /**
     * Returns the scoring pose of the robot relative to the reef AprilTag that was last seen.
     * @param left whether to return the left branch or the right branch scoring pose
     */
    public Pose2d getReefPose(boolean left) {
        return getReefPose(lastReefID, left);
    }

    /**
     * Returns a scoring pose of the robot relative to a reef AprilTag.
     * @param tagID the ID of the reef AprilTag
     * @param left whether to return the left branch or the right branch scoring pose
     */
    public Pose2d getReefPose(int tagID, boolean left) {
        if (left) {
            return leftReefHashMap.get(tagID);
        } else {
            return rightReefHashMap.get(tagID);
        }
    }

    /**
     * Returns the L1 scoring pose of the robot relative to a reef AprilTag.
     * @param tagID the ID of the reef AprilTag
     * @param left whether to return the left  or the right L1 scoring pose
     */
    public Pose2d getL1ReefPose(int tagID, boolean left) {
        if (left) {
            return leftL1ReefHashMap.get(tagID);
        } else {
            return rightL1ReefHashMap.get(tagID);
        }
    }

    /**
     * Returns a right branch scoring pose of the robot relative to a reef AprilTag, closer than 
     * the standard reef pose in order to faciliate grabbing an algae.
     * @param tagID the ID of the reef AprilTag
     */
    public Pose2d getReefAlgaePose(int tagID, boolean left) {
        return (left ? leftReefAlgaePoses : rightReefAlgaePoses).get(tagID);
    }

    public Pose2d getReefAlgaeBackupPose(int tagID, boolean left) {
        return (left ? leftReefAlgaeBackupPoses : rightReefAlgaeBackupPoses).get(tagID);
    }

    /**
     * Returns the scoring pose of the robot relative to the processor.
     * @param blue whether to return the blue processor pose or the red processor pose
     */
    public Pose2d getProcessorPose(boolean blue) {
        return blue ? blueProcessorPose : redProcessorPose;
    }
    
    /**
     * Returns the closest reef scoring pose to the robot (accounting for alliance), or null
     */
    @NotLogged
    public Pose2d getClosestReefPose() {
        return closestReefPoseValid ? closestReefPose : null;
    }

    private final static double BARGE_BLUE_X = 7.28;
    private final static double BARGE_RED_X = FlippingUtil.fieldSizeX - BARGE_BLUE_X;

    public Pose2d getBargePose() {
        Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();

        return new Pose2d(
            Robot.isBlue() ? BARGE_BLUE_X : BARGE_RED_X,
            robotPose.getY(),
            Robot.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg
        );
    }

    public int getReefTagFromPose(Pose2d pose) {
        for (Entry<Integer, Pose2d> entry : leftReefHashMap.entrySet()) {
            if (entry.getValue().equals(pose)) {
                return entry.getKey();
            }
        }
        for (Entry<Integer, Pose2d> entry : rightReefHashMap.entrySet()) {
            if (entry.getValue().equals(pose)) {
                return entry.getKey();
            }
        }
        return -1;
    }

    public boolean isScoringCameraConnected() {
        return inputs.scoringCameraConnected;
    }

    public boolean isFrontCameraConnected() {
        return inputs.frontCameraConnected;
    }

    public boolean isBackCameraConnected() {
        return inputs.backCameraConnected;
    }

    /**
     * Returns the position of the closest detected coral, or null
     */
    @NotLogged
    public Pose2d getCoralPose() {
        return coralPoseValid ? coralPose : null;
    }

    public Pose3d getCoralPose3D() {
        if (!coralPoseValid) return Pose3d.kZero;

        return new Pose3d(coralPose);
    }

    @NotLogged
    public Pose2d getCoralPickupPose() {
        if (coralPickupPose == null && coralPoseValid) {
            Translation2d robotPosition = RobotContainer.instance.drivetrain.getPose().getTranslation();
            Translation2d coralPosition = coralPose.getTranslation();
            double pickupOffset = RobotContainer.instance.coralIntakeReady.getAsBoolean() ? 0.6 : 0.9;
            double centerDistance = robotPosition.getDistance(coralPosition);
            double pickupDistance = centerDistance - pickupOffset;

            Translation2d pickupPosition = robotPosition.interpolate(coralPosition, pickupDistance / centerDistance);
            double radiansToCoral = Math.atan2(coralPosition.getY() - robotPosition.getY(), coralPosition.getX() - robotPosition.getX());

            coralPickupPose = new Pose2d(pickupPosition, new Rotation2d(radiansToCoral + Math.PI));
        }
        return coralPickupPose;
    }

    public PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate) {
        if (poseEstimate == null) return null;

        double tagMin = 1;
        double tagMax = 99;
        double maxDist = poseEstimate.tagCount == 1 ? 3.7 : 6;
        double minArea = poseEstimate.tagCount == 1 ? 0.18 : 0.08;
        if (poseEstimate.tagCount > tagMax || poseEstimate.tagCount < tagMin) return null;
        if (poseEstimate.avgTagArea < minArea) return null;
        if (poseEstimate.avgTagDist > maxDist) return null;

        // Rejected if the pose estimate is too far from the last one
        // if (lastPoseEstimate != null && deltaSeconds <= 0.25) {
        //     double maxReasonableDistance = deltaSeconds * DrivetrainSubsystem.MAX_SPEED;
        //     Translation2d diff = poseEstimate.pose.getTranslation().minus(lastPoseEstimate.pose.getTranslation());
        //     if (!Helpers.withinTolerance(diff, maxReasonableDistance)) return null;
        // }

        return poseEstimate;
    }

    public PoseEstimate validateMT2PoseEstimate(PoseEstimate poseEstimate) {
        if (poseEstimate == null) return null;
        if (poseEstimate.tagCount == 0) return null;
        // if (Math.abs(RobotContainer.instance.drivetrain.getGyroscopeRate()) > 720) return null;

        return poseEstimate;
    }

    @NotLogged
    public boolean reefVisible() {
        boolean isReefVisible = false;
        for (int i = 0; i < inputs.scoringFiducials.length; i++) {
            RawFiducial fiducial = inputs.scoringFiducials[i];
            if (redReefTags.contains(fiducial.id) || blueReefTags.contains(fiducial.id)) {
                isReefVisible = true;
            } else {
                isReefVisible = false;
            }

        }
        return isReefVisible;
    }

    /**
     * Blocks pose estimates from updating the robot's position while this command is running.
     */
    public Command blockPoseEstimates() {
        return Commands.runEnd(
            () -> allowPoseEstimates = false,
            () -> allowPoseEstimates = true
        );
    }

    public void setAllowPoseEstimates(boolean allow) {
        allowPoseEstimates = allow;
    }

    private void cameraTemperatureAlert(Alert alert, String cameraName, double temperature) {
        if (temperature >= BAD_CAMERA_TEMP) {
            int roundedtemp = (int) Math.round(temperature);
            alert.setText(cameraName + " Camera temp high (" + roundedtemp + "°F)");
            alert.set(true);
        } else {
            alert.set(false);
        }
    }
}
