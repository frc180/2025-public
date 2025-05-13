package frc.robot.commands;

import java.util.function.Function;
import java.util.function.Supplier;
import com.spamrobotics.util.Helpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;

public class DriveToPose extends Command {
  
    private final DrivetrainSubsystem drivetrain;
    private final ChassisSpeeds noneSpeeds = new ChassisSpeeds(0, 0, 0);

    private Supplier<Pose2d> targetPoseSupplier = null;
    private Function<Integer, Pose2d> tagToPoseFunction = null;
    private Pose2d currentPose = null;
    private Pose2d targetPose = null;
    private Supplier<ChassisSpeeds> additionalSpeedsSupplier = null;
    private Double xEndTolerance = null;
    private Double yEndTolerance = null;
    private Double headingEndTolerance = null;
    private boolean dynamicTarget = false;
    private boolean holdWithinTolerance = false;
    private Supplier<Boolean> finishCriteria = null;
    private PoseTarget poseTargetType = PoseTarget.STANDARD;
    private double maxSpeed = 1.0;
    private Supplier<Integer> targetPoseTagSupplier = null;
    private Function<Pose2d, Pose2d> intermediatePoses = null;
    private TrapezoidProfile profileOverride = null;
    private TrapezoidProfile.Constraints constraintsOverride = null;

    private int targetPoseTag = -1;

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Function<Integer, Pose2d> tagToPoseFunction) {
        drivetrain = drivetrainSubsystem;
        this.tagToPoseFunction = tagToPoseFunction;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        drivetrain = drivetrainSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d targetPose) {
        drivetrain = drivetrainSubsystem;
        this.targetPose = targetPose;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToPose withXEndTolerance(double toleranceMeters) {
        this.xEndTolerance = toleranceMeters;
        return this;
    }

    public DriveToPose withYEndTolerance(double toleranceMeters) {
        this.yEndTolerance = toleranceMeters;
        return this;
    }

    public DriveToPose withXYEndTolerance(double toleranceMeters) {
        return withXEndTolerance(toleranceMeters).withYEndTolerance(toleranceMeters);
    }

    public DriveToPose withHeadingEndTolerance(double toleranceDegrees) {
        this.headingEndTolerance = toleranceDegrees;
        return this;
    }

    public DriveToPose withAdditionalSpeeds(Supplier<ChassisSpeeds> additionalSpeedsSupplier) {
        this.additionalSpeedsSupplier = additionalSpeedsSupplier;
        return this;
    }

    public DriveToPose withDynamicTarget(boolean dynamicTarget) {
        this.dynamicTarget = dynamicTarget;
        return this;
    }

    public DriveToPose withFinishCriteria(Supplier<Boolean> finishCriteria) {
        this.finishCriteria = finishCriteria;
        return this;
    }
 
    public DriveToPose withHoldWithinTolerance(boolean stopWithinTarget) {
        this.holdWithinTolerance = stopWithinTarget;
        return this;
    }

    public DriveToPose withPoseTargetType(PoseTarget target) {
        poseTargetType = target;
        return this;
    }

    public DriveToPose withMaxSpeed(double maxSpeedPercentage) {
        maxSpeed = maxSpeedPercentage;
        return this;
    }

    public DriveToPose withTargetPoseTag(Supplier<Integer> targetPoseTagSupplier) {
        this.targetPoseTagSupplier = targetPoseTagSupplier;
        return this;
    }

    public DriveToPose withIntermediatePoses(Function<Pose2d, Pose2d> intermediatePoses) {
        this.intermediatePoses = intermediatePoses;
        return this;
    }

    public DriveToPose withProfileOverride(TrapezoidProfile profile, TrapezoidProfile.Constraints constraints) {
        profileOverride = profile;
        constraintsOverride = constraints;
        return this;
    }

    @Override
    public void initialize() {
        targetPoseTag = targetPoseTagSupplier != null ? targetPoseTagSupplier.get() : -1;

        drivetrain.setTargetPoseTag(targetPoseTag);
        drivetrain.setPoseTargetType(poseTargetType);
        drivetrain.resetPIDs(HeadingTarget.POSE);

        if (targetPoseSupplier != null) {
            targetPose = targetPoseSupplier.get();
        }
        if (tagToPoseFunction != null) {
            targetPose = tagToPoseFunction.apply(targetPoseTag);
        }
        drivetrain.setTargetPose(targetPose);
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getPose();

        if (dynamicTarget) {
            if (targetPoseSupplier != null) targetPose = targetPoseSupplier.get();
            if (tagToPoseFunction != null) targetPose = tagToPoseFunction.apply(targetPoseTag);  
        }
        drivetrain.setTargetPose(targetPose);
        Pose2d iterationTarget = targetPose;
        
        if (intermediatePoses != null) {
            iterationTarget = intermediatePoses.apply(targetPose);
        }

        if (holdWithinTolerance) {
            boolean inTolerances = Helpers.withinTolerance(currentPose, iterationTarget, xEndTolerance, yEndTolerance, headingEndTolerance);
            if (inTolerances) {
                drivetrain.drive(noneSpeeds);
                return;
            }
        }

        ChassisSpeeds speeds;
        if (profileOverride != null) {
            speeds = drivetrain.driveProfiled(currentPose, iterationTarget, profileOverride, constraintsOverride);
        } else {
            speeds = drivetrain.driveProfiled(currentPose, iterationTarget);
        }
        if (additionalSpeedsSupplier != null) {
            Helpers.addChassisSpeedsOverwrite(speeds, additionalSpeedsSupplier.get());
        }
        double maxSpeedMeters = DrivetrainSubsystem.MAX_SPEED * maxSpeed;
        speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -maxSpeedMeters, maxSpeedMeters);
        speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -maxSpeedMeters, maxSpeedMeters);

        drivetrain.driveClosedLoop(speeds);
    }

    @Override
    public boolean isFinished() {
        // If the desired behavior is to hold position within the tolerance window, instead
        // of exiting the command, then we should not ever exist by ourselves.
        if (holdWithinTolerance) {
            return false;
        }
        // If no end tolerances are set, then the command will never finish on its own
        if (xEndTolerance == null && yEndTolerance == null && headingEndTolerance == null && finishCriteria == null) {
            return false;
        }

        boolean finishCriteriaSatisfied = finishCriteria == null || finishCriteria.get();
        boolean inTolerances = Helpers.withinTolerance(currentPose, targetPose, xEndTolerance, yEndTolerance, headingEndTolerance);

        return inTolerances && finishCriteriaSatisfied;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(noneSpeeds);
        drivetrain.setTargetPose(null);
        drivetrain.setPoseTargetType(PoseTarget.STANDARD);
        drivetrain.setTargetHeading(null);
    }
}
