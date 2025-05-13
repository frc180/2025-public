package com.spamrobotics.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Helpers {
    
    public static ChassisSpeeds addChassisSpeeds(ChassisSpeeds a, ChassisSpeeds b) {
        return new ChassisSpeeds(
            capValue(a.vxMetersPerSecond + b.vxMetersPerSecond, DrivetrainSubsystem.MAX_SPEED),
            capValue(a.vyMetersPerSecond + b.vyMetersPerSecond, DrivetrainSubsystem.MAX_SPEED),
            capValue(a.omegaRadiansPerSecond + b.omegaRadiansPerSecond, DrivetrainSubsystem.MAX_ANGULAR_RATE)
        );
    }

    public static ChassisSpeeds addChassisSpeedsOverwrite(ChassisSpeeds a, ChassisSpeeds b) {
        a.vxMetersPerSecond = capValue(a.vxMetersPerSecond + b.vxMetersPerSecond, DrivetrainSubsystem.MAX_SPEED);
        a.vyMetersPerSecond = capValue(a.vyMetersPerSecond + b.vyMetersPerSecond, DrivetrainSubsystem.MAX_SPEED);
        a.omegaRadiansPerSecond = capValue(a.omegaRadiansPerSecond + b.omegaRadiansPerSecond, DrivetrainSubsystem.MAX_ANGULAR_RATE);
        return a;
    }

    public static double capValue(double value, double max) {
        if (Math.abs(value) > max) return max * (value / Math.abs(value));
        return value;
    }

    public static boolean withinTolerance(Pose2d pose, Pose2d targetPose, Distance xDistance, Distance yDistance, Angle angle) {
        Double xMeters = xDistance == null ? null : xDistance.in(Meters);
        Double yMeters = yDistance == null ? null : yDistance.in(Meters);
        Double degrees = angle == null ? null : angle.in(Degrees);
        return withinTolerance(pose, targetPose, xMeters, yMeters, degrees);
    }

    public static boolean withinTolerance(Pose2d pose, Pose2d targetPose, Double xMeters, Double yMeters, Double degrees) {
        // If one or more of the poses don't exist, we can't be within tolerance
        if (pose == null || targetPose == null) {
            return false;
        }
        
        boolean xSatisfied = xMeters == null || Math.abs(pose.getX() - targetPose.getX()) <= xMeters;
        boolean ySatisfied = yMeters == null || Math.abs(pose.getY() - targetPose.getY()) <= yMeters;

        // If we haven't met the x and y criteria, don't bother calculating any further
        if (!xSatisfied || !ySatisfied) {
            return false;
        }

        boolean headingSatisfied;
        if (degrees == null) {
            headingSatisfied = true;
        } else {
            double poseDegrees = MathUtil.inputModulus(pose.getRotation().getDegrees(), -180, 180);
            double targetPoseDegrees = MathUtil.inputModulus(targetPose.getRotation().getDegrees(), -180, 180);
            double degreesDiff = MathUtil.inputModulus(poseDegrees - targetPoseDegrees, -180, 180);
            headingSatisfied = Math.abs(degreesDiff) <= degrees;
        }
        return headingSatisfied;
    }

    // Numbers pulled from https://store.ctr-electronics.com/products/minion-brushless-motor
    public static DCMotor getMinion(int numMotors) {
        return new DCMotor(
            12,
            3.1,
            200.46,
            1.43,
            Units.rotationsPerMinuteToRadiansPerSecond(7200),
            numMotors
        );
    }

    // Numbers pulled from https://wcproducts.com/blogs/wcp-blog/kraken-x44
    public static DCMotor getKrakenX44(int numMotors) {
        return new DCMotor(
            12,
            4.05,
            275,
            1.4,
            Units.rotationsPerMinuteToRadiansPerSecond(7530),
            numMotors
        );
    }

    // Helper methods from https://github.com/frc6995/Robot-2025/blob/main/src/main/java/frc/robot/subsystems/drive/Pathing.java#L18

    public static double velocityTowards(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
        return pointRelativeSpeeds(currentPose, fieldRelativeSpeeds, targetTranslation).vxMetersPerSecond;
    }
    
    public static ChassisSpeeds pointRelativeSpeeds(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, headingTo(currentPose, targetTranslation));
    }
    
    public static Rotation2d headingTo(Pose2d currentPose, Translation2d target) {
        return target.minus(currentPose.getTranslation()).getAngle().minus(Rotation2d.kPi);
    }
}
