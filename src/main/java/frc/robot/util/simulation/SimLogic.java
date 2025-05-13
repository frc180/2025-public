package frc.robot.util.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public abstract class SimLogic {

    public static final Pose2d redHPCoralPose = new Pose2d(16.17, 1.33, new Rotation2d());
    public static final Pose2d blueHPCoralPose = FlippingUtil.flipFieldPose(redHPCoralPose);
    public static final double CORAL_LENGTH = Field.CORAL_LENGTH.in(Meters);

    public static boolean intakeHasCoral = false;
    public static boolean armHasCoral = false;
    public static boolean intakeHasAlgae = false;
    public static boolean armHasAlgae = false;
    public static int coralScored = 0;

    public static double armCoralPosition = -1;

    public static boolean robotHasCoral() {
        return intakeHasCoral || armHasCoral;
    }

    public static boolean robotHasAlgae() {
        return intakeHasAlgae || armHasAlgae;
    }

    public static void spawnHumanPlayerCoral() {
        spawnHumanPlayerCoral(Robot.isBlue());
    }

    public static void spawnHumanPlayerCoral(boolean blue) {
        if (!RobotContainer.MAPLESIM) {
            return;
        }

        for (int i = 0; i < 2; i++) {
            Pose2d coralPose = blue ? blueHPCoralPose : redHPCoralPose;
            
            if (i == 1) {
                coralPose = coralPose.transformBy(new Transform2d(0, 5, new Rotation2d()));
            }

            // generate a random physical offset between -0.3 and 0.3 meters and a random rotation
            double xOffset = randomNumberPlusMinus(0.3);
            double yOffset = randomNumberPlusMinus(0.3);
            double rotationOffset = Math.random() * 360;
            Transform2d randomTransform = new Transform2d(xOffset, yOffset, Rotation2d.fromDegrees(rotationOffset));

            spawnCoral(coralPose.transformBy(randomTransform));
        }
    }

    public static void spawnCoral(Pose2d pose) {
        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(pose));
    }

    public static void scoreCoral() {
        if (!RobotContainer.MAPLESIM) {
            return;
        }

        RobotContainer rc = RobotContainer.instance;
        SwerveDriveSimulation swerveSim = rc.drivetrain.getDriveSim();
        Pose2d simRobotPose = swerveSim.getSimulatedDriveTrainPose();
        double coralAngle;
        double heightOffset = 0.6;
        double xOffset;
        if (rc.elevator.getTargetPosition() == ElevatorSubsystem.L4) {
            coralAngle = -90;
            xOffset = 0.6;
        } else {
            coralAngle = rc.elevatorArmPivot.getDegrees();
            xOffset = 0.4;
        }
        Distance coralHeight = Meters.of(rc.elevator.getPositionMeters() + heightOffset);
    
        SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
            simRobotPose.getTranslation(),
            // The scoring mechanism position on the robot
            new Translation2d(xOffset, 0),
            swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            simRobotPose.getRotation(),
            coralHeight,
            // The initial speed of the coral
            MetersPerSecond.of(2),
            Degrees.of(coralAngle))
        );
    }

    public static void outtakeAlgae() {
        if (!RobotContainer.MAPLESIM) {
            return;
        }

        SwerveDriveSimulation swerveSim = RobotContainer.instance.drivetrain.getDriveSim();
        Pose2d simRobotPose = swerveSim.getSimulatedDriveTrainPose();
    
        SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeAlgaeOnFly(
            simRobotPose.getTranslation(),
            new Translation2d(0.6, 0), // scoring mechanism position on the robot
            swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            simRobotPose.getRotation().rotateBy(Rotation2d.kCCW_90deg),
            Inches.of(16), // outtake height
            MetersPerSecond.of(2), // outtake speed
            Degrees.of(0))
        );
    }

    public static void netAlgae(boolean forwards) {
        if (!RobotContainer.MAPLESIM) {
            return;
        }

        SwerveDriveSimulation swerveSim = RobotContainer.instance.drivetrain.getDriveSim();
        Pose2d simRobotPose = swerveSim.getSimulatedDriveTrainPose();
    
        SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeAlgaeOnFly(
            simRobotPose.getTranslation(),
            new Translation2d(-0.1, 0), // scoring mechanism position on the robot
            swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            simRobotPose.getRotation().rotateBy(forwards ? Rotation2d.kZero : Rotation2d.k180deg),
            ElevatorSubsystem.NET, // outtake height
            MetersPerSecond.of(6), // outtake speed
            Degrees.of(75))
        );
    }

    private static double randomNumberPlusMinus(double range) {
        return Math.random() * (range * 2) - range;
    }
}
