package frc.robot.util.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import java.util.function.Function;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import com.spamrobotics.util.JoystickInputs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedAIRobot extends SubsystemBase {
    /* If an AI robot is not on the field, it is placed in a queening position for performance. */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
    };

    public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
    };

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;

    private XboxController controller = null;
    private Rotation2d zeroHeading = Rotation2d.kZero;
    private JoystickInputs inputs = new JoystickInputs();

    public SimulatedAIRobot(int id) {
        this.id = id;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        this.driveSimulation = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
            drivetrainConfig(), 
            queeningPose
        ));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());

        controller = new XboxController(id + 1);
        this.setDefaultCommand(joystickDrive(controller));
    }

    public void drive(ChassisSpeeds speeds) {
        driveSimulation.runChassisSpeeds(speeds, Translation2d.kZero, false, true);
    }

    public Pose2d getPose() {
        return driveSimulation.getActualPoseInSimulationWorld();
    }

    private Command joystickDrive(XboxController joystick) {
         final Function<Double, Double> axisToLinearSpeed = (axis) -> {
            return axis * driveSimulation.maxLinearVelocity().in(MetersPerSecond);
         };

        // Obtain chassis speeds from joystick input
        final Supplier<ChassisSpeeds> joystickSpeeds = () -> {
            inputs.of(-joystick.getLeftY(), -joystick.getLeftX())
                            .deadband(0.1)
                            .polarDistanceTransform(JoystickInputs.SQUARE_KEEP_SIGN)
                            .clamp(1)
                            .transform(axisToLinearSpeed);

            return new ChassisSpeeds(
                inputs.x,
                inputs.y,
                MathUtil.applyDeadband(-joystick.getRightX(), 0.1) * driveSimulation.maxAngularVelocity().in(RadiansPerSecond)
            );
        };

        return run(() -> {
            if (joystick == null || !joystick.isConnected()) {
                drive(new ChassisSpeeds());
                return;
            }

            Pose2d pose = driveSimulation.getActualPoseInSimulationWorld(); 
            if (joystick.getBackButton()) {
                zeroHeading = pose.getRotation();
            }

            ChassisSpeeds driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(joystickSpeeds.get(), pose.getRotation().minus(zeroHeading));
            drive(driveSpeeds);
        })
        .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
            FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id])
        ));
    }

    private DriveTrainSimulationConfig drivetrainConfig() {
        return DriveTrainSimulationConfig.Default()
                .withRobotMass(Pounds.of(130))
                .withBumperSize(Inches.of(30), Inches.of(30))
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(COTS.ofMark4(
                    DCMotor.getKrakenX60Foc(1),
                    DCMotor.getKrakenX60Foc(1),
                    COTS.WHEELS.SLS_PRINTED_WHEELS.cof,
                    3
                ));
    }
}