// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.StatusSignals;
import frc.robot.util.simulation.SimLogic;
import frc.robot.util.simulation.SimVisuals;
import frc.robot.util.simulation.SimulatedAIRobot;

@Logged
public class Robot extends TimedRobot {

    private static boolean DEMO_MODE = false;

    public static boolean currentlyScoringCoral = false;
    public static boolean justScoredCoral = false;
    public static boolean wasEverEnabled = false;
    private static boolean isBlueAlliance = false;

    private Command m_autonomousCommand;
    private Command partnerPush = null;
    private boolean shouldPartnerPush = false;
    @NotLogged
    private final Trigger nullTrigger = new Trigger(() -> false);
    @NotLogged
    private Trigger didPartnerPush = new Trigger(() -> shouldPartnerPush && !partnerPush.isScheduled());

    @Logged(name = "RobotContainer")
    private final RobotContainer robotContainer;

    private final Alert noAutoAlert = new Alert("Setup - No auto selected!", AlertType.kWarning);
    private final Alert noCoralAlert = new Alert("Setup - No coral detected!", AlertType.kWarning);
    private final Alert wrongSideAutoAlert = new Alert("Setup - Auto side does not match robot position!", AlertType.kError);
    private final Alert controllerDisconnectedAlert = new Alert("Setup - Driver controller not connected!", AlertType.kError);
    private final Alert posingModeAlert = new Alert("Robot is in Posing Mode!", AlertType.kInfo);
    private final Alert demoModeAlert = new Alert("Robot is in Demo Mode!", AlertType.kInfo);

    @Logged(name = "Battery Voltage")
    double batteryVoltage = 0;

    StructArrayPublisher<Pose3d> robotComponentPoses = NetworkTableInstance.getDefault()
                                                    .getStructArrayTopic("Robot Component Poses", Pose3d.struct)
                                                    .publish();

    @NotLogged
    Pose3d[] robotComponentPosesArray = new Pose3d[1];

    private List<SimulatedAIRobot> simulatedAIRobots = new ArrayList<>();

    public Robot() {
        SimVisuals.init();
        Field.init();

        robotContainer = new RobotContainer();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Epilogue.configure(config -> {
            config.minimumImportance = Logged.Importance.DEBUG;
        });
        Epilogue.bind(this);
    }

    @Override
    public void robotInit() {
        // Make sure Pathplanner is loaded and ready to go
        FollowPathCommand.warmupCommand().schedule();
        partnerPush = Auto.partnerPush();
        didPartnerPush.onTrue(Commands.runOnce(() -> m_autonomousCommand.schedule()));

        // Elastic.selectTab("Autonomous");
    }

    @Override
    public void robotPeriodic() {
        isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        robotContainer.drivetrain.clearCache();
        StatusSignals.refreshAll();
        CommandScheduler.getInstance().run();
        SimVisuals.update();

        batteryVoltage = RobotController.getBatteryVoltage();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        noCoralAlert.set(!robotContainer.robotHasCoral.getAsBoolean());

        Command selectedAuto = robotContainer.getAutonomousCommand();
        noAutoAlert.set(selectedAuto == null || selectedAuto == robotContainer.autoDoNothing);

        Pose2d robotPose = robotContainer.drivetrain.getPose();
        boolean robotLeft = (robotPose.getY() > FlippingUtil.fieldSizeY / 2);
        if (Robot.isRed()) robotLeft = !robotLeft;
        boolean autoLeft = selectedAuto.getName().contains("Left");
        wrongSideAutoAlert.set(robotLeft != autoLeft);

        controllerDisconnectedAlert.set(!robotContainer.driverController.isConnected());
        posingModeAlert.set(RobotContainer.POSING_MODE);
        demoModeAlert.set(isDemoMode());
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Auto.init();
        Field.resetReefAlgae();
        if (Robot.isSimulation()) {
            SimulatedArena.getInstance().resetFieldForAuto();
            SimLogic.armHasCoral = true;
            SimLogic.intakeHasCoral = false;
            SimLogic.armHasAlgae = false;
            SimLogic.intakeHasAlgae = false;
            SimLogic.coralScored = 0;
        }
        wasEverEnabled = true;

        if (Robot.isDemoMode()) return;

        shouldPartnerPush = robotContainer.shouldAutoPush();
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            if (shouldPartnerPush) {
                partnerPush.schedule();
            } else {
                m_autonomousCommand.schedule();
            }
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Command currentDrive = robotContainer.drivetrain.getCurrentCommand();
        if (currentDrive != null) {
            currentDrive.cancel();
        }
        robotContainer.vision.setAllowPoseEstimates(true);
        wasEverEnabled = true;
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    StructArrayPublisher<Pose2d> aiRobotPoses = NetworkTableInstance.getDefault()
            .getStructArrayTopic("AI Robot Poses", Pose2d.struct)
            .publish();

    StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Coral Poses", Pose3d.struct)
            .publish();

    StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Algae Poses", Pose3d.struct)
            .publish();

    @Override
    public void simulationInit() {
        if (RobotContainer.MAPLESIM) {
            SimLogic.spawnHumanPlayerCoral(true);
            SimLogic.spawnHumanPlayerCoral(false);
            simulatedAIRobots.add(new SimulatedAIRobot(0));
        }
    }

    @NotLogged
    private final Transform3d robotAlgaeIntakeTransform = new Transform3d(0, 0.2, 0.3, Rotation3d.kZero);

    @Override
    public void simulationPeriodic() {
        RobotContainer rc = robotContainer;
        // Get the positions of all maplesim coral and publish them to NetworkTables
        Pose3d[] coral = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        coralPoses.accept(coral);

        // Get the positions of all algae and publish them to NetworkTables
        Pose3d[] algae = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
        Pose3d[] fieldAlgae = Field.getReefAlgaePoses();
        Pose3d robotAlgae;
        if (rc.elevatorArmAlgae.hasAlgae.getAsBoolean() || rc.intakeAlgae.hasAlgae.getAsBoolean()) {
            Pose3d robotPose = new Pose3d(rc.drivetrain.getSimPose());
            if (rc.intakeAlgae.hasAlgae.getAsBoolean()) {
                robotAlgae = robotPose.transformBy(robotAlgaeIntakeTransform);
            } else {
                Transform3d algaeArmTransform = new Transform3d(0, 0.1, rc.elevator.getPositionMeters() + 0.5, Rotation3d.kZero);
                robotAlgae = robotPose.transformBy(algaeArmTransform);
            }
        } else {
            robotAlgae = Pose3d.kZero;
        }
        Pose3d[] combinedAlgae = new Pose3d[algae.length + fieldAlgae.length + 1];
        System.arraycopy(algae, 0, combinedAlgae, 0, algae.length);
        System.arraycopy(fieldAlgae, 0, combinedAlgae, algae.length, fieldAlgae.length);
        combinedAlgae[algae.length + fieldAlgae.length] = robotAlgae;
        algaePoses.accept(combinedAlgae);

        // Get the positions of all maplesim AI robots and publish them to NetworkTables
        Pose2d[] aiRobotPosesArray = new Pose2d[simulatedAIRobots.size()];
        for (int i = 0; i < simulatedAIRobots.size(); i++) {
            aiRobotPosesArray[i] = simulatedAIRobots.get(i).getPose();
        }
        aiRobotPoses.accept(aiRobotPosesArray);
    }

    // Helper method to simplify checking if the robot is blue or red alliance
    public static boolean isBlue() {
        return isBlueAlliance;
    }

    public static boolean isRed() {
        return !isBlue();
    }

    public static boolean isDemoMode() {
        return DEMO_MODE;
    }
}
