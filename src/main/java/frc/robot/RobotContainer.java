/* 
           _   _ ______ __  __  ____  _   _ ______ 
     /\   | \ | |  ____|  \/  |/ __ \| \ | |  ____|
    /  \  |  \| | |__  | \  / | |  | |  \| | |__   
   / /\ \ | . ` |  __| | |\/| | |  | | . ` |  __|  
  / ____ \| |\  | |____| |  | | |__| | |\  | |____ 
 /_/    \_\_| \_|______|_|  |_|\____/|_| \_|______|

 ====================================================

 2025 FRC Team 180 - S.P.A.M.

*/
package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import com.ctre.phoenix.led.Animation;
import com.pathplanner.lib.util.FlippingUtil;
import com.spamrobotics.util.Helpers;
import com.spamrobotics.util.JoystickInputs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveToCoralPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RumbleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotSubsystem;
import frc.robot.subsystems.IntakeCoral.IntakeCoralSubsystem;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberState;
import frc.robot.subsystems.coralIndexer.CoralIndexerSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CoralScoringPosition;
import frc.robot.util.simulation.SimLogic;
import frc.robot.subsystems.elevatorArm.ElevatorArmSubsystem;
import frc.robot.subsystems.elevatorArmAlgae.ElevatorArmAlgaeSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;

@Logged
public class RobotContainer {
    /**
     * Set this to false to disable MapleSim in simulation
     */
    private static final boolean USE_MAPLESIM = true;
    
    public static final boolean POSING_MODE = false;

    public static final boolean MAPLESIM = USE_MAPLESIM && Robot.isSimulation();
    public static final double DEADBAND = 0.025;

    public final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandGenericHID testController = new CommandGenericHID(1);
    private final JoystickInputs inputs = new JoystickInputs();

    private final Telemetry logger = new Telemetry(DrivetrainSubsystem.MAX_SPEED);

    @Logged(name = "Drivetrain")
    public final DrivetrainSubsystem drivetrain;
    @Logged(name = "Vision")
    public final VisionSubsystem vision;
    @Logged(name = "Algae Intake")
    public final IntakeAlgaeSubsystem intakeAlgae;
    @Logged(name = "Algae Intake Pivot")
    public final IntakeAlgaePivotSubsystem intakeAlgaePivot;
    @Logged(name = "Coral Intake")
    public final IntakeCoralSubsystem intakeCoral;
    @Logged(name = "Coral Intake Pivot")
    public final IntakeCoralPivotSubsystem intakeCoralPivot;
    @Logged(name = "Elevator")
    public final ElevatorSubsystem elevator;
    @Logged(name = "Elevator Arm Pivot")
    public final ElevatorArmPivotSubsystem elevatorArmPivot;
    @Logged(name = "Elevator Arm")
    public final ElevatorArmSubsystem elevatorArm;
    @Logged(name = "Elevator Arm Algae")
    public final ElevatorArmAlgaeSubsystem elevatorArmAlgae;
    @Logged(name = "Coral Indexer")
    public final CoralIndexerSubsystem coralIndexer;
    @Logged(name = "Climber")
    public final Climber climber;
    public final LEDSubsystem leds;

    @NotLogged
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    @NotLogged
    private final SendableChooser<Boolean> pushChooser = new SendableChooser<Boolean>();

    public final Command autoDoNothing;

    public boolean climbDeployedBool = false;

    @NotLogged
    private final Trigger falseTrigger = new Trigger(() -> false);
    public Trigger robotHasCoral = falseTrigger;
    public Trigger robotHasAlgae = falseTrigger;
    @NotLogged
    public Trigger coralIntakeTrigger = falseTrigger;
    @NotLogged
    public Trigger coralIntakeReady = falseTrigger;
    public Trigger driverRightReef = falseTrigger;
    public Trigger driverLeftReef = falseTrigger;
    @NotLogged
    public Trigger driverAlgaeDescore = falseTrigger;
    @Logged(name = "Reef - Near")
    public Trigger nearReef = null;
    @Logged(name = "Reef - At")
    public Trigger atReef = null;
    public Trigger strugglingNearReef = falseTrigger;
    public final Trigger climbDeployed = new Trigger(() -> climbDeployedBool);

    // Debug
    public Trigger atReefXY;
    public Trigger atReefAngle;
    public Trigger posingAtReef = falseTrigger;

    public static RobotContainer instance;

    public RobotContainer() {
        instance = this;

        drivetrain = TunerConstants.createDrivetrain();

        vision = new VisionSubsystem();
        intakeAlgae = new IntakeAlgaeSubsystem();
        intakeAlgaePivot = new IntakeAlgaePivotSubsystem();
        intakeCoral = new IntakeCoralSubsystem();
        intakeCoralPivot = new IntakeCoralPivotSubsystem();
        elevator = new ElevatorSubsystem();
        elevatorArmPivot = new ElevatorArmPivotSubsystem();
        elevatorArm = new ElevatorArmSubsystem();
        elevatorArmAlgae = new ElevatorArmAlgaeSubsystem();
        coralIndexer = new CoralIndexerSubsystem();
        climber = new Climber();
        leds = new LEDSubsystem();

        autoDoNothing = Commands.none().withName("Do Nothing");

        Pose2d leftBargeSimStart = new Pose2d(7, 5.5, Rotation2d.fromDegrees(180 + 60));
        Pose2d rightBargeSimStart = new Pose2d(7, FlippingUtil.fieldSizeY - 6.5, Rotation2d.fromDegrees(120));

        Pose2d middleBargeSimStart = new Pose2d(7, FlippingUtil.fieldSizeY / 2, Rotation2d.fromDegrees(180));

        Command leftBargeLeft = Auto.bargeCoralAuto(
            Auto.leftBarge5(true),
            true,
            leftBargeSimStart
        );
        Command leftBargeRight = Auto.bargeCoralAuto(
            Auto.leftBarge5(false),
            true,
            leftBargeSimStart
        );
        Command leftBargeLeftNoFront = Auto.bargeCoralAuto(
            Auto.leftBargeAvoidFront(true),
            true,
            leftBargeSimStart
        );
        Command leftBargeRightNoFront = Auto.bargeCoralAuto(
            Auto.leftBargeAvoidFront(false),
            true,
            leftBargeSimStart
        );
        Command rightBargeLeft = Auto.bargeCoralAuto(Auto.rightBarge5(true), false, rightBargeSimStart);
        Command rightBargeRight = Auto.bargeCoralAuto(Auto.rightBarge5(false), false, rightBargeSimStart);

        Command rightBargeLeftNoFront = Auto.bargeCoralAuto(Auto.rightBargeAvoidFront(true), false, rightBargeSimStart);
        Command rightBargeRightNoFront = Auto.bargeCoralAuto(Auto.rightBargeAvoidFront(false), false, rightBargeSimStart);

        Command middleBargeLeft = Auto.bargeCoralAuto(Auto.middleBarge(true), true, middleBargeSimStart);
        Command middleBargeRight = Auto.bargeCoralAuto(Auto.middleBarge(false), true, middleBargeSimStart);

        autoChooser.setDefaultOption("Do Nothing", autoDoNothing);
        autoChooser.addOption("Left Barge - 1st Left", leftBargeLeft);
        autoChooser.addOption("Left Barge - 1st Right", leftBargeRight);
        autoChooser.addOption("Left Barge No Front - 1st Left", leftBargeLeftNoFront);
        autoChooser.addOption("Left Barge No Front - 1st Right", leftBargeRightNoFront);
        autoChooser.addOption("Right Barge - 1st Left", rightBargeLeft);
        autoChooser.addOption("Right Barge - 1st Right", rightBargeRight);
        autoChooser.addOption("Right Barge No Front - 1st Left", rightBargeLeftNoFront);
        autoChooser.addOption("Right Barge No Front - 1st Right", rightBargeRightNoFront);

        autoChooser.addOption("Middle Barge - 1st Left", middleBargeLeft);
        autoChooser.addOption("Middle Barge - 1st Right", middleBargeRight);

        autoChooser.addOption("Drive straight", drivetrain.run(() -> {
            drivetrain.drive(new ChassisSpeeds(0, 1, 0));
        }));

        SmartDashboard.putData("Auto Mode", autoChooser);

        pushChooser.setDefaultOption("No SPAM Ram", false);
        pushChooser.addOption("Yes, SPAM CAN RAM", true);    
        SmartDashboard.putData("SPAM Ram?", pushChooser);

        SmartDashboard.putData("Climber Coast", intakeAlgaePivot.coastMode());
        SmartDashboard.putData("Climber Brake", intakeAlgaePivot.brakeMode());

        configureBindings();

        if (POSING_MODE) {
            SmartDashboard.putNumber("Posing/Level", 4);
        }
    }

    private void configureBindings() {
        // Robot modes
        final Trigger disabled = RobotModeTriggers.disabled();
        final Trigger teleop = RobotModeTriggers.teleop();
        final Trigger autonomous = RobotModeTriggers.autonomous();
        final Trigger testMode = RobotModeTriggers.test();
        final Trigger demoMode = new Trigger(Robot::isDemoMode);
        final Trigger notDemoMode = demoMode.negate();
        final Trigger algaeMode = driverController.povDown();
        final Trigger coralMode = algaeMode.negate();


        // Driver buttons
        // Coral
        final Trigger driverSpit = driverController.y();
        final Trigger driverIntake = driverController.leftTrigger().and(coralMode);
        // final Trigger driverIntakeHP = driverController.povRight().and(coralMode); // trigger for intaking from the human player station
        final Trigger driverIntakeHP = falseTrigger;
        final Trigger driverL1 = driverController.y().and(coralMode);
        final Trigger driverL2 = driverController.leftBumper().and(coralMode);
        final Trigger driverL3 = driverController.rightTrigger().and(coralMode);
        final Trigger driverL4 = driverController.rightBumper().and(coralMode);
        driverAlgaeDescore = driverController.a().and(coralMode).and(notDemoMode);
        driverLeftReef = driverController.x().and(coralMode);
        driverRightReef = driverController.b().and(coralMode).or(driverAlgaeDescore);
        final Trigger driverAnyReef = driverLeftReef.or(driverRightReef);
        // Algae
        final Trigger driverProcessor = driverController.povUp().and(notDemoMode);               // algaeMode.and(driverController.b());
        final Trigger driverNet = falseTrigger;                                                  // algaeMode.and(driverController.x());
        final Trigger driverNetSlow = algaeMode.and(driverController.x());                       // algaeMode.and(driverController.b());
        final Trigger driverSpitAlgae = algaeMode.and(driverSpit);
        // final Trigger driverIntakeAlgae = algaeMode.and(driverController.leftTrigger());
        final Trigger driverIntakeAlgae = driverController.povRight();
        // Climb
        final Trigger driverReadyClimb = driverController.leftStick().and(driverController.rightStick()).and(notDemoMode); // left/right stick is M1 and M2
        final Trigger driverStartClimb = driverController.start().and(notDemoMode);

        // More complex triggers
        robotHasCoral = intakeCoral.hasCoral.or(elevatorArm.hasPartialCoral);
        robotHasAlgae = intakeAlgae.hasAlgae.or(elevatorArmAlgae.hadAlgae);
        final Trigger justScoredCoral = new Trigger(() -> Robot.justScoredCoral);
        final Trigger drivetrainAvailable = new Trigger(() -> drivetrain.getCurrentCommand() == drivetrain.getDefaultCommand());
        final Trigger scoringCameraDisconnected = vision.scoringCameraConnected.negate();
        final Trigger canDriveToPose = vision.scoringCameraConnected
                                             .and(testMode.negate())
                                             .and(notDemoMode);

        final Trigger targetingL2_3 = driverL2.or(driverL3);
        final Trigger reefAlgaeTarget = driverRightReef.and(targetingL2_3);
        final Trigger targetingL4 = driverL4.or(Auto.targetingLevel(4));

        Trigger generousNearReef = drivetrain.withinTargetPoseTolerance(
            Meters.of(2.5),
            Meters.of(2.5),
            Degrees.of(45)
        );

        Trigger l1_2_3NearReef = targetingL2_3.or(driverL1).and(generousNearReef);

        nearReef = drivetrain.targetingReef().and(
                    drivetrain.withinTargetPoseTolerance(         
                        Meters.of(0.35),
                        Meters.of(0.35),
                        Degrees.of(45)
                    ).or(l1_2_3NearReef)
        );
        
        // Used for right L2 and L1, where the vision target is blocked if we deploy too early
        Trigger almostAtReef = drivetrain.targetingReef().and(drivetrain.withinTargetPoseTolerance(
                        Inches.of(3), 
                        Inches.of(3),
                        Degrees.of(4)
        ));

        atReefXY = drivetrain.withinTargetPoseTolerance(
                        Inches.of(1 * 0.75),
                        Inches.of(1 * 0.75),
                        null
        );

        atReefAngle = drivetrain.withinTargetPoseTolerance(
                        null,
                        null,
                        Degrees.of(2 * 0.75)
        );

        atReef = drivetrain.targetingReef().and(atReefXY).and(atReefAngle);

        strugglingNearReef = nearReef.and(atReef.negate()).debounce(1.5, DebounceType.kRising);

        if (POSING_MODE) {
            posingAtReef = new Trigger(() -> {
                return Helpers.withinTolerance(drivetrain.getPose(), vision.getClosestReefPose(), Inches.of(1 * 0.75), Inches.of(1 * 0.75), Degrees.of(2 * 0.75));
            });
        }

        // Auto triggers
        final Trigger autoCoralIntake = autonomous.and(Auto::isCoralIntaking);


        // Teleop driving

        final double slowElevatorHeight = ElevatorSubsystem.L3.in(Meters);

        final Function<Double, Double> axisToLinearSpeed = (axis) -> {
            axis *= DrivetrainSubsystem.MAX_SPEED;
            // Slow down drivetrain when scoring in barge or elevator is high
            if (driverNet.getAsBoolean() || 
                driverNetSlow.getAsBoolean() ||
                elevator.getPositionMeters() > slowElevatorHeight) {
                    axis *= 0.25;
            }

            // Slow down a little bit when climbing, scoring L1 manually, or in demo mode
            if (climbDeployedBool || driverController.povLeft().getAsBoolean()) {
                axis *= 0.5;
            } else if (Robot.isDemoMode()) {
                axis *= 0.3;
            }
        
            return axis;
         };

        final Supplier<JoystickInputs> joystickInputsSupplier = () -> {
            return inputs.of(-driverController.getLeftY(), -driverController.getLeftX())
                            .deadband(DEADBAND)
                            .polarDistanceTransform(JoystickInputs.SQUARE_KEEP_SIGN)
                            .clamp(1)
                            .transform(axisToLinearSpeed);
        };

        final DoubleSupplier rotationSupplier = () -> {
            return modifyAxis(-driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_RATE;
        };
    
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, joystickInputsSupplier, rotationSupplier));
        driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope));

        // Coral reef auto-aligns
        driverLeftReef.and(canDriveToPose).whileTrue(new DriveToCoralPose(
            () -> vision.lastReefID,
            (tagID) -> {
                if (elevator.isTargetingReefAlgaePosition()) {
                    return vision.getReefAlgaePose(tagID, true);
                } else if (elevator.getTargetPosition() == ElevatorSubsystem.L1) {
                    return vision.getL1ReefPose(tagID, true);
                } else {
                    return vision.getReefPose(tagID, true);
                }
            }
        ).withDynamicTarget(true));

        driverRightReef.and(canDriveToPose).whileTrue(new DriveToCoralPose(
            () -> vision.lastReefID,
            (tagID) -> {
                if (elevator.isTargetingReefAlgaePosition()) {
                    return vision.getReefAlgaePose(tagID, false);
                } else if (elevator.getTargetPosition() == ElevatorSubsystem.L1) {
                    return vision.getL1ReefPose(tagID, false);
                } else {
                    return vision.getReefPose(tagID, false);
                }
            }
        ).withDynamicTarget(true));

        // Ensure we reset if we cancelled mid-score previously
        driverAnyReef.onTrue(Commands.runOnce(() -> Robot.currentlyScoringCoral = false));

        // Driver Coral Intake
        coralIntakeTrigger = driverIntake.and(driverSpit.negate())
                                         .or(autoCoralIntake);
        
        coralIntakeReady = coralIntakeTrigger.and(elevatorArmPivot::isAtReceivingPosition)
                                             .and(elevator::isElevatorInPosition);

        coralIntakeTrigger
            .whileTrue(elevatorArmPivot.receivePosition().alongWith(elevator.stow(), intakeCoralPivot.extend()));

        coralIntakeReady
            .whileTrue(coralIndexer.runSpeed(0.5).alongWith(elevatorArm.intakeAndIndex()));

        coralIntakeReady.and(elevatorArm.hasNoCoral)
            .whileTrue(intakeCoral.runSpeed(1));

            
        driverIntake.and(driverL1).whileTrue(intakeCoral.runSpeed(-1).alongWith(coralIndexer.runSpeed(-1), intakeCoralPivot.stow()));

        //intaking from the human player station 
        driverIntakeHP.whileTrue(elevatorArmPivot.receiveHPposition().alongWith(elevator.setPosition(ElevatorSubsystem.receiveHP), elevatorArm.reverseIntakeAndIndex()))
                        .onFalse(elevatorArmPivot.stowPosition().alongWith(elevator.stow()));

        elevatorArm.setDefaultCommand(elevatorArm.passiveIndex());
        elevatorArmAlgae.setDefaultCommand(elevatorArmAlgae.passiveIndex());
        
        // Notify driver we've intaken a coral
        driverIntake.and(elevatorArm.hasCoral)
            .whileTrue(new RumbleCommand(1));

        //Algae Intake (using left paddle + right trigger)
        driverIntakeAlgae.whileTrue(elevatorArmPivot.lollipopIntakePosition()
                                    .alongWith(elevator.stow(), elevatorArmAlgae.intakeAndIndex(1)))
                         .onFalse(elevatorArmPivot.receivePosition()
                                    .alongWith(elevatorArmAlgae.intakeAndIndex(1).asProxy().withTimeout(1.5)));

        driverIntakeAlgae.and(elevatorArmAlgae.hasAlgae)
                         .whileTrue(new RumbleCommand(1));

        final Command climbDeployedRumble = new ScheduleCommand(new RumbleCommand(1).withTimeout(1));

        Command readyClimb = elevatorArmPivot.climbPosition().alongWith(
                                climber.deploy().andThen(climbDeployedRumble),
                                elevator.climbHeight(),
                                intakeCoralPivot.extremeStow(),
                                Commands.runOnce(() -> climbDeployedBool = true)
                            );

        driverReadyClimb.whileTrue(readyClimb);
        driverStartClimb.whileTrue(climber.climb().alongWith(elevator.climbStowHeight(), elevator.brakeMode()));
        
        climber.hasCage.and(climber.isState(ClimberState.DEPLOYED))
               .whileTrue(new RumbleCommand(1));

        driverProcessor.whileTrue(elevatorArmPivot.processorPosition().alongWith(
                            elevator.stow(),
                            drivetrain.targetHeadingContinuous(270.0, HeadingTarget.GYRO))
                        )
                       .onFalse(elevatorArmPivot.stowPosition().alongWith(
                            drivetrain.targetHeading(null, HeadingTarget.POSE))
                        );

        driverProcessor.and(driverSpit)
                       .and(elevatorArmPivot.isAt(ElevatorArmPivotSubsystem.PROCESSOR))
                       .and(drivetrain.withinTargetHeadingTolerance(5))
                       .whileTrue(elevatorArmAlgae.runSpeed(-0.5));

        driverProcessor.and(elevatorArmAlgae.hasAlgae.negate())
                       .whileTrue(new RumbleCommand(1));

        
        disabled.onTrue(elevatorArmPivot.coastMode());

        teleop.onTrue(Commands.sequence(
            elevatorArmPivot.brakeMode(),
            new RumbleCommand(0.5).withTimeout(0.3)
        ));

        // Deploy bottom roller at start of auto or teleop
        autonomous.or(teleop).onTrue(intakeCoral.runBottomRollerSpeed(-1).withTimeout(1));

        // Stop elevator from slamming into the top if auto ends while elevator is moving up
        autonomous.and(() -> Timer.getFPGATimestamp() - Auto.startTime >= 14.9)
                  .and(() -> !elevator.isElevatorInPosition() && elevator.getTargetErrorMeters() > 0)
                    .onTrue(
                        elevator.stow().alongWith(elevatorArmPivot.receivePosition())
                                .until(teleop.or(disabled))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                    );

        
        final Trigger manualL1 = driverController.povLeft()
                                    .and(driverAnyReef.negate())
                                    .and(elevatorArm.hasEnteringCoral.negate())
                                    .and(elevatorArm.hasPartialCoral);
        final Trigger manualL1Recently = manualL1.debounce(0.75, DebounceType.kFalling);

        // Make the robot point towards the closest side of the reef
        teleop.and(coralMode)
                .and(elevatorArm.hasPartialCoral.or(intakeCoral.hasCoral))
                .and(elevatorArmAlgae.hadAlgae.negate())
                .and(climbDeployed.negate())
                .and(manualL1Recently.negate())
                .and(driverIntakeAlgae.negate())
                .and(notDemoMode)
            .whileTrue(drivetrain.targetHeadingContinuous(() -> {
                Pose2d reefPose = vision.getClosestReefPose();
                return reefPose != null ? reefPose.getRotation().getDegrees() : null;
            }, HeadingTarget.POSE));

        Command chosenElevatorHeight = elevator.run(() -> {
            // In autonomous, read the next coral scoring position from the list to determine the elevator height
            if (RobotState.isAutonomous()) {
                CoralScoringPosition next = Auto.nextCoralScoringPosition();
                if (next == null) {
                    elevator.setPositionDirect(ElevatorSubsystem.STOW);
                    return;
                }
                elevator.setPositionDirect(elevator.levelToPosition(next.level));
                return;
            }

            if (driverAlgaeDescore.getAsBoolean()) {
                int level = Field.getAlgaeLevel(drivetrain.getTargetPoseTag());
                elevator.setPositionDirect(elevator.levelToPosition(level));
                return;
            }

            if (driverL1.getAsBoolean()) {
                elevator.setPositionDirect(ElevatorSubsystem.L1);

            } else if (driverL2.getAsBoolean()) {
                elevator.setPositionDirect(ElevatorSubsystem.L2);

            } else if (driverL3.getAsBoolean()) {
                elevator.setPositionDirect(ElevatorSubsystem.L3);

            } else if (driverL4.getAsBoolean()) {
                elevator.setPositionDirect(ElevatorSubsystem.L4);

            } else {
                elevator.setPositionDirect(ElevatorSubsystem.STOW);
            }
        });

        Trigger autoHasCoral = autonomous.and(elevatorArm.hasPartialCoral);
        Trigger reefDeployAllowed = elevatorArm.hasEnteringCoral.negate()
                                        .and(teleop.or(autoHasCoral));
                                        

        Trigger isScoringCoral = new Trigger(() -> Robot.currentlyScoringCoral);

        // This extra OR statement fixes a bug that caused us to miss L2's when descoring algae
        Trigger rightL2 = driverRightReef.and(driverL2).or(() -> {
            if (driverAlgaeDescore.getAsBoolean()) {
                int level = Field.getAlgaeLevel(drivetrain.getTargetPoseTag());
                return level == 2;
            }
            return false;
        });

        Trigger nearReefModified = nearReef.and(rightL2.negate())
                                            .or(almostAtReef.debounce(0.1));

        Trigger directReefControl = scoringCameraDisconnected.or(testMode).or(demoMode)
                                            .and(driverL1.or(driverL2).or(driverL3).or(driverL4));
        
        Trigger finalReefTrigger = nearReefModified.and(reefDeployAllowed)
                                                    .or(isScoringCoral)
                                                    .or(directReefControl);

        finalReefTrigger
            .whileTrue(chosenElevatorHeight.alongWith(elevatorArmPivot.matchElevatorPreset()))
            .onFalse(elevator.stow().alongWith(elevatorArmPivot.receivePosition()));


        final Trigger targetingAlgae = driverAlgaeDescore.or(() -> {
            if (!RobotState.isAutonomous()) return false;
            
            CoralScoringPosition next = Auto.nextCoralScoringPosition();
            return next != null && next.isAlgae();
        });

        final double algaeGrabSpeed = 1;

        // Algae grab from reef (start intaking algae earlier when going to descore)
        finalReefTrigger.and(targetingAlgae)
            .whileTrue(elevatorArmAlgae.intakeAndIndex(algaeGrabSpeed));

        Trigger l4Advance = targetingL4.and(drivetrain.targetingReef())
                                        .and(generousNearReef)
                                        .and(finalReefTrigger.negate())
                                        .and(reefDeployAllowed)
                                        .and(elevatorArm.hasPartialCoral);

        // Move elevator partially up when approaching reef targeting L4, but not yet at
        // the range where we are near the reef
        l4Advance.whileTrue(elevator.setPosition(ElevatorSubsystem.L4_ADVANCE).alongWith(elevatorArmPivot.matchElevatorPreset()));

        // FOR TUNING - Track exit velocity of coral
        isScoringCoral.and(elevatorArm.hasNoCoral).onTrue(
            Commands.runOnce(() -> {
                SmartDashboard.putNumber("Coral " + (SimLogic.coralScored + 1) + " Eject Velocity", elevatorArm.getVelocity());
            })
        );
 
        BooleanSupplier shouldObtainAlgae = () -> {
            return elevator.isTargetingReefAlgaePosition() && targetingAlgae.getAsBoolean(); 
        };


        Command obtainAlgae = Commands.race(Commands.waitUntil(elevatorArmAlgae.hasAlgae), Commands.waitSeconds(3), Commands.waitUntil(driverAlgaeDescore.negate()))
                                        .andThen(coralEject())
                                        .deadlineFor(elevatorArmAlgae.intakeAndIndex(algaeGrabSpeed).asProxy());

        Command scoringSequence = Commands.either(
            obtainAlgae,
            coralEject(),
            shouldObtainAlgae
        );

        Command backupWithAlgae = new DriveToCoralPose(
            () -> drivetrain.getTargetPoseTag(),
            (tagID) -> vision.getReefAlgaeBackupPose(tagID, false)
        )
        .withPoseTargetType(PoseTarget.STANDARD)
        .until(drivetrain.withinTargetPoseDistance(0.15))
        .withTimeout(1);

        Trigger visionScoreReady = vision.poseEstimateDiffLow.or(scoringCameraDisconnected)
                                                             .or(() -> {
                                                                Distance elevatorTarget = elevator.getTargetPosition();
                                                                boolean blockableTarget = elevatorTarget == ElevatorSubsystem.L1 || elevatorTarget == ElevatorSubsystem.L2;
                                                                return blockableTarget && driverRightReef.getAsBoolean();
                                                             });

        // Coral scoring sequence - kCancelIncoming means nothing else will be able to stop this command until it finishes
        atReef.and(elevator.inReefPosition)
              .and(elevatorArmPivot.elevatorArmInScoringPosition)
              .and(drivetrain.almostStationary)
              .and(visionScoreReady).onTrue(
            Commands.sequence(
                Commands.runOnce(() -> Robot.currentlyScoringCoral = true)
                        .alongWith(drivetrain.runOnce(() -> drivetrain.drive(new ChassisSpeeds())))
                        .alongWith(scoringSequence),
                Commands.either(
                    backupWithAlgae.andThen(coralScoreEnd()),
                    coralScoreEnd(),
                    shouldObtainAlgae
                )
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        justScoredCoral.and(teleop).onTrue(
            new RumbleCommand(1).withTimeout(0.5)
                .alongWith(Commands.runOnce(() -> Robot.justScoredCoral = false))
        );

        strugglingNearReef.and(teleop).whileTrue(
            new RumbleCommand(1)
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.2))
                .repeatedly()
        );


        final Trigger manualL1Ready = elevator.elevatorInScoringPosition.and(elevatorArmPivot.elevatorArmInScoringPosition);
        final Command manualL1Score = l1CoralEject().andThen(Commands.runOnce(() -> Robot.justScoredCoral = true));

        manualL1.whileTrue(elevator.setPosition(ElevatorSubsystem.L1).alongWith(elevatorArmPivot.matchElevatorPreset()))
                .onFalse(Commands.either(
                    manualL1Score,
                    Commands.none(),
                    manualL1Ready
                ).andThen(elevator.stow().alongWith(elevatorArmPivot.receivePosition())));

        // Outtaking algae without trying to score it
        driverSpitAlgae.and(driverProcessor.negate())
                        .whileTrue(elevatorArmPivot.setPosition(ElevatorArmPivotSubsystem.horizontal))
                       .onFalse(elevatorArmPivot.stowPosition());

        driverSpitAlgae.and(driverProcessor.negate())
                        .and(elevatorArmPivot.isAt(ElevatorArmPivotSubsystem.horizontal))
                       .whileTrue(elevatorArmAlgae.runSpeed(-1));

        // Rehome arm
        algaeMode.and(driverController.rightStick()).whileTrue(Commands.sequence(
            Commands.runOnce(() -> elevatorArmPivot.syncAbsolute()),
            new RumbleCommand(1).withTimeout(2) 
        ));

        algaeMode.and(notDemoMode)
                    .whileTrue(drivetrain.targetHeadingContinuous(0.0, HeadingTarget.GYRO))
                    .onFalse(drivetrain.targetHeading(null, HeadingTarget.POSE));

        // Scoring in the net
        final Trigger canNetDeploy = drivetrain.withinTargetHeadingTolerance(5).debounce(0.1)
                                               .or(demoMode);

        driverNetSlow.and(canNetDeploy).whileTrue(
            Commands.parallel(
                elevator.setPosition(ElevatorSubsystem.NET),
                elevatorArmPivot.netScorePosition()
            )
        ).onFalse(Commands.sequence(
            Commands.either(
                elevatorArmAlgae.runSpeed(-1).withTimeout(0.5),
                Commands.none(),
                elevator.elevatorInScoringPosition
            ),
            netPlaceStow()
        ));

        if (leds != null) {
            leds.setDefaultCommand(leds.run(() -> {
                if (POSING_MODE) {
                    if (elevator.isElevatorInPosition() && elevatorArmPivot.isInPosition()) {
                        leds.setAnimation(leds.greenStrobe);
                    } else {
                        leds.setSplitColor(leds.BLUE, leds.RED);
                    }
                    return;
                }

                if (!vision.isScoringCameraConnected()) {
                    leds.setAnimation(leds.rainbow);
                    return;
                }

                if (!vision.isFrontCameraConnected()) {
                    leds.setAnimation(leds.yellowFadeFast);
                    return;
                }

                boolean doClimbAnimation = climbDeployedBool;

                if (RobotState.isDisabled()) {
                    if (doClimbAnimation) {
                        leds.setAnimation(Robot.isBlue() ? leds.blueFlow : leds.redFlow);
                    } else if (!elevatorArmPivot.isHomed()) {
                        leds.setAnimation(leds.greenStrobe);
                    } else if (!vision.hasPoseEstimates.getAsBoolean()) {
                        leds.setAnimation(leds.yellowLarson);
                    } else if (!DriverStation.isDSAttached()) {
                        leds.setAnimation(leds.yellowFade);
                    } else {
                        leds.setAnimation(Robot.isBlue() ? leds.blueFade : leds.redFade);
                    }
                    return;
                }

                if (drivetrain.isTargetingReefPose()) {
                    leds.setAnimation(leds.purpleFade);
                    return;
                }

                if (elevatorArm.hasPartialCoralBool() && !elevatorArm.hasEnteringCoralBool()) {
                    leds.setAnimation(leds.whiteFade);
                    return;
                }

                var primaryColor = Robot.isBlue() ? leds.BLUE : leds.RED;
                var topColor = robotHasAlgae.getAsBoolean() ? leds.ALGAE : primaryColor;
                var bottomColor = primaryColor;
                if (topColor != bottomColor) {
                    leds.setSplitColor(topColor, bottomColor);
                } else {
                    Animation animation;
                    if (doClimbAnimation) {
                        animation = Robot.isBlue() ? leds.blueFlow : leds.redFlow;
                    } else {
                        animation = Robot.isBlue() ? leds.blueTwinkle : leds.redTwinkle;
                    }
                    leds.setAnimation(animation);
                }
            }).ignoringDisable(true));
        }

        // ================= Autonomous Trigger Logic =================

        Auto.intakingState.and(autoCoralIntake).and(intakeCoral.hasCoral)
            .onTrue(Auto.driveToReefWithCoral());

        Command coralTrackingDelay = Commands.either(
            Commands.waitSeconds(0.5),
            Commands.none(),
            () -> Auto.previousCoralScoringPosition.isFrontMiddle()
        );

        Command autoHPDrive = Auto.driveToHPStation().withDeadline(
            coralTrackingDelay.andThen(Commands.waitUntil(() -> vision.getCoralPose() != null))
        );

        Command autoIntakeCoral = Commands.sequence(
            autoHPDrive,
            Auto.driveToCoral()
                .until(intakeCoral.hasCoral) // at this point, the command gets interrupted by the auto coral intake trigger
                .alongWith(Commands.runOnce(() -> Auto.seenCoralThisCycle = true))
        ).alongWith(Auto.startCoralIntake());
    
        justScoredCoral.and(autonomous).and(drivetrainAvailable).onTrue(
            Commands.either(
                autoIntakeCoral,
                Commands.none(),
                () -> Auto.nextCoralScoringPosition() != null
            ).alongWith(Commands.runOnce(() -> {
                Robot.justScoredCoral = false;
                Auto.seenCoralThisCycle = false;
                vision.resetCoralDetector();
            }))
        );

        final Trigger lostCoral = new Trigger(() -> {
            return Auto.seenCoralThisCycle && vision.getCoralPose() == null;
        }).debounce(1, DebounceType.kRising);

        lostCoral.and(autonomous).and(Auto.intakingState).onTrue(Auto.retryIntake());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public boolean shouldAutoPush() {
        return pushChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        return Math.copySign(value * value, value);
    }

    private Command coralEject() {
        return Commands.either(
            Commands.none(),
            coralEjectGuaranteed(),
            () -> {
                if (RobotState.isAutonomous()) return false;

                List<Integer> otherAllianceTags = Robot.isBlue() ? VisionSubsystem.redReefTags : VisionSubsystem.blueReefTags;
                return otherAllianceTags.contains(vision.lastReefID);
            }
        );
    }

    private Command coralEjectGuaranteed() {
        Command coralEject = elevatorArm.runSpeed(0.3).until(elevatorArm.hasNoCoral)
                                        .andThen(Commands.waitSeconds(0.2));

        return Commands.select(Map.of(
                1, l1CoralEject(),
                2, l4CoralEjectFlip(),
                3, coralEject
            ), 
            () -> {
                if (elevator.getTargetPosition() == ElevatorSubsystem.L1) {
                    return 1;
                } else if (elevator.getTargetPosition() == ElevatorSubsystem.L4) {
                    return 2;
                } else {
                    return 3;
                }
            }
        );
    }

    private Command l1CoralEject() {
        return elevatorArm.runSpeed(0.3).until(elevatorArm.hasNoCoral)
                          .andThen(Commands.waitSeconds(0.5));
    }

    private static final double L4_EJECT_SPEED = 0.27;

    // Move the arm pivot up slightly before retracting helps
    // flip bad L4 corals onto the branch
    private Command l4CoralEjectFlip() {
        return elevatorArm.runSpeed(L4_EJECT_SPEED).until(elevatorArm.hasNoCoral)
                          .andThen(Commands.waitSeconds(0.1))
                          .andThen(Commands.waitSeconds(0.15).deadlineFor(elevatorArmPivot.runSpeed(0.25)));
    }

    private Command coralScoreEnd() {
        return Commands.runOnce(() -> {
            elevator.setPositionDirect(ElevatorSubsystem.STOW);
            elevatorArmPivot.setArmPositionDirect(ElevatorArmPivotSubsystem.receiving);
            Robot.currentlyScoringCoral = false;
            Robot.justScoredCoral = true;

            if (RobotState.isAutonomous()) {
                if (!Auto.coralScoringPositions.isEmpty()) {
                    Auto.previousCoralScoringPosition = Auto.coralScoringPositions.get(0);
                    Auto.coralScoringPositions.remove(0);
                    Auto.firstCoralCycle = false;
                }
            }
            SimLogic.coralScored++;
            SmartDashboard.putNumber("Coral/" + SimLogic.coralScored, Timer.getFPGATimestamp() - Auto.startTime);
            if (Robot.isSimulation()) {
                SimLogic.spawnHumanPlayerCoral();
                SimLogic.intakeHasCoral = false;
            }
        });
    }

    /**
     * Stows the elevator and arm pivot after placing in the net. The arm pivot begins slightly before the elevator stow to
     * reduce the chance of the arm catching on the net if we were too close.
     */
    private Command netPlaceStow() {
        return elevatorArmPivot.stowPosition().alongWith(Commands.waitSeconds(0.1).andThen(elevator.stow()));
    }
}
