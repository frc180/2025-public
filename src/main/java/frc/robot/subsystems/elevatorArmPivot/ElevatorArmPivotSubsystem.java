package frc.robot.subsystems.elevatorArmPivot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotIO.ElevatorArmPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class ElevatorArmPivotSubsystem extends SubsystemBase {

    private final ElevatorArmPivotIO io;
    private final ElevatorArmPivotIOInputs inputs;

    protected static final Angle FORWARD_LIMIT = Degrees.of(60.7);
    protected static final Angle REVERSE_LIMIT = Degrees.of(-44.6);

    protected static final Angle HARD_STOP_STARTING_ANGLE = Degrees.of(61.61);

    public static final double L4_ADVANCE = Units.degreesToRotations(30);
    public static final double L4_SCORE = Units.degreesToRotations(-16);
    public static final double L3_SCORE = Units.degreesToRotations(-6 - 2);
    public static final double L2_SCORE = L3_SCORE;
    public static final double L1_SCORE = Units.degreesToRotations(-16.7);
    
    public static final double receiving = Units.degreesToRotations(46.4);
    public static final double receivingHP = Units.degreesToRotations(42.1);
    public static final double algaeReceive = Units.degreesToRotations(-70);
    public static final double horizontal = 0;
    public static final double PROCESSOR = Units.degreesToRotations(-40);
    public static final double LOLLIPOP_INTAKE = Units.degreesToRotations(-35);
    public static final double CLIMB = Units.degreesToRotations(-37.2);

    public static final double netScore = Units.degreesToRotations(8 - 2);
    public static final double netScoreBackwards = Units.degreesToRotations(46.01);

    private static final double IN_POSITION_TOLERANCE = Units.degreesToRotations(0.6);

    private final Alert notHomedAlert = new Alert("Arm Pivot is not homed!", AlertType.kWarning);

    private Double targetPositionMax = null;
    private Double targetPositionMin = null;

    private double targetPosition = 0;
    private boolean homed = false;
    private boolean absoluteSyncAllowed = true;

    private double absoluteScalar = Robot.isReal() ? 1.03612754169595125 : 4;
    private double absoluteOffset = Robot.isReal() ? -0.641 : 30 * 4;

    private double lastPositionSyncTime = 0;
    private double absoluteRatio = 0;
    private double absoluteRatioFiltered = 0;
    private double absoluteRatioSamples = 0;
    private final MedianFilter absoluteRatioFilter = new MedianFilter(15);

    @NotLogged
    public Trigger elevatorArmInPosition = new Trigger(() -> isInPosition());
    @NotLogged
    public Trigger elevatorArmInScoringPosition = new Trigger(() -> isElevatorArmInScoringPosition());

    private Alert unexpectedStartPositionAlert = new Alert("Arm Pivot sensor not in expected start position!", AlertType.kWarning);

    public ElevatorArmPivotSubsystem() {
        inputs = new ElevatorArmPivotIOInputs();
        io = new ElevatorArmPivotIOTalonFX();

        if (RobotContainer.POSING_MODE) {
            setDefaultCommand(matchElevatorPreset().ignoringDisable(true));
        }
    }

    @NotLogged
    boolean firstPeriodic = true;
    @NotLogged 
    boolean wasEverEnabled = false;
    boolean absPosValid = false;

    @Override
    public void periodic() {
        boolean enabled = RobotState.isEnabled();
        io.update(inputs);
        SimVisuals.setElevatorArmPivotDegrees(getDegrees());

        notHomedAlert.set(!homed);

        absoluteRatio = inputs.position / getAbsolutePosition();
        absoluteRatioFiltered = absoluteRatioFilter.calculate(absoluteRatio);

        double currentTime = Timer.getFPGATimestamp();
        boolean firstDisable = !enabled && !wasEverEnabled;
        boolean disableSync = firstDisable && !RobotContainer.POSING_MODE && currentTime - lastPositionSyncTime > 0.5;
        boolean shouldSync = firstPeriodic || disableSync;

       absPosValid = getAbsolutePosition() < 0.25;

        if (absoluteSyncAllowed && shouldSync && absPosValid) {
            syncAbsolute();
            lastPositionSyncTime = currentTime;
        } else if (!absoluteSyncAllowed && shouldSync) {
            io.zero(HARD_STOP_STARTING_ANGLE.in(Rotations));
            homed = true;
            lastPositionSyncTime = currentTime;
        }

        if (!enabled && !wasEverEnabled) {
            unexpectedStartPositionAlert.set(Math.abs(getAbsolutePosition() - .165) > .005);
        }

        if (firstPeriodic) firstPeriodic = false;
        if (!wasEverEnabled) wasEverEnabled = enabled;
    }

    public void syncAbsolute() {
        if (!absoluteSyncAllowed) return;

        io.zero(getAbsolutePosition());
        homed = true;
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command receivePosition() {
        return setPosition(receiving);
    }

    public Command stowPosition() {
        return setPosition(receiving);
    }

    public Command horizontalPosition() {
        return setPosition(horizontal);
    }

    public Command netScorePosition() {
        return setPosition(netScore);
    }

    public Command netScoreBackwardsPosition() {
        return setPosition(netScoreBackwards);
    }

    public Command receiveAlgaePosition() {
        return setPosition(algaeReceive);
    }

    public Command processorPosition() {
        return setPosition(PROCESSOR);
    }
    public Command receiveHPposition() {
        return setPosition(receivingHP);
    }

    public Command lollipopIntakePosition() {
        return setPosition(LOLLIPOP_INTAKE);
    }

    public Command climbPosition() {
        return setPosition(CLIMB);
    }

    public Command calculateAbsoluteRatio() {
        return Commands.sequence(
            runOnce(() -> {
                io.zero(0);
                zeroAbsolute();
                absoluteRatioFiltered = 0;
                absoluteRatioSamples = 0;
                absoluteRatioFilter.reset();
            })
        ).ignoringDisable(true);
    }

    public Command zero(Angle angle) {
        return zero(angle.in(Rotations));
    }

    public Command zero(double rotations) {
        return runOnce(() -> io.zero(rotations));
    }

    @NotLogged
    private final double MAX_ERROR_THRESHOLD = Inches.of(20).in(Meters);

    public Command matchElevatorPreset() {
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        return run(() -> {
            Distance elevatorTarget = elevator.getTargetPosition();

            if (elevatorTarget == ElevatorSubsystem.L4_ADVANCE) {
                setArmPositionDirect(L4_ADVANCE);
            } else if (elevatorTarget == ElevatorSubsystem.L4) {
                setArmPositionDirect(L4_SCORE);
            } else if (elevatorTarget == ElevatorSubsystem.L3) {
                setArmPositionDirect(L3_SCORE);
            } else if (elevatorTarget == ElevatorSubsystem.L2) {
                setArmPositionDirect(L2_SCORE);
            } else if (elevatorTarget == ElevatorSubsystem.L1) {
                setArmPositionDirect(L1_SCORE);
            } else {
                setArmPositionDirect(receiving);
            }
        });
    }

    public Command setPosition(double position) {
        return run(() -> setArmPositionDirect(position));
    }

    public Command setSpeed(double speed) {
        return run(() -> {
            io.setSpeed(speed);
        });
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> {
                io.setSpeed(speed);
            },
            () -> io.stopMotor()
        );
    }

    public Command stop() {
        return run(() -> {
            io.stopMotor();
        });
    }

    public Command brakeMode() {
        return Commands.runOnce(io::brakeMode).ignoringDisable(true);
    }

    public Command coastMode() {
        return Commands.runOnce(io::coastMode).ignoringDisable(true);
    }

    public Trigger isTargeting(double target) {
        return new Trigger(() -> targetPosition == target);
    }

    public Trigger isAt(double target) {
        return new Trigger(() -> targetPosition == target && isInPosition());
    }

    public void setPositionLimits(Double min, Double max) {
        targetPositionMin = min;
        targetPositionMax = max;
    }

    public void setArmPositionDirect(double position) {
        targetPosition = position;
        if (targetPositionMin != null) position = Math.max(position, targetPositionMin);
        if (targetPositionMax != null) position = Math.min(position, targetPositionMax);
        io.setPosition(position);
    }

    public boolean isInPosition() {
        return Math.abs(targetPosition - inputs.position) <= IN_POSITION_TOLERANCE;
    }

    public boolean isAtReceivingPosition() {
        if (!isInPosition()) return false;
        
        return (targetPosition == receiving || targetPosition == algaeReceive);
    }

    public boolean isElevatorArmInScoringPosition() {
        if (!isInPosition()) return false;

        return targetPosition == L1_SCORE || targetPosition == L2_SCORE || targetPosition == L3_SCORE || 
               targetPosition == L4_SCORE || targetPosition == netScore || targetPosition == netScoreBackwards;
    }

    @NotLogged
    public double getTargetPosition() {
        return targetPosition;
    }

    public double getDegrees() {
        return Units.rotationsToDegrees(inputs.position);
    }

    public double getTargetDegrees() {
        return Units.rotationsToDegrees(targetPosition);
    }

    public double getErrorDegrees() {
        return getTargetDegrees() - getDegrees();
    }

    public double getAbsolutePosition() {
        return (inputs.absolutePosition * absoluteScalar) - absoluteOffset;
    }

    public double getAbsoluteDegrees() {
        return Units.rotationsToDegrees(getAbsolutePosition());
    }

    public void zeroAbsolute() {
        absoluteOffset = (inputs.absolutePosition * absoluteScalar);
    }

    @NotLogged
    public boolean isHomed() {
        return homed;
    }
}
