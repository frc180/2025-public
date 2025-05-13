package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.climber.ClimberIO.ClimberInputs;
import frc.robot.commands.RumbleCommand;

@Logged
public class Climber extends SubsystemBase {

    public enum ClimberState {
        IDLE,
        DEPLOYED,
        LATCHED,
        CLIMBING
    }

    private static final double HARD_STOP = 0.584;

    private static final double DEPLOYED = 0.6;
    private static final double MAX_CLIMB = 0.815 + Units.degreesToRotations(15);
    
    private final ClimberIO io;
    private final ClimberInputs inputs;

    public final Trigger grabberStalling;
    public final Trigger hasCage;

    private ClimberState state = ClimberState.IDLE;
    private double targetGrabberSpeed = 0;

    private boolean sensorATripped = false;
    private boolean sensorBTripped = false;

    public Climber() {
        inputs = new ClimberInputs();
        if (Robot.isReal()) {
            io = new ClimberIOTalonFX();
        } else {
            io = new ClimberIOSim();
        }

        grabberStalling = new Trigger(this::isGrabberStalling).debounce(0.75, DebounceType.kRising);
        hasCage = new Trigger(this::isCageLatched);
    }

    @Override
    public void periodic() {
        io.update(inputs);

        if (hasCage.getAsBoolean()) {
            setGrabberSpeed(0);
        } else {
            if (state == ClimberState.DEPLOYED) {
                setGrabberSpeed(0.5);
            } else if (state == ClimberState.CLIMBING || state == ClimberState.LATCHED) {
                setGrabberSpeed(0);
            }
        }

        if (state == ClimberState.DEPLOYED) {
            if (!sensorATripped) {
                sensorATripped = inputs.sensorA;
            }
            if (!sensorBTripped) {
                sensorBTripped = inputs.sensorB;
            }
        }
    }

    public Command deploy() {
        return runSpeed(0.1)
                .until(this::isDeployed)
                .andThen(setState(ClimberState.DEPLOYED));
    }

    final Command climbDoneRumble = new RumbleCommand(1).withTimeout(2);

    public Command climb() {
        return runEnd(
            () -> {
                io.setSpeed(0.3);
            },
            () -> io.setSpeed(0)
        ).until(this::shouldStopClimbing)
        .andThen(new ScheduleCommand(climbDoneRumble))
        .alongWith(setState(ClimberState.CLIMBING));
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.setSpeed(0)
        );
    }

    public Command runGrabberSpeed(double speed) {
        return runEnd(
            () -> setGrabberSpeed(speed),
            () -> setGrabberSpeed(0)
        );
    }

    public Command setState(ClimberState newState) {
        return Commands.runOnce(() -> state = newState);
    }

    public Trigger isState(ClimberState state) {
        return new Trigger(() -> this.state == state);
    }

    public boolean isDeployed() {
        return inputs.jointPosition < DEPLOYED;
    }

    public boolean shouldStopClimbing() {
        return inputs.jointPosition > MAX_CLIMB;
    }

    private void setGrabberSpeed(double speed) {
        targetGrabberSpeed = speed;
        io.setGrabberSpeed(speed);
    }

    public boolean isGrabberStalling() {
        return Math.abs(inputs.grabberVelocity) < 10 && targetGrabberSpeed != 0;
    }

    public boolean isCageLatched() {
        return (!inputs.sensorA && sensorATripped) && (!inputs.sensorB && sensorBTripped);
    }
}
