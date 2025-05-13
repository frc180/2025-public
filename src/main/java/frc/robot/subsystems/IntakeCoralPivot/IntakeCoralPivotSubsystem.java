package frc.robot.subsystems.IntakeCoralPivot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotIO.IntakeCoralPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class IntakeCoralPivotSubsystem extends SubsystemBase {

    //presets for intake positions
    public static final double extend = .779;
    public static final double stow = extend - .417;
    public static final double EXTREME_STOW = stow - .1;

    private static final double IN_POSITION_TOLERANCE = Units.degreesToRotations(3);

    private final IntakeCoralPivotIO io;
    private final IntakeCoralPivotIOInputs inputs;

    private final ProfiledPIDController profiledPID;

    private double targetPosition = -1;
    private boolean firstPeriodic = true;

    @NotLogged
    public final Trigger atTarget = new Trigger(this::isAtTarget);
    @NotLogged
    public final Trigger atStowPosition = new Trigger(this::isAtStowPosition);

    public IntakeCoralPivotSubsystem() {
        inputs = new IntakeCoralPivotIOInputs();
        io = new IntakeCoralPivotIOTalonFXS();

        double kP = 10;
        if (Robot.isSimulation()) kP = 5;
        profiledPID = new ProfiledPIDController(kP, 0, 0, new Constraints(99, 99));
        SmartDashboard.putData("CoralPivotPID", profiledPID);
    }

    @Override
    public void periodic() {
        io.update(inputs);
        SimVisuals.setCoralIntakeDegrees(getDegrees() * 0.6);

        if (firstPeriodic) {
            profiledPID.reset(inputs.absolutePosition);
            targetPosition = inputs.absolutePosition;
            firstPeriodic = false;
        }

        double output = profiledPID.calculate(inputs.absolutePosition, targetPosition);
        io.setSpeed(output);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public boolean isAtTarget() {
        return Math.abs(inputs.position - targetPosition) <= IN_POSITION_TOLERANCE;
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

    public Command stow() {
        return setPosition(stow);
    }

    public Command extremeStow() {
        return setPosition(EXTREME_STOW);
    }

    public Command extend() {
        return setPosition(extend);
    }

    public Command setPosition(double encoderPosition) {
        return run(() -> {
            targetPosition = encoderPosition;
          });
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> setSpeed(speed),
            () -> setSpeed(0)
        );
    }

    public Command stop() {
        return run(() -> {
            io.stopMotor();
        });
    }

    public Command zero() {
        return zero(0);
    }

    public Command zero(Angle angle) {
        return zero(angle.in(Rotations));
    }

    public Command zero(double rotations) {
        return Commands.runOnce(() -> io.zero(rotations));
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public boolean isAtStowPosition() {
        return targetPosition == stow && isAtTarget();
    }

    public boolean isStalling() {
        return Math.abs(inputs.velocity) < 0.001;
    }
}
