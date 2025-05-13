package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotIO.IntakeAlgaePivotIOInputs;

@Logged
public class IntakeAlgaePivotSubsystem extends SubsystemBase {
    
    public static final double extend = Units.degreesToRotations(55);
    public static final double stow = .499;
    public static final double up = .463;
    public static final double climbReady = .1502;

    private static final double IN_POSITION_TOLERANCE = Units.degreesToRotations(3);

    private final IntakeAlgaePivotIO io;
    private final IntakeAlgaePivotIOInputs inputs;
    private final ProfiledPIDController profiledPID;
    
    private boolean firstPeriodic = true;
    private double targetPosition = stow;

    public IntakeAlgaePivotSubsystem() {
        inputs = new IntakeAlgaePivotIOInputs();

        if (Robot.isReal()) {
            io = new IntakeAlgaePivotIOSim();
        } else {
            io = new IntakeAlgaePivotIOTalonFXS();
        }

        profiledPID = new ProfiledPIDController(0, 0, 0, new Constraints(99, 99));
        SmartDashboard.putData("AlgaePivotPID", profiledPID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.update(inputs);

        if (firstPeriodic) {
            io.zero(inputs.absolutePosition);
            firstPeriodic = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command extend() {
        return setPosition(extend);
    }

    public Command stow() {
        return setPosition(stow);
    }

    public Command readyClimb() {
        return setPosition(climbReady);
    }

    public Command setPosition(double position) {
        return this.run(() -> {
            io.setPosition(position);
            targetPosition = position;
        });
    }

    public Command setSpeed(double speed) {
        return run(() -> io.setSpeed(speed));
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.stopMotor()
        );
    }

    public Command runWinchSpeed(double speed) {
        return runEnd(
            () -> io.setWinchSpeed(speed),
            () -> io.setWinchSpeed(0)
        );
    }

    public Command climb() {
        double climbSpeed = 0.25;
        return Commands.sequence(
            run(() -> io.setWinchSpeed(climbSpeed)).withTimeout(0.1),
            runEnd(
                () -> {
                    io.setSpeed(-climbSpeed);
                    io.setWinchSpeed(climbSpeed);
                },
                () -> {
                    io.setSpeed(0);
                    io.setWinchSpeed(0);
                }
            ).until(this::shouldStopClimb)
        ).finallyDo(interrupted -> {
            io.setSpeed(0);
            io.setWinchSpeed(0);
        });
    }

    public Command brakeMode() {
        return Commands.runOnce(io::brakeMode).ignoringDisable(true);
    }

    public Command coastMode() {
        return Commands.runOnce(io::coastMode).ignoringDisable(true);
    }

    public boolean shouldStopClimb() {
        return inputs.absolutePosition > up;
    }

    public Command stop() {
        return run(() -> {
            io.stopMotor();
        });
    }

    @NotLogged
    public double getPosition() {
        return inputs.position;
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

    public boolean isAtTarget() {
        return Math.abs(inputs.position - targetPosition) <= IN_POSITION_TOLERANCE;
    }

    @NotLogged
    public Pose3d getPose() {
        double angle = Units.rotationsToRadians(inputs.position);
        return new Pose3d(0, 0.2, 0.2, new Rotation3d(0, angle, Units.degreesToRadians(270)));
    }
}
