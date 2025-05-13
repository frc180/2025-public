package frc.robot.subsystems.elevatorArm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevatorArm.ElevatorArmIO.ElevatorArmIOInputs;

@Logged
public class ElevatorArmSubsystem extends SubsystemBase{

    final ElevatorArmIO io;
    final ElevatorArmIOInputs inputs;

    @NotLogged
    public final Trigger hasCoral;
    @NotLogged
    public final Trigger hasPartialCoral;
    public final Trigger hasNoCoral;
    @NotLogged
    public final Trigger hasEnteringCoral;

    @NotLogged
    final double slowIndexSpeed = 0.04;

    public ElevatorArmSubsystem() {
        inputs = new ElevatorArmIOInputs();
        io = new ElevatorArmIOTalonFX();

        hasCoral = new Trigger(this::hasCoralBool);
        hasPartialCoral = new Trigger(this::hasPartialCoralBool);
        hasNoCoral = hasPartialCoral.negate();
        hasEnteringCoral = new Trigger(this::hasEnteringCoralBool);
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command intakeAndIndex() {
        return indexCommand(0.2);
    }

    public Command passiveIndex() {
        return indexCommand(0);
    }

    public Command reverseIntakeAndIndex() {
        return indexCommand(-0.3);
    }

    private Command indexCommand(double idleSpeed) {
        return runEnd(() -> {
            // Middle sensor only means we're centered, so stop
            if (!inputs.backCoralSensor && inputs.middleCoralSensor && !inputs.frontCoralSensor) {
                io.setSpeed(0);
                return;
            }

            // Front sensor means we've overshot, so reverse
            if (inputs.frontCoralSensor) {
                io.setSpeed(-slowIndexSpeed);
                return;
            }

            // Back and middle sensors means we're approaching the sweet spot, so move forward slowly
            if (inputs.backCoralSensor && inputs.middleCoralSensor) {
                io.setSpeed(slowIndexSpeed);
                return;
            }

            io.setSpeed(idleSpeed);
        },
        () -> io.setSpeed(0));
    }

    public Command runRollers() {
        return run(() -> io.run());
    }

    public Command reverseRun() {
        return run(() -> io.reverse());
    }

    public Command stop() {
        return run(() -> io.stop());
    }

    public Command setSpeed(double speed) {
        return run(() -> io.setSpeed(speed));
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.stop()
        );
    }

    public boolean hasCoralBool() {
        return inputs.middleCoralSensor && !inputs.backCoralSensor;
    }

    public boolean hasPartialCoralBool() {
        return inputs.frontCoralSensor || inputs.middleCoralSensor || inputs.backCoralSensor;
    }

    public boolean hasEnteringCoralBool() {
        return inputs.backCoralSensor && !inputs.middleCoralSensor && !inputs.frontCoralSensor;
    }

    public double getVelocity() {
        return inputs.velocity;
    }
}
