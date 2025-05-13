package frc.robot.subsystems.elevatorArmAlgae;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevatorArmAlgae.ElevatorArmAlgaeIO.ElevatorArmAlgaeInputs;

@Logged
public class ElevatorArmAlgaeSubsystem extends SubsystemBase {

    protected static final double HAS_ALGAE_THRESHOLD = 0.3;
    protected static final double HAS_ALGAE_CLOSE_THRESHOLD = 0.1;

    private final ElevatorArmAlgaeIO io;
    private final ElevatorArmAlgaeInputs inputs;

    private final LinearFilter distanceFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private double distanceFiltered = 0;

    public final Trigger hasAlgae, hadAlgae;


    public ElevatorArmAlgaeSubsystem() {
        inputs = new ElevatorArmAlgaeInputs();
        io = new ElevatorArmAlgaeIOTalonFX();

        hasAlgae = new Trigger(() -> distanceFiltered < HAS_ALGAE_THRESHOLD);
        hadAlgae = hasAlgae.debounce(1.5, DebounceType.kFalling);

        SmartDashboard.putNumber("Algae Amps", 20);
    }

    @Override
    public void periodic() {
        io.update(inputs);
        distanceFiltered = distanceFilter.calculate(inputs.distance);
    }
    
    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command intakeAndIndex(double speed) {
        return indexWithIdle(speed, speed);
    }

    public Command passiveIndex() {
        return indexWithIdle(0, 1);
    }

    private Command indexWithIdle(double idleSpeed, double indexSpeed) {
        return runEnd(
            () -> {
                double holdSpeed = 0.6;

                if (distanceFiltered < HAS_ALGAE_CLOSE_THRESHOLD) {
                    io.setSpeed(holdSpeed);
                } else if (hasAlgae.getAsBoolean()) {
                    io.setSpeed(holdSpeed);
                } else if (hadAlgae.getAsBoolean()) {
                    io.setSpeed(indexSpeed);
                } else {
                    io.setSpeed(idleSpeed);
                }
            },
            () -> io.setSpeed(0.0)
        );
    }

    public Command spit() {
        return runSpeed(-1);
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

    public Command stop() {
        return run(() -> io.stop());
    }
    
    public double getSpeed() {
        return inputs.speed;
    }
}
