package frc.robot.subsystems.IntakeCoral;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeCoral.IntakeCoralIO.IntakeIOInputs;

@Logged
public class IntakeCoralSubsystem extends SubsystemBase {

    private final IntakeCoralIO io;
    private final IntakeIOInputs inputs;

    private final Alert coralSensorDisconnectedAlert;
    private final Alert coralSensorStatusAlert;

    public final Trigger hasCoral;

    public IntakeCoralSubsystem() {
        inputs = new IntakeIOInputs();
        if (Robot.isReal()) {
            io = new IntakeCoralIOSpark();
        } else {
            io = new IntakeCoralIOSim();
        }

        hasCoral = new Trigger(this::hasCoralBool);

        coralSensorDisconnectedAlert = new Alert("Coral intake sensor disconnected!", AlertType.kError);
        coralSensorStatusAlert = new Alert("Coral intake sensor status: %d", AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.update(inputs);

        coralSensorDisconnectedAlert.set(!inputs.coralSensorConnected);
        coralSensorStatusAlert.setText("Coral intake sensor status: " + inputs.coralSensorStatus);
        coralSensorStatusAlert.set(inputs.coralSensorStatus != 0);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command intake() {
        return run(() -> io.setSpeed(1));
    }

    public Command stopIntake() {
        return run(() -> io.setSpeed(0));
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.setSpeed(0)
        );
    }

    public Command runBottomRollerSpeed(double speed) {
        return runEnd(
            () -> io.setBottomRollerSpeed(speed),
            () -> io.setBottomRollerSpeed(0)
        );
    }

    public boolean hasCoralBool() {
        return inputs.coralSensorConnected && inputs.coralDistance < 0.23; // .2 measured to be the 
    }
}
