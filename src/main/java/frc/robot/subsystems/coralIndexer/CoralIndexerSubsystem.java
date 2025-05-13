package frc.robot.subsystems.coralIndexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralIndexer.CoralIndexerIO.CoralIndexerInputs;

public class CoralIndexerSubsystem extends SubsystemBase {

    final CoralIndexerIO io;
    final CoralIndexerInputs inputs;

    public CoralIndexerSubsystem() {
        io = new CoralIndexerIOTalonFXS();
        inputs = new CoralIndexerInputs();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.setSpeed(0)
        );
    }
}
