package frc.robot.subsystems.IntakeAlgae;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeAlgaeIO {

    @Logged
    public class IntakeAlgaeIOInputs {
        public boolean hasAlgae;
    }

    public void startRollers();

    public void stop();

    public void spit();

    public void setSpeed(double speed);

    public void update(IntakeAlgaeIOInputs inputs);
}
