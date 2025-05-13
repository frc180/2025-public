package frc.robot.subsystems.IntakeCoral;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeCoralIO {

    @Logged
    public class IntakeIOInputs {
        public double voltage = 0;
        public double coralDistance = 0;
        public boolean coralSensorConnected = false;
        public int coralSensorStatus = -1;
    }

    public void update(IntakeIOInputs inputs);

    public void setSpeed(double speed);

    public void setBottomRollerSpeed(double speed);

    public default void simulationPeriodic() {}

}
