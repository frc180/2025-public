package frc.robot.subsystems.elevatorArmAlgae;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorArmAlgaeIO {

    @Logged
    public class ElevatorArmAlgaeInputs {
        public double speed;
        public double distance;
        public double distanceSignalStrength;
    }

    public void setSpeed(double speed);

    public void setAmps(double amps);
    
    public void stop();

    public void update(ElevatorArmAlgaeInputs inputs);

    public default void simulationPeriodic() {}
}
