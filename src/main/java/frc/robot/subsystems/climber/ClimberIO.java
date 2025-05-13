package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ClimberIO {

    @Logged
    public class ClimberInputs {
        double jointPosition = 0;
        double grabberVelocity = 0;
        boolean sensorA = false;
        boolean sensorB = false;
    }

    public void update(ClimberInputs inputs);

    public void setSpeed(double speed);

    public void setGrabberSpeed(double speed);
    
}
