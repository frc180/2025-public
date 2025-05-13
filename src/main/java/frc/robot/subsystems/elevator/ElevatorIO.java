package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorIO {

    @Logged
    public class ElevatorIOInputs {
        double position;
        double velocity;
        double dutyCycle;
        double target;
        boolean bottomLimit = false;
    }

    public void update(ElevatorIOInputs inputs);
    
    public void setPower(double power);

    public void setVoltage(double volts);

    public void setPosition(double encoderPosition);

    public void stopMotor();

    public void zero();

    public void brakeMode();

    public default void simulationPeriodic() {}

}
