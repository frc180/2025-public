package frc.robot.subsystems.elevatorArmPivot;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorArmPivotIO {
    
    @Logged
    public class ElevatorArmPivotIOInputs {
        double position;
        double voltage;
        double target;
        double absolutePosition;
        double cancoderRotations;
    }
    
    public void setPosition(double encoderPosition);

    public default void simulationPeriodic() {}

    public void update(ElevatorArmPivotIOInputs inputs);

    public void setSpeed(double speed);

    public void stopMotor();

    public void zero(double offset);

    public void brakeMode();

    public void coastMode();
    
}
