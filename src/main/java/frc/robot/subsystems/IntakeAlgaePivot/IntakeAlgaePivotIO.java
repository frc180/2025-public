package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeAlgaePivotIO {

    @Logged
    public class IntakeAlgaePivotIOInputs {
        double position;
        double voltage;
        double target;
        double absolutePosition;
    }

    public void update(IntakeAlgaePivotIOInputs inputs);
    
    public void setPosition(double encoderPosition);

    public default void simulationPeriodic() {}

    public void setSpeed(double speed);

    public void setWinchSpeed(double speed);

    public void stopMotor();

    public void zero(double offset);

    public void brakeMode();

    public void coastMode();
}
