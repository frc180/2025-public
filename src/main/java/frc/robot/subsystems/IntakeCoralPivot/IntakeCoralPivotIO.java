package frc.robot.subsystems.IntakeCoralPivot;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeCoralPivotIO {

    @Logged
    public class IntakeCoralPivotIOInputs {
        double position;
        double velocity;
        double voltage;
        double target;
        double absolutePosition;
    }
    
    public void setIntakePosition(double position);

    public default void simulationPeriodic() {}

    public void update(IntakeCoralPivotIOInputs inputs);

    public void setSpeed(double speed);

    public void stopMotor();

    public void zero(double rotations);
}
