package frc.robot.subsystems.IntakeCoralPivot;

import edu.wpi.first.math.controller.PIDController;

public class IntakeCoralPivotIOSim implements IntakeCoralPivotIO{

    double position = IntakeCoralPivotSubsystem.stow;
    double speed = 0;
    double target = 0;

    PIDController pid;
    boolean usingPID = false;

    public IntakeCoralPivotIOSim() {
        pid = new PIDController(0.01, 0, 0);
    }

    @Override
    public void setIntakePosition(double encoderPosition){
        target = encoderPosition;
        usingPID = true;
    }

    @Override
    public void update(IntakeCoralPivotIOInputs inputs) {
        if (usingPID) {
            speed = pid.calculate(position, target);
            speed = Math.max(-1, Math.min(1, speed));
        }

        inputs.position = position;
        inputs.voltage = speed * 12;
        inputs.target = target;

    }

    @Override
    public void simulationPeriodic() {
        position += speed * 8;
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
        usingPID = false;
    }

    @Override
    public void stopMotor() {
        speed = 0;
    }

    @Override
    public void zero(double rotations) {
        // Do nothing
    }
    
}
