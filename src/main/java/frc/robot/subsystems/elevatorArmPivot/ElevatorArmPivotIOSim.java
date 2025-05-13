package frc.robot.subsystems.elevatorArmPivot;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorArmPivotIOSim implements ElevatorArmPivotIO{

    double position;
    double speed;
    double target;

    PIDController pid;
    boolean usingPID = false;

    ElevatorArmPivotIOInputs input = new ElevatorArmPivotIOInputs();

    public ElevatorArmPivotIOSim() {
        pid = new PIDController(0.05, 0, 0);
    }

    @Override
    public void setPosition(double degrees) {
        target = degrees;
        usingPID = true;

    }

    @Override
    public void update(ElevatorArmPivotIOInputs inputs) {
        if (usingPID) {
            speed = pid.calculate(position, target);
            speed = Math.max(-1, Math.min(1, speed));

        }

        inputs.position = position;
        inputs.target = target;
        inputs.voltage = speed * 12;
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
    public void zero(double offset) {
        // do nothing
    }

    @Override
    public void brakeMode() {}

    @Override
    public void coastMode() {}
}
