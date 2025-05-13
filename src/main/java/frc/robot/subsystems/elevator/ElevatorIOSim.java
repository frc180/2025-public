package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOSim implements ElevatorIO {

    final PIDController pid;
    boolean usingPID = false;
    double position = 0;
    double speed = 0;
    double target = 0;

    public ElevatorIOSim() {
        pid = new PIDController(0.01, 0, 0);
    }

    @Override
    public void setPower(double power) {
        speed = power;
        usingPID = false;
    }

    @Override
    public void setVoltage(double volts) {
        setPower(volts / 12);
    }

    @Override
    public void setPosition(double encoderPosition) {
        target = encoderPosition;
        usingPID = true;
    }

    @Override
    public void update(ElevatorIOInputs inputs) {
        // Simulate movement with previously set speed
        position += speed * 8;

        if (usingPID) {
            speed = pid.calculate(position, target);
            speed = Math.max(-1, Math.min(1, speed));
        }

        inputs.position = position;
        inputs.dutyCycle = speed;
        inputs.target = target;
    }

    @Override
    public void stopMotor() {
        speed = 0;
    }

    @Override
    public void zero() {}

    @Override
    public void brakeMode() {}
}
