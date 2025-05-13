package frc.robot.subsystems.climber;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {

    final TalonFX winchMotor, grabberMotor;
    final DutyCycleEncoder absoluteEncoder;
    final DigitalInput sensorA, sensorB;
    final VoltageOut voltageControl, grabberVoltage;

    final StatusSignal<AngularVelocity> grabberVelocity;

    public ClimberIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        winchMotor = new TalonFX(Constants.CLIMBER_WINCH_TALON, Constants.CANIVORE);
        winchMotor.getConfigurator().apply(config);

        config = new TalonFXConfiguration();
        grabberMotor = new TalonFX(Constants.CLIMBER_GRABBER_TALON, Constants.CANIVORE);
        grabberMotor.getConfigurator().apply(config);

        absoluteEncoder = new DutyCycleEncoder(Constants.DIO_INTAKE_ALGAE_ENCODER);

        sensorA = new DigitalInput(Constants.DIO_CLIMBER_SENSOR_A);
        sensorB = new DigitalInput(Constants.DIO_CLIMBER_SENSOR_B);

        voltageControl = new VoltageOut(0);
        grabberVoltage = new VoltageOut(0);

        grabberVelocity = trackSignal(grabberMotor.getVelocity());
    }

    @Override
    public void update(ClimberInputs inputs) {
        inputs.jointPosition = absoluteEncoder.get();
        inputs.sensorA = !sensorA.get();
        inputs.sensorB = !sensorB.get();
        inputs.grabberVelocity = grabberVelocity.getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        winchMotor.setControl(voltageControl.withOutput(speed * 12));
    }

    @Override
    public void setGrabberSpeed(double speed) {
        grabberMotor.setControl(grabberVoltage.withOutput(speed * 12));
    }
}
