package frc.robot.subsystems.IntakeCoralPivot;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeCoralPivotIOTalonFXS implements IntakeCoralPivotIO {
    final double PIVOT_GEARING = 7 * 5 * 3 * (22.0/16.0) * (36.0/22.0);

    final TalonFXS motor;
    final MotionMagicExpoVoltage motionMagicControl;
    final VoltageOut voltageControl;
    final DutyCycleEncoder absoluteEncoder;

    // Status signals
    final StatusSignal<Angle> positionSignal;
    final StatusSignal<Voltage> voltageSignal;
    final StatusSignal<Double> targetSignal;

    // Simulation-only variables
    TalonFXSSimState motorSim = null;
    double simulatedPosition = IntakeCoralPivotSubsystem.EXTREME_STOW;
    
    public IntakeCoralPivotIOTalonFXS() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
        if (Robot.isReal()) {
            config.ExternalFeedback.SensorToMechanismRatio = PIVOT_GEARING;
            config.Slot0.kP = 0;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0;
            config.Slot0.kV = 0;
            config.MotionMagic.MotionMagicExpo_kV = 2;
            config.MotionMagic.MotionMagicExpo_kA = 0;
        } else {
            config.Slot0.kP = 1440;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0;
            config.Slot0.kV = 0;
            config.MotionMagic.MotionMagicExpo_kV = .05;
            config.MotionMagic.MotionMagicExpo_kA = 0;
        }
        config.MotionMagic.MotionMagicJerk = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor = new TalonFXS(Constants.INTAKE_CORAL_PIVOT_TALON, Constants.CANIVORE);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);

        voltageControl = new VoltageOut(0);
        motionMagicControl = new MotionMagicExpoVoltage(0);

        positionSignal = trackSignal(motor.getPosition());
        voltageSignal = trackSignal(motor.getMotorVoltage());
        targetSignal = trackSignal(motor.getClosedLoopReference());

        absoluteEncoder = new DutyCycleEncoder(Constants.DIO_INTAKE_CORAL_ENCODER);

        if (Robot.isReal()) return;

        motorSim = motor.getSimState();
    }

    @Override
    public void update(IntakeCoralPivotIOInputs inputs) {
        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.target = targetSignal.getValueAsDouble();
        
        if (Robot.isReal()) {
            inputs.absolutePosition = absoluteEncoder.get();
        } else {
            inputs.absolutePosition = simulatedPosition;
        }
    }

    @Override
    public void simulationPeriodic() {
        simulatedPosition += motorSim.getMotorVoltage() * Units.degreesToRotations(1.5);
        motorSim.setRawRotorPosition(simulatedPosition);
    }

    @Override
    public void setIntakePosition(double position){
        motor.setControl(motionMagicControl.withPosition(position));
    }

    @Override
    public void setSpeed(double speed) {
        motor.setControl(voltageControl.withOutput(speed * 12));
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void zero(double rotations) {
        motor.setPosition(rotations);
    }
}
