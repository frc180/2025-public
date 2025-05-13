package frc.robot.subsystems.elevatorArmPivot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorArmPivotIOTalonFX implements ElevatorArmPivotIO {
    final double PIVOT_GEARING = 128;
    final double radToRotations = 1 / (2 * Math.PI);

    final Angle POTENTIOMETER_OFFSET = Degrees.of(0);
    final Angle SIM_HARD_STOP_POSITION = Degrees.of(60.46875 + 3.955078125 + 2.109375 - 2.63671875);

     // Converts absolute encoder units to mechanism rotations
    final double ABSOLUTE_ENCODER_RATIO = (1.0 / 3.0) * (1-0.0482);
    final double ABSOLUTE_ENCODER_OFFSET = Degrees.of(0).in(Rotations);

    final TalonFX armPivotMotor;
    final AnalogPotentiometer potentiometer;
    final MotionMagicVoltage motionMagic;
    final VoltageOut voltageControl;

    NeutralModeValue neutralMode = null;
    CANcoder cancoder;
    DutyCycleEncoder absoluteEncoder;

    // Status signals
    StatusSignal<Angle> positionSignal;
    StatusSignal<Voltage> voltageSignal;
    StatusSignal<Double> targetSignal;
    StatusSignal<Angle> cancoderPositionSignal;

    final Alert cancoderAlert = new Alert("ArmPivot CANcoder disconnected!", AlertType.kError);

    // Simulation-only variables
    TalonFXSimState armPivotMotorSim;
    SingleJointedArmSim armSim;

    public ElevatorArmPivotIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        if (Robot.isReal()) {
            config.Slot0.kP = 2000;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0.23;
            config.Slot0.kV = 15;
            config.Slot0.kA = 0;
        } else {
            config.Slot0.kP = 1000;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0;
            config.Slot0.kV = 17;
            config.Slot0.kA = 0;
        }
        config.MotionMagic.MotionMagicCruiseVelocity = 2.6;
        config.MotionMagic.MotionMagicAcceleration = 5;
        config.MotionMagic.MotionMagicJerk = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Internal encoder only
        config.Feedback.SensorToMechanismRatio = PIVOT_GEARING;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorArmPivotSubsystem.FORWARD_LIMIT.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorArmPivotSubsystem.REVERSE_LIMIT.in(Rotations);

        armPivotMotor = new TalonFX(Constants.ELEVATOR_ARM_PIVOT_TALON, Constants.CANIVORE);
        armPivotMotor.getConfigurator().apply(config);
        setNeutralMode(NeutralModeValue.Coast);

        motionMagic = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);

        potentiometer = new AnalogPotentiometer(
            Constants.ANALOG_ELEVATOR_ARM_PIVOT_POTENTIOMETER,
            1,
            POTENTIOMETER_OFFSET.in(Rotations)
        );

        positionSignal = trackSignal(armPivotMotor.getPosition());
        voltageSignal = trackSignal(armPivotMotor.getMotorVoltage());
        targetSignal = trackSignal(armPivotMotor.getClosedLoopReference());

        // Everything below this line is for simulation only
        if (Robot.isReal()) return;

        armPivotMotor.setPosition(SIM_HARD_STOP_POSITION);
        armPivotMotorSim = armPivotMotor.getSimState();

        double hardStopRadians = SIM_HARD_STOP_POSITION.in(Radians);
        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            PIVOT_GEARING,
            1,
            0.4572,
            -Math.PI,
            hardStopRadians,
            true,
            hardStopRadians
        );
    }

    @Override
    public void update(ElevatorArmPivotIOInputs inputs) {
        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.target = targetSignal.getValueAsDouble();

        if (Robot.isReal()) {
            inputs.absolutePosition = -potentiometer.get();
        } else {
            inputs.absolutePosition = 30 + (Units.radiansToRotations(armSim.getAngleRads()) * 0.25);
        }
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(armPivotMotorSim.getMotorVoltage());
        armSim.update(Constants.LOOP_TIME);

        armPivotMotorSim.setRawRotorPosition(armSim.getAngleRads() * radToRotations * PIVOT_GEARING);
        armPivotMotorSim.setRotorVelocity(armSim.getVelocityRadPerSec() * radToRotations * PIVOT_GEARING);
        armPivotMotorSim.setSupplyVoltage(12);
    }

    @Override
    public void setSpeed(double speed) {
        armPivotMotor.setControl(voltageControl.withOutput(speed * 12));
    }

    @Override
    public void stopMotor() {
        armPivotMotor.stopMotor();
    }

    @Override
    public void setPosition(double encoderPosition) {
        armPivotMotor.setControl(motionMagic.withPosition(encoderPosition));
    }

    @Override
    public void zero(double rotations) {
        armPivotMotor.setPosition(rotations);
    }

    @Override
    public void brakeMode() {
        setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void coastMode() {
        setNeutralMode(NeutralModeValue.Coast);
    }

    private void setNeutralMode(NeutralModeValue value) {
        if (value == neutralMode) return;

        StatusCode code = armPivotMotor.setNeutralMode(value);
        if (code == StatusCode.OK) {
            neutralMode = value;
        } else {
            System.out.println("FAILED to set arm pivot neutral mode: " + code);
        }
    }
}
