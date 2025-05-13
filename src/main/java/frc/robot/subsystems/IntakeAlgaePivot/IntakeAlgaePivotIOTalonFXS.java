package frc.robot.subsystems.IntakeAlgaePivot;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeAlgaePivotIOTalonFXS implements IntakeAlgaePivotIO {
    
    final double intakeArmGearing = 200;
    final double radToRotations = 1 / (2 * Math.PI);
    final double absoluteEncoderResolution = 2048;

    final double absoluteOffset = 0;

    TalonFX pivotMotor, winchMotor;
    MotionMagicVoltage motionMagicControl;
    VoltageOut voltageControl, winchVoltageControl;
    Follower followerControl;
    DutyCycleEncoder absoluteEncoder;

    // Status signals
    StatusSignal<Angle> positionSignal;
    StatusSignal<Voltage> voltageSignal;
    StatusSignal<Double> targetSignal;

    // Simulation-only variables
    TalonFXSimState pivotMotorASim;
    SingleJointedArmSim intakeSim;

    public IntakeAlgaePivotIOTalonFXS() {
        pivotMotor = new TalonFX(Constants.INTAKE_ALGAE_PIVOT_TALON, Constants.CANIVORE);
        winchMotor = new TalonFX(Constants.CLIMBER_WINCH_TALON, Constants.CANIVORE);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = intakeArmGearing;
        if (Robot.isReal()) {
            configuration.Slot0.kP = 0;
            configuration.Slot0.kI = 0;
            configuration.Slot0.kD = 0;
            configuration.Slot0.kG = 0;
            configuration.Slot0.kV = 0;
            configuration.Slot0.kA = 0;
        } else  {
            configuration.Slot0.kP = 36;
            configuration.Slot0.kI = 0;
            configuration.Slot0.kD = 0;
            configuration.Slot0.kG = 0;
            configuration.Slot0.kV = 0;
            configuration.Slot0.kA = 0;
        }
        configuration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configuration.MotionMagic.MotionMagicCruiseVelocity = 999;
        configuration.MotionMagic.MotionMagicAcceleration = 999;
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.MotionMagic.MotionMagicJerk = 0;

        pivotMotor.getConfigurator().apply(configuration);
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);

        configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        winchMotor.getConfigurator().apply(configuration);
        winchMotor.setNeutralMode(NeutralModeValue.Brake);

        motionMagicControl = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);
        winchVoltageControl = new VoltageOut(0);
        followerControl = new Follower(Constants.INTAKE_ALGAE_PIVOT_TALON, true);

        absoluteEncoder = new DutyCycleEncoder(Constants.DIO_INTAKE_ALGAE_ENCODER);

        positionSignal = trackSignal(pivotMotor.getPosition());
        voltageSignal = trackSignal(pivotMotor.getMotorVoltage());
        targetSignal = trackSignal(pivotMotor.getClosedLoopReference());

        //for simulation
        if (Robot.isReal()) return;

        pivotMotorASim = pivotMotor.getSimState();

        intakeSim = new SingleJointedArmSim(
            DCMotor.getNeo550(2),
            intakeArmGearing,
            1,
            .37,
            0,
            2.0944,
            false,
            Units.rotationsToRadians(IntakeAlgaePivotSubsystem.stow)
        );
    }


    @Override
    public void update(IntakeAlgaePivotIOInputs inputs){ 
        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.target = targetSignal.getValueAsDouble();
        inputs.absolutePosition = absoluteEncoder.get() - absoluteOffset;
    }

    // Simulation only
    @Override
    public void simulationPeriodic() {
        intakeSim.setInput(pivotMotorASim.getMotorVoltage());
        intakeSim.update(Constants.LOOP_TIME);

        pivotMotorASim.setRawRotorPosition(intakeSim.getAngleRads() * radToRotations * intakeArmGearing); 
        pivotMotorASim.setRotorVelocity(intakeSim.getVelocityRadPerSec() * radToRotations * intakeArmGearing);
        pivotMotorASim.setSupplyVoltage(12);
    }

    @Override
    public void setPosition(double encoderPosition) {
        pivotMotor.setControl(motionMagicControl.withPosition(encoderPosition));
    }

    @Override
    public void setSpeed(double speed) {
        pivotMotor.setControl(voltageControl.withOutput(speed * 12));
    }

    @Override
    public void setWinchSpeed(double speed) {
        winchMotor.setControl(winchVoltageControl.withOutput(speed  * 12));
    }

    @Override
    public void stopMotor() {
        pivotMotor.stopMotor();
    }

    @Override
    public void zero(double offset) {
        pivotMotor.setPosition(offset, 0);
    }


    @Override
    public void brakeMode() {
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        winchMotor.setNeutralMode(NeutralModeValue.Brake);
    }


    @Override
    public void coastMode() {
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);
        winchMotor.setNeutralMode(NeutralModeValue.Coast);
    }
}
