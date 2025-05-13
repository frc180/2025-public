package frc.robot.subsystems.elevatorArm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.spamrobotics.util.Helpers;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class ElevatorArmIOTalonFX implements ElevatorArmIO {

    static final double VELOCITY_RPS_12V = Robot.isReal() ? 48 : 30.1875;
    static final double GEAR_RATIO = Robot.isReal() ? 2.25 : 4; 
    
    final TalonFX rollerMotor;
    final CANdi candiA, candiB;
    final VoltageOut voltageControl;
    final VelocityVoltage velocityControl;

    final StatusSignal<Boolean> frontSensorSignal, middleSensorSignal, backSensorSignal;
    final StatusSignal<AngularVelocity> velocitySignal;

    final Alert candiADisconnectedAlert, candiBDisconnectedAlert;

    double targetVelocity = 0;

    // Simulation-only variables
    TalonFXSimState motorSimState;
    CANdiSimState candiASim, candiBSim;
    DCMotorSim motorSim;

    public ElevatorArmIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        if (Robot.isReal()) {
            config.Slot0.kP = 0;
            config.Slot0.kV = 0;
        } else {
            config.Slot0.kP = 0.15;
            config.Slot0.kV = 0.3975;
        }
        rollerMotor = new TalonFX(Constants.ELEVATOR_ARM_TALON, Constants.CANIVORE);
        rollerMotor.getConfigurator().apply(config);

        voltageControl = new VoltageOut(0);
        velocityControl = new VelocityVoltage(0);

        CANdiConfiguration candiConfig = new CANdiConfiguration();
        candiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenNotHigh;
        candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
        candiConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenNotHigh;
        candiConfig.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
        candiA = new CANdi(Constants.ELEVATOR_ARM_CANDI_A, Constants.CANIVORE);
        candiB = new CANdi(Constants.ELEVATOR_ARM_CANDI_B, Constants.CANIVORE);
        candiA.getConfigurator().apply(candiConfig);
        candiB.getConfigurator().apply(candiConfig);

        frontSensorSignal = trackSignal(candiA.getS2Closed());
        middleSensorSignal = trackSignal(candiA.getS1Closed());
        backSensorSignal = trackSignal(candiB.getS1Closed());
        velocitySignal = trackSignal(rollerMotor.getVelocity());

        candiADisconnectedAlert = new Alert("ElevatorArm CANDi A disconnected (#" + Constants.ELEVATOR_ARM_CANDI_A + ")", AlertType.kError);
        candiBDisconnectedAlert = new Alert("ElevatorArm CANDi B disconnected (#" + Constants.ELEVATOR_ARM_CANDI_B + ")", AlertType.kError);

        if (Robot.isReal()) return;

        motorSimState = rollerMotor.getSimState();
        candiASim = candiA.getSimState();
        candiBSim = candiB.getSimState();

        DCMotor physicalMotor = Helpers.getMinion(1);
        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                physicalMotor, 0.001, GEAR_RATIO
            ),
            physicalMotor
        );
    }

    @Override
    public void update(ElevatorArmIOInputs inputs) {
        boolean candiADisconnected = frontSensorSignal.getTimestamp().getLatency() > 0.5;
        boolean candiBDisconnected = backSensorSignal.getTimestamp().getLatency() > 0.5;

        candiADisconnectedAlert.set(candiADisconnected);
        candiBDisconnectedAlert.set(candiBDisconnected);

        inputs.frontCoralSensor = frontSensorSignal.getValueAsDouble() == 1;
        inputs.middleCoralSensor = middleSensorSignal.getValueAsDouble() == 1;
        inputs.backCoralSensor = backSensorSignal.getValueAsDouble() == 1;
        inputs.velocity = velocitySignal.getValueAsDouble();
        inputs.targetVelocity = targetVelocity;
    }

    @Override
    public void run() {
        setSpeed(1);
    }

    @Override
    public void reverse() {
        setSpeed(-1);
    }

    @Override
    public void stop() {
        setSpeed(0);
    }

    @Override
    public void setSpeed(double speed) {
        targetVelocity = speed * VELOCITY_RPS_12V;
        setVoltage(speed * 12);
    }

    @Override
    public void setVelocity(double velocityPercent) {
        setVelocityDirect(velocityPercent * VELOCITY_RPS_12V);
    }

    private void setVoltage(double volts) {
        rollerMotor.setControl(voltageControl.withOutput(volts));
    }

    private void setVelocityDirect(double velocity) {
        targetVelocity = velocity;
        rollerMotor.setControl(velocityControl.withVelocity(velocity));
    }

    // ======================= Simulation Logic =======================

    @Override
    public void simulationPeriodic() {
        double motorVoltage = motorSimState.getMotorVoltage();
        motorSim.setInputVoltage(motorVoltage);
        motorSim.update(Constants.LOOP_TIME);

        // Apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        AngularVelocity motorVelocity = motorSim.getAngularVelocity();
        motorSimState.setRawRotorPosition(motorSim.getAngularPosition().times(GEAR_RATIO));
        motorSimState.setRotorVelocity(motorVelocity.times(GEAR_RATIO));
        double rollerSpeed = (motorVelocity.in(Rotations.per(Second)) / VELOCITY_RPS_12V);

        ElevatorArmPivotSubsystem armPivot = RobotContainer.instance.elevatorArmPivot;
        boolean readyForCoral = SimLogic.intakeHasCoral && armPivot.isAtReceivingPosition();

        boolean armHasCoralPrevious = SimLogic.armHasCoral;

        if (readyForCoral && rollerSpeed > 0) {
            SimLogic.armHasCoral = true;
            SimLogic.intakeHasCoral = false;
        }

        if (SimLogic.armHasCoral) {
            if (!armHasCoralPrevious) {
                // Reset simulated coral position to start if we just got coral
                SimLogic.armCoralPosition = 0;
            } else if (SimLogic.armCoralPosition == -1) {
                // Preloaded corral
                SimLogic.armCoralPosition = SimLogic.CORAL_LENGTH;
            } else {
                SimLogic.armCoralPosition += rollerSpeed * 0.4;
            }
        }

        boolean backCoral = simCoralAtSensor(0);
        boolean middleCoral = simCoralAtSensor(SimLogic.CORAL_LENGTH * .7);
        boolean frontCoral = simCoralAtSensor((SimLogic.CORAL_LENGTH * .7) * 2);

        // Coral has passed through the entire arm, score it
        if (SimLogic.armHasCoral && !backCoral && !middleCoral && !frontCoral) {
            SimLogic.armHasCoral = false;
            SimLogic.scoreCoral();
        }

        candiASim.setS2State(frontCoral ? S2StateValue.Low : S2StateValue.High);
        candiASim.setS1State(middleCoral ? S1StateValue.Low : S1StateValue.High);
        candiBSim.setS1State(backCoral ? S1StateValue.Low : S1StateValue.High);
    }

    private boolean simCoralAtSensor(double sensorPosition) {
        if (!SimLogic.armHasCoral) {
            return false;
        }

        double coralFront = SimLogic.armCoralPosition;
        double coralBack = SimLogic.armCoralPosition - SimLogic.CORAL_LENGTH;
        return sensorPosition >= coralBack && sensorPosition <= coralFront;
    }
}
