package frc.robot.subsystems.IntakeAlgae;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeAlgaeIOTalonFXS implements IntakeAlgaeIO {
    
    TalonFXS intakeMotor;
    VoltageOut voltageControl;
    DigitalInput sensor;

    public IntakeAlgaeIOTalonFXS() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        intakeMotor = new TalonFXS(Constants.INTAKE_ALGAE_TALON, Constants.CANIVORE);
        intakeMotor.getConfigurator().apply(config);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        voltageControl = new VoltageOut(0);
    }

    @Override
    public void update(IntakeAlgaeIOInputs inputs) {
        inputs.hasAlgae = false;
    }
    
    @Override
    public void startRollers() {
        setSpeed(1);
    }

    @Override
    public void stop() {
        setSpeed(0);
    }

    @Override
    public void spit() {
        setSpeed(-1);
    }

    /**
     * Sets the speed (-1 to 1) of the intake
     */
    @Override
    public void setSpeed(double speed) {
        setVoltage(speed * 12);
    }

    private void setVoltage(double voltage) {
        intakeMotor.setControl(voltageControl.withOutput(voltage));
    }
}
