package frc.robot.subsystems.IntakeCoral;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class IntakeCoralIOSpark implements IntakeCoralIO {

    final SparkMax motor;
    final TalonFXS bottomRoller;
    final LaserCan laserCan;
    final VoltageOut voltageControl;

    double previousDistance = 0;
    double lastDistanceChangeTime = 0;

    public IntakeCoralIOSpark() {
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.idleMode(IdleMode.kBrake)
                   .voltageCompensation(12);

        motor = new SparkMax(Constants.INTAKE_CORAL_SPARK, MotorType.kBrushless);
        motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        TalonFXSConfiguration talonConfig = new TalonFXSConfiguration();
        talonConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        bottomRoller = new TalonFXS(Constants.INTAKE_CORAL_ROLLER_TALON, Constants.CANIVORE);
        bottomRoller.getConfigurator().apply(talonConfig);

        voltageControl = new VoltageOut(0);

        laserCan = configureLaser(new LaserCan(Constants.INTAKE_LASERCAN));
    }

    @Override
    public void update(IntakeIOInputs inputs) {
        double distance = 2;

        if (laserCan != null) {
            LaserCan.Measurement measurement = laserCan.getMeasurement();
            if (measurement != null) {
                if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                    distance = measurement.distance_mm * 0.001;
                }
                inputs.coralSensorStatus = measurement.status;
            } else {
                inputs.coralSensorStatus = -1;
            }
        }

        double currentTime = Timer.getFPGATimestamp();
        if (distance - previousDistance != 0) {
            lastDistanceChangeTime = currentTime;
        }

        inputs.coralSensorConnected = laserCan != null && currentTime - lastDistanceChangeTime < 0.5;
        inputs.coralDistance = distance;
        inputs.voltage = motor.getAppliedOutput() * 12;

        previousDistance = distance;
    }

    @Override
    public void setSpeed(double speed) {
        motor.setVoltage(speed * 12);
        setBottomRollerSpeed(speed);
    }

    private LaserCan configureLaser(LaserCan laser) {
        try {
            laser.setRangingMode(LaserCan.RangingMode.SHORT);
            laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
            return null;
        }
        return laser;
    }

    @Override
    public void setBottomRollerSpeed(double speed) {
        bottomRoller.setControl(voltageControl.withOutput(speed * 12));
    }
}
