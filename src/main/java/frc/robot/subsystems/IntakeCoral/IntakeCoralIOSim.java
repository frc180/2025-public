package frc.robot.subsystems.IntakeCoral;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class IntakeCoralIOSim implements IntakeCoralIO {
    
    private IntakeSimulation intakeSim = null;

    double rollerSpeed = 0;

    public IntakeCoralIOSim() {
        if (!RobotContainer.MAPLESIM) return;
    
        intakeSim = IntakeSimulation.OverTheBumperIntake(
            "Coral",
            RobotContainer.instance.drivetrain.getDriveSim(),
            Inches.of(15),
            Inches.of(10),
            IntakeSide.BACK,
            1
        );
    }

    // For testing behaviors when the intake fails to pick up a coral (mainly for auto)
    int failIntakeCount = 0;

    @Override
    public void update(IntakeIOInputs inputs) {
        IntakeCoralPivotSubsystem coralIntake = RobotContainer.instance.intakeCoralPivot;
        boolean intakeDeployed = coralIntake.getTargetPosition() == IntakeCoralPivotSubsystem.extend && coralIntake.isAtTarget();
        boolean intakeReady = intakeDeployed && rollerSpeed > 0;

        if (intakeSim != null) {
            if (intakeReady) {
                intakeSim.startIntake();
            } else {
                intakeSim.stopIntake();
            }
            if (!SimLogic.intakeHasCoral) {
                if (failIntakeCount > 0) {
                    if (intakeSim.obtainGamePieceFromIntake()) failIntakeCount--;
                } else {
                    SimLogic.intakeHasCoral = intakeSim.obtainGamePieceFromIntake();
                }
            }
        } else {
            Pose2d coralPose = RobotContainer.instance.vision.getCoralPose();
            if (coralPose != null && rollerSpeed != 0) {
                if (SimLogic.intakeHasCoral && rollerSpeed < 0) {
                    // We have the coral and are ejecting it
                    SimLogic.intakeHasCoral = false;
                } else if (!SimLogic.intakeHasCoral && intakeReady) {
                    // Say we have the coral if we are within 0.75 meters of it
                    Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();
                    double distance = robotPose.getTranslation().getDistance(coralPose.getTranslation());
                    SimLogic.intakeHasCoral = distance <= 0.75;
                }
            }
        }

        inputs.voltage = rollerSpeed * 12;
        inputs.coralDistance = SimLogic.intakeHasCoral ? 0.07 : 3;
        inputs.coralSensorConnected = true;
        inputs.coralSensorStatus = 0;
    }

    @Override
    public void setSpeed(double speed) {
        rollerSpeed = speed;
    }

    @Override
    public void setBottomRollerSpeed(double speed) {}
}
