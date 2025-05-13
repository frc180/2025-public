package frc.robot.commands;

import java.util.function.Function;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RumbleCommand extends Command {

    Function<Double, Double> rumbleFunction = null;
    double rumbleStrength = 0;

    double startTime = 0;

    public RumbleCommand(double rumbleStrength) {
        this.rumbleStrength = rumbleStrength;
    }

    public RumbleCommand(Function<Double, Double> rumbleFunction) {
        this.rumbleFunction = rumbleFunction;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void execute() {
        if (rumbleFunction != null) {
            rumbleStrength = rumbleFunction.apply(Timer.getFPGATimestamp() - startTime);
        }
        RobotContainer.instance.driverController.setRumble(RumbleType.kBothRumble, rumbleStrength);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.instance.driverController.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
