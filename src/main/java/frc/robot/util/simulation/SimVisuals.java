package frc.robot.util.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;

public abstract class SimVisuals {

    public static final Mechanism2d mech2d = new Mechanism2d(2, 2);
    private static MechanismRoot2d elevatorRoot, coralArmRoot, coralIntakeRoot;
    private static MechanismLigament2d elevatorLigament, coralIntakeLigament, coralArmLigamentA, coralArmLigamentB;

    private static final double Y_BOTTOM = 0.1; // Represents resting on the robot chassis

    private static final double ELEVATOR_BASE_HEIGHT_METERS = 1.041;
    private static final double ELEVATOR_X = 1;

    private static final double CORAL_ARM_LENGTH = 0.5;
    private static final double CORAL_ARM_Y_OFFSET = -0.7;
    private static final double CORAL_ARM_X = ELEVATOR_X;

    private static final double CORAL_INTAKE_LENGTH = 0.3;
    private static final double CORAL_INTAKE_X = 0.65;

    private static final Color8Bit ELEVATOR_NOCORAL_COLOR = new Color8Bit(Color.kBlue);
    private static final Color8Bit ELEVATOR_CORAL_COLOR = new Color8Bit(Color.kGreen);

    private static final Color8Bit INTAKE_NOCORAL_COLOR = new Color8Bit(Color.kRed);
    private static final Color8Bit INTAKE_CORAL_COLOR = new Color8Bit(Color.kWhite);

    private static final Color8Bit ARM_NOCORAL_COLOR = new Color8Bit(Color.kYellow);
    private static final Color8Bit ARM_CORAL_COLOR = new Color8Bit(Color.kWhite);

    public static void init() {
        elevatorRoot = mech2d.getRoot("Elevator Root", ELEVATOR_X, Y_BOTTOM);
        coralArmRoot = mech2d.getRoot("Coral Arm Root", CORAL_ARM_X, Y_BOTTOM);
        coralIntakeRoot = mech2d.getRoot("Coral Intake Root", CORAL_INTAKE_X, Y_BOTTOM);

        elevatorLigament = elevatorRoot.append(new MechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit(Color.kBlue)));
        coralIntakeLigament = coralIntakeRoot.append(new MechanismLigament2d("Coral Intake", CORAL_INTAKE_LENGTH, 90, 6, new Color8Bit(Color.kRed)));

        coralArmLigamentA = coralArmRoot.append(new MechanismLigament2d("Coral Arm A", CORAL_ARM_LENGTH / 2, 0, 10, new Color8Bit(Color.kYellow)));
        coralArmLigamentB = coralArmRoot.append(new MechanismLigament2d("Coral Arm B", CORAL_ARM_LENGTH / 2, 180, 10, new Color8Bit(Color.kYellow)));

        SmartDashboard.putData("Mechanism 2D", mech2d);
    }

    public static void setElevatorHeight(double heightMeters) {
        elevatorLigament.setLength(ELEVATOR_BASE_HEIGHT_METERS + heightMeters);
    }

    public static void setCoralIntakeDegrees(double degrees) {
        coralIntakeLigament.setAngle(degrees);
    }

    public static void setElevatorArmPivotDegrees(double degrees) {
        coralArmLigamentA.setAngle(degrees);
        coralArmLigamentB.setAngle(degrees + 180);
    }

    /**
     * Updates the simulation visuals - called periodically from Robot.robotPeriodic()
     */
    public static void update() {
        RobotContainer rc = RobotContainer.instance;
        boolean armHasCoral = rc.elevatorArm.hasPartialCoral.getAsBoolean();

        // Keep coral arm attached to elevator
        coralArmRoot.setPosition(CORAL_ARM_X, Y_BOTTOM + CORAL_ARM_Y_OFFSET + elevatorLigament.getLength());

        // Update color of mechanisms based on coral status
        elevatorLigament.setColor(rc.robotHasCoral.getAsBoolean() ? ELEVATOR_CORAL_COLOR : ELEVATOR_NOCORAL_COLOR);
        coralIntakeLigament.setColor(rc.intakeCoral.hasCoral.getAsBoolean() ? INTAKE_CORAL_COLOR : INTAKE_NOCORAL_COLOR);
        coralArmLigamentA.setColor(armHasCoral ? ARM_CORAL_COLOR : ARM_NOCORAL_COLOR);
        coralArmLigamentB.setColor(armHasCoral ? ARM_CORAL_COLOR : ARM_NOCORAL_COLOR);
    }
}
