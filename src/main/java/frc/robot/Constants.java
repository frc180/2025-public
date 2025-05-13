package frc.robot;

// Named constant values for the robot, including electrical mappings
public class Constants {

    public static final String CANIVORE = "Pseudo-Tesla";

    public static final double LOOP_TIME = 0.020;

    // =================== CAN IDs - CANivore ===================

    public static final int ELEVATOR_REAR = 15;
    public static final int ELEVATOR_FRONT = 16;

    public static final int ELEVATOR_ARM_TALON = 19;
    public static final int ELEVATOR_ARM_PIVOT_TALON = 17;

    public static final int INTAKE_ALGAE_TALON = 18;
    public static final int INTAKE_ALGAE_PIVOT_TALON = 13;
    public static final int CLIMBER_WINCH_TALON = 14;

    public static final int ELEVATOR_ARM_ALGAE = 20;

    public static final int ELEVATOR_ARM_CANDI_A = 21;
    public static final int ELEVATOR_ARM_CANDI_B = 22;

    public static final int CORAL_INDEXER = 23;

    // public static final int INTAKE_CORAL_TALON = 24; // removed, is replaced by INTAKE_CORAL_SPARK
    public static final int INTAKE_CORAL_PIVOT_TALON = 25;

    public static final int ALGAE_ARM_CANRANGE = 26; 

    public static final int CANDLE = 27;
    public static final int INTAKE_CORAL_ROLLER_TALON = 28;

    public static final int CLIMBER_GRABBER_TALON = 29;

    // =================== CAN IDs - RIO Bus ===================

    public static final int INTAKE_LASERCAN = 13;
    public static final int INTAKE_CORAL_SPARK = 22;


    // =================== roboRIO sensor channels ===================

    public static final int DIO_INTAKE_ALGAE_ENCODER = 0;
    public static final int DIO_INTAKE_CORAL_ENCODER = 1;
    public static final int DIO_CLIMBER_SENSOR_A = 9;
    public static final int DIO_CLIMBER_SENSOR_B = 2;

    public static final int ANALOG_ELEVATOR_ARM_PIVOT_POTENTIOMETER = 1;
}
