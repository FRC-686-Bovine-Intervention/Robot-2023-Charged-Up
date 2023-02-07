package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // Drivetrain Hardware
    public final static int kPigeonID =           1;
    public final static int kLeftMasterID =       2;
    public final static int kLeftSlaveID =        3;
    public final static int kRightMasterID =      4;
    public final static int kRightSlaveID =       5;
    // Intake Hardware
    public final static int kArmMotorID =         6;
    public final static int kRollerMotorID =      7;
    // Arm Hardware
    public final static int kShoulderAnalogInputPort = 0;
    public final static int kElbowAnalogInputPort    = 1;
    public final static int kShoulderEncoderId = 8;
    public final static int kElbowEncoderId = 8;
    
    // Control Hardware
    public final static int kThrustmasterPort =   0;
    public final static int kButtonboardPort =    1;

    public final static double kLoopDt = 0.01;
    public final static int kTalonTimeoutMs = 5;


    // Robot Dimensions
    public final static double kCenterToSideBumper = 15.0;
    public final static double kCenterToFrontBumper = 19.5;
    public final static double kCenterToIntake = 32.0;
}

