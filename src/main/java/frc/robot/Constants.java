package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // Drivetrain Hardware
    public final static int kPigeonID =         0;
    public final static int kLeftMasterID =     1;
    public final static int kLeftSlaveID =      2;
    public final static int kRightMasterID =    3;
    public final static int kRightSlaveID =     4;
    // Intake Hardware
    public final static int kRollerMotorID =    5;
    public final static int kIntakeSolenoidID = 0;
    // Arm Hardware
    public final static int kTurretMotorID =    6;
    public final static int kShoulderMotorID =  7;
    public final static int kShoulderAnalogInputPort = 0;
    public final static int kShoulderEncoderId = 8;
    public final static int kElbowMotorID =     8;
    public final static int kElbowAnalogInputPort    = 1;
    public final static int kElbowEncoderId = 8;
    public final static int kClawSolenoidID =   1;
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

