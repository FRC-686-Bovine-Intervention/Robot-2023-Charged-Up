package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // Drivetrain Hardware
    public final static int kPigeonID =         0;

    public final static int kLeftMasterID =     1;
    public final static int kLeftSlaveID =      2;
    public final static int kLeftCANCoderID =   5;

    public final static int kRightMasterID =    3;
    public final static int kRightSlaveID =     4;
    public final static int kRightCANCoderID =  6;

    // Intake Hardware
    public final static int kRollerMotorID =    11;
    public final static int kIntakeSolenoidID = 0;

    // Arm Hardware
    public final static int kTurretMotorID =            21;
    
    public final static int kShoulderMotorID =          22;
    public final static int kShoulderAnalogInputPort =  0;
    public final static int kShoulderEncoderId =        24;

    public final static int kElbowMotorID =             23;
    public final static int kElbowAnalogInputPort =     1;
    public final static int kElbowEncoderId =           25;

    public final static int kClawSolenoidID =           1;

    // Control Hardware
    public final static int kThrustmasterPort =   0;
    public final static int kButtonboardPort =    1;

    // Robot Dimensions
    public final static double kCenterToSideBumper = 15.0;
    public final static double kCenterToFrontBumper = 19.5;
    public final static double kCenterToIntake = 32.0;
}

