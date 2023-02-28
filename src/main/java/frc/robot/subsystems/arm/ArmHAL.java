package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.lib.sensorCalibration.PotAndEncoder;
import frc.robot.subsystems.arm.json.ArmConfigJson;

public class ArmHAL {
    private static ArmHAL instance;
    public static ArmHAL getInstance() {if(instance == null){instance = new ArmHAL();}return instance;}

    private final TalonSRX turretMotor;
    private final WPI_TalonFX shoulderMotor, elbowMotor;
    private final Solenoid clawSolenoid;

    private static final boolean kClawSolenoidInverted = false;

    private static final double kEncoderUnitsToDegrees = 360.0 / 4096.0;
    private static final double kTurretGearRatio = 1; // Gear ratio is 1:1 because of worm gear
    private static final boolean kTurretMotorInverted = true;
    private static final boolean kTurretEncoderInverted = true;
    private static final int kRelativePIDId = 0;
    private static final int kAbsolutePIDId = 1;

    public static final double kArmMotorFullVoltage = 10.0;  // voltage compensation          

    public static final TalonFXInvertType kShoulderMotorInverted    = TalonFXInvertType.Clockwise;          
    public static final TalonFXInvertType kElbowMotorInverted       = TalonFXInvertType.CounterClockwise;   

    protected final static double kShoulderMotorGearRatio = 4.0 * 4.0 * 72.0/16.0;
    protected final static double kElbowMotorGearRatio = 4.0 * 4.0 * 64.0/16.0;

    private static final double kShoulderEncoderUnitsPerRev = 2048.0 * kShoulderMotorGearRatio;
    private static final double kShoulderEncoderUnitsPerRad = kShoulderEncoderUnitsPerRev / (2*Math.PI);

    private static final double kElbowEncoderUnitsPerRev = 2048.0 * kElbowMotorGearRatio;
    private static final double kElbowEncoderUnitsPerRad = kElbowEncoderUnitsPerRev / (2*Math.PI);    

    public static final double kArmCurrentLimit = 100;
    public static final double kArmTriggerThresholdCurrent = 40;
    public static final double kArmTriggerThresholdTime = 0.5;

    private final double shoulderMaxAngleRad;
    private final double shoulderMinAngleRad;
    private final double elbowMaxAngleRad;
    private final double elbowMinAngleRad;
    private static final double kRelativeMaxAngleRad = Math.toRadians(160.0);    // don't let grabber smash into proximal arm
    private static final double kRelativeMinAngleRad = Math.toRadians(-135.0);   // we'll probably never need this one
   
	// Constant import
	public static final int kTalonTimeoutMs = 5;
	public static final int kTalonPidIdx = 0;  

    private final static double kShoulderPotentiometerGearRatio           = 72.0/16.0;
    private final static double kShoulderEncoderGearRatio                 = 72.0/16.0;
    private final static double kShoulderPotentiometerNTurns              = 5.0;    
    private final static double kShoulderAngleAtCalibration               = -90.0;      // calibrated 2/23 (straight down)
    private final static double kShoulderPotNormalizedVoltageAtCalib      = 0.5135;     // calibrated 2/23
    private final static double kShoulderAbsoluteEncoderAngleDegAtCalib   =  81.37;     // calibrated 2/23
    private final static boolean kShoulderPotInverted                     = false;
    private final static boolean kShoulderEncInverted                     = false;

    private final PotAndEncoder shoulderPotEncoder;
    private final PotAndEncoder.Config shoulderPotAndEncoderConfig;
    private final PotAndEncoder.HAL shoulderPotAndEncoderHAL;

    private final static double kElbowPotentiometerGearRatio              = 64.0/16.0;
    private final static double kElbowEncoderGearRatio                    = 64.0/16.0;
    private final static double kElbowPotentiometerNTurns                 = 5.0;
    private final static double kElbowAngleAtCalibration                  = 0.0;       // calibrated 2/23 (straight out)
    private final static double kElbowPotNormalizedVoltageAtCalib         = 0.4845;    // calibrated 2/23
    private final static double kElbowAbsoluteEncoderAngleDegAtCalib      = 109.25;    // calibrated 2/23
    private final static boolean kElbowPotInverted                        = true;
    private final static boolean kElbowEncInverted                        = true;

    
    private final PotAndEncoder elbowPotEncoder;
    private final PotAndEncoder.Config elbowPotAndEncoderConfig;
    private final PotAndEncoder.HAL elbowPotAndEncoderHAL;
        
    private ArmHAL()
    {
        ArmConfigJson config = Arm.getInstance().getConfig();
        shoulderMinAngleRad = config.shoulder().minAngle();
        shoulderMaxAngleRad = config.shoulder().maxAngle();
        elbowMinAngleRad = config.elbow().minAngle();
        elbowMaxAngleRad = config.elbow().maxAngle();

        if(RobotBase.isReal())
        {
            turretMotor = null;//new TalonSRX(Constants.kTurretMotorID);
            shoulderMotor = new WPI_TalonFX(Constants.kShoulderMotorID);
            elbowMotor    = new WPI_TalonFX(Constants.kElbowMotorID);    
            shoulderPotAndEncoderHAL = new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, kShoulderPotentiometerNTurns, 
                                                             kShoulderPotentiometerGearRatio, kShoulderPotNormalizedVoltageAtCalib, kShoulderAngleAtCalibration, 
                                                             kShoulderPotInverted, kShoulderEncInverted);            
            elbowPotAndEncoderHAL    = new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, kElbowPotentiometerNTurns, 
                                                            kElbowPotentiometerGearRatio, kElbowPotNormalizedVoltageAtCalib, kElbowAngleAtCalibration, 
                                                            kElbowPotInverted, kElbowEncInverted); 
            clawSolenoid = null;//new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kClawSolenoidID);
        }
        else
        {
            turretMotor = null;
            shoulderMotor = null;
            elbowMotor    = null;            
            shoulderPotAndEncoderHAL = new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, kShoulderPotentiometerNTurns, 
                                                             kShoulderPotentiometerGearRatio, kShoulderPotNormalizedVoltageAtCalib, kShoulderAngleAtCalibration, 
                                                             kShoulderPotInverted, kShoulderEncInverted);            
            elbowPotAndEncoderHAL    = new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, kElbowPotentiometerNTurns, 
                                                            kElbowPotentiometerGearRatio, kElbowPotNormalizedVoltageAtCalib, kElbowAngleAtCalibration, 
                                                            kElbowPotInverted, kElbowEncInverted); 
            clawSolenoid = null;
        }

        if (turretMotor != null) {
            turretMotor.configFactoryDefault();
            turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kRelativePIDId, 0);
            turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kAbsolutePIDId, 0);
            if(turretMotor.getSelectedSensorPosition(kRelativePIDId) == 0) // Reset relative to absolute only on power on
                turretMotor.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true);
            turretMotor.setSensorPhase(kTurretEncoderInverted);
            turretMotor.setInverted(kTurretMotorInverted);
        }
        
        shoulderPotAndEncoderConfig = new PotAndEncoder.Config(kShoulderPotentiometerGearRatio, kShoulderEncoderGearRatio, kShoulderPotentiometerNTurns, 
                                                                kShoulderAngleAtCalibration, kShoulderPotNormalizedVoltageAtCalib, kShoulderAbsoluteEncoderAngleDegAtCalib, 
                                                                kShoulderPotInverted, kShoulderEncInverted, shoulderPotAndEncoderHAL);
        elbowPotAndEncoderConfig = new PotAndEncoder.Config(kElbowPotentiometerGearRatio, kElbowEncoderGearRatio, kElbowPotentiometerNTurns, 
                                                                kElbowAngleAtCalibration, kElbowPotNormalizedVoltageAtCalib, kElbowAbsoluteEncoderAngleDegAtCalib, 
                                                                kElbowPotInverted, kElbowEncInverted, elbowPotAndEncoderHAL);
        shoulderPotEncoder = new PotAndEncoder(shoulderPotAndEncoderConfig);
        elbowPotEncoder = new PotAndEncoder(elbowPotAndEncoderConfig);

        if(shoulderMotor != null)
        {
            shoulderMotor.configFactoryDefault();
            
            // Set up the encoders
            shoulderMotor.setInverted(kShoulderMotorInverted);           
            shoulderMotor.setSensorPhase(true);
            shoulderMotor.setNeutralMode(NeutralMode.Brake);
            shoulderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, kArmCurrentLimit, kArmTriggerThresholdCurrent, kArmTriggerThresholdTime));
            shoulderMotor.configVoltageCompSaturation(kArmMotorFullVoltage);
            shoulderMotor.enableVoltageCompensation(true);
        }  

        if(elbowMotor != null)
        {
            elbowMotor.configFactoryDefault();
            
            // Set up the encoders
            elbowMotor.setInverted(kElbowMotorInverted);           
            elbowMotor.setSensorPhase(false);
            elbowMotor.setNeutralMode(NeutralMode.Brake);
            elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, kArmCurrentLimit, kArmTriggerThresholdCurrent, kArmTriggerThresholdTime));
            elbowMotor.configVoltageCompSaturation(kArmMotorFullVoltage);
            elbowMotor.enableVoltageCompensation(true);       }
    }

    // Turret
    public ArmHAL setTurretPower(double power){
        if (turretMotor != null) {
            turretMotor.set(ControlMode.PercentOutput, power);
        }
        return this;
    }

    public double getTurretRelative()   {return turretMotor != null ? turretMotor.getSelectedSensorPosition(kRelativePIDId) * kTurretGearRatio * kEncoderUnitsToDegrees : 0;}
    public double getTurretAbsolute()   {return turretMotor != null ? turretMotor.getSelectedSensorPosition(kAbsolutePIDId) * kTurretGearRatio * kEncoderUnitsToDegrees : 0;}

    // Arm
    boolean shoulderSoftLimitSet = false;
    boolean elbowSoftLimitSet = false;

    // call this every update cycle
    public void setArmMotorSoftLimits() {
        ArmStatus status = ArmStatus.getInstance();
        if (!shoulderSoftLimitSet) {
            if (status.getShoulderStatus().calibrated) {
                double shoulderAngleRad = Units.degreesToRadians(status.getShoulderStatus().positionDeg);
                double revOffset = shoulderAngleRad - shoulderMinAngleRad;
                double fwdOffset = shoulderMaxAngleRad - shoulderAngleRad;
                
                double currentPosInSensorUnits = shoulderMotor.getSelectedSensorPosition();
                shoulderMotor.configForwardSoftLimitThreshold(currentPosInSensorUnits + shoulderRadiansToSensorUnits(fwdOffset));
                shoulderMotor.configReverseSoftLimitThreshold(currentPosInSensorUnits - shoulderRadiansToSensorUnits(revOffset));
                shoulderMotor.configForwardSoftLimitEnable(true);
                shoulderMotor.configReverseSoftLimitEnable(true);

                shoulderSoftLimitSet = true;
            }
        }
        if (!elbowSoftLimitSet) {
            if (status.getElbowStatus().calibrated) {
                double elbowAngleRad = Units.degreesToRadians(status.getElbowStatus().positionDeg);
                double revOffset = elbowAngleRad - elbowMinAngleRad;
                double fwdOffset = elbowMaxAngleRad - elbowAngleRad;
                
                double currentPosInSensorUnits = elbowMotor.getSelectedSensorPosition();
                elbowMotor.configForwardSoftLimitThreshold(currentPosInSensorUnits + elbowRadiansToSensorUnits(fwdOffset));
                elbowMotor.configReverseSoftLimitThreshold(currentPosInSensorUnits - elbowRadiansToSensorUnits(revOffset));
                elbowMotor.configForwardSoftLimitEnable(true);
                elbowMotor.configReverseSoftLimitEnable(true);

                elbowSoftLimitSet = true;
            }
        }
    }
    
    // Shoulder
    public PotAndEncoder getShoulderPotEncoder()    {return shoulderPotEncoder;}

    private boolean isGoodArmAngle() {
        ArmStatus status = ArmStatus.getInstance();
        double shoulderAngleRad = Units.degreesToRadians(status.getShoulderStatus().positionDeg);
        double elbowAngleRad = Units.degreesToRadians(status.getElbowStatus().positionDeg);
        double relativeAngle = elbowAngleRad - shoulderAngleRad;

        return ((shoulderAngleRad >= shoulderMinAngleRad) && (shoulderAngleRad <= shoulderMaxAngleRad) &&
                (elbowAngleRad >= elbowMinAngleRad) && (elbowAngleRad <= elbowMaxAngleRad) &&
                (relativeAngle >= kRelativeMinAngleRad) && (relativeAngle <= kRelativeMaxAngleRad));
    }

    
    public void setShoulderMotorPower(double _power) {
        double power = _power;

        if (shoulderMotor != null)
        {
            ArmStatus status = ArmStatus.getInstance();
            double shoulderAngleRad = Units.degreesToRadians(status.getShoulderStatus().positionDeg);
            double elbowAngleRad = Units.degreesToRadians(status.getElbowStatus().positionDeg);
            double relativeAngle = elbowAngleRad - shoulderAngleRad;
    
            // check forward limits
            if ((shoulderAngleRad > shoulderMaxAngleRad) || (relativeAngle > kRelativeMaxAngleRad)) {
                power = Math.max(power, 0.0);   // still allow movement in reverse direction
            }
            // check reverse limits
            if ((shoulderAngleRad < shoulderMinAngleRad) || (relativeAngle < kRelativeMinAngleRad)) {
                power = Math.min(power, 0.0);   // still allow movement in forward direction
            }
        }
        shoulderMotor.set(power);
    }

    public static double shoulderRadiansToSensorUnits(double _radians) { return _radians * kShoulderEncoderUnitsPerRad; }

    // Elbow
    public PotAndEncoder getElbowPotEncoder()       {return elbowPotEncoder;}   

    public void setElbowMotorPower(double _power) {
        double power = _power;

        if (elbowMotor != null)
        {
            ArmStatus status = ArmStatus.getInstance();
            double shoulderAngleRad = Units.degreesToRadians(status.getShoulderStatus().positionDeg);
            double elbowAngleRad = Units.degreesToRadians(status.getElbowStatus().positionDeg);
            double relativeAngle = elbowAngleRad - shoulderAngleRad;

            // check forward limits
            if ((elbowAngleRad > elbowMaxAngleRad) || (relativeAngle > kRelativeMaxAngleRad)) {
                power = Math.max(power, 0.0);   // still allow movement in reverse direction
            }
            // check reverse limits
            if ((elbowAngleRad < elbowMinAngleRad) || (relativeAngle < kRelativeMinAngleRad)) {
                power = Math.min(power, 0.0);   // still allow movement in forward direction
            }
            elbowMotor.set(power);
        }
    }

    public static double elbowRadiansToSensorUnits(double _radians) { return _radians * kElbowEncoderUnitsPerRad; }

    // Claw
    public ArmHAL setClawGrabbing(boolean clawGrabbing) {
        if(clawSolenoid != null)
            clawSolenoid.set(clawGrabbing ^ kClawSolenoidInverted); 
        return this;
    }
}

