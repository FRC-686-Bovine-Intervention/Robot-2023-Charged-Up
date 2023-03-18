package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.lib.sensorCalibration.PotAndEncoder;

public class ArmHAL {
    private static ArmHAL instance;
    public static ArmHAL getInstance() {if(instance == null){instance = new ArmHAL();}return instance;}

    private final TalonSRX turretMotor;
    private final WPI_TalonFX shoulderMotor, elbowMotor;
    private final Solenoid clawSolenoid;

    private static final boolean kClawSolenoidInverted = true;

    private static final int kRelativePIDId = 0;
    private static final int kAbsolutePIDId = 1;
    private static final double kTurretEncoderUnitsToDegrees = 360.0 / 4096.0;
    private static final double kTurretGearRatio = 1; // Gear ratio is 1:1 because of worm gear
    private static final boolean kTurretMotorInverted = false;
    private static final boolean kTurretEncoderInverted = true;
    private static final double kTurretEncoderZeroingCalib = 436;//943;   // value read from encoder when turret is set to 0 degrees
    private static final double kTurretSoftLimitDeg = 225;

    public static final double kArmMotorFullVoltage = 10.0;  // voltage compensation     

    public static final TalonFXInvertType kShoulderMotorInverted    = TalonFXInvertType.Clockwise;          
    public static final TalonFXInvertType kElbowMotorInverted       = TalonFXInvertType.CounterClockwise;    

    public static final double kArmCurrentLimit = 40;
    public static final double kArmTriggerThresholdCurrent = 25;
    public static final double kArmTriggerThresholdTime = 0.1;

   
	// Constant import
	public static final int kTalonTimeoutMs = 5;
	public static final int kTalonPidIdx = 0;  

    private final static double kShoulderPotentiometerGearRatio           = 72.0/16.0;
    private final static double kShoulderEncoderGearRatio                 = 72.0/16.0;
    private final static double kShoulderPotentiometerNTurns              = 5.0;    
    private final static double kShoulderAngleAtCalibration               = -90.0;      // calibrated 2/23 (straight down)
    private final static double kShoulderPotNormalizedVoltageAtCalib      = 0.516;//0.524;//0.522;     // calibrated 2/23
    private final static double kShoulderAbsoluteEncoderAngleDegAtCalib   = 83.68;//88.93;//85.20;     // calibrated 2/23
    private final static boolean kShoulderPotInverted                     = false;
    private final static boolean kShoulderEncInverted                     = false;

    private final PotAndEncoder shoulderPotEncoder;
    private final PotAndEncoder.Config shoulderPotAndEncoderConfig;
    private final PotAndEncoder.HAL shoulderPotAndEncoderHAL;

    private final static double kElbowPotentiometerGearRatio              = 64.0/16.0;
    private final static double kElbowEncoderGearRatio                    = 64.0/16.0;
    private final static double kElbowPotentiometerNTurns                 = 5.0;
    private final static double kElbowAngleAtCalibration                  = 0.0;       // calibrated 2/23 (straight out)
    private final static double kElbowPotNormalizedVoltageAtCalib         = 0.518;//0.442;//0.473;    // calibrated 2/23
    private final static double kElbowAbsoluteEncoderAngleDegAtCalib      = 104.83;//108.28;//108.8;    // calibrated 2/23
    private final static boolean kElbowPotInverted                        = true;
    private final static boolean kElbowEncInverted                        = true;

    
    private final PotAndEncoder elbowPotEncoder;
    private final PotAndEncoder.Config elbowPotAndEncoderConfig;
    private final PotAndEncoder.HAL elbowPotAndEncoderHAL;
        
    private ArmHAL()
    {
        if(RobotBase.isReal())
        {
            turretMotor = new TalonSRX(Constants.kTurretMotorID);
            shoulderMotor = new WPI_TalonFX(Constants.kShoulderMotorID, Constants.kShoulderMotorCanBus);
            elbowMotor    = new WPI_TalonFX(Constants.kElbowMotorID, Constants.kElbowMotorCanBus);    
            shoulderPotAndEncoderHAL = new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, Constants.kShoulderEncoderCanBus, kShoulderPotentiometerNTurns, 
                                                             kShoulderPotentiometerGearRatio, kShoulderPotNormalizedVoltageAtCalib, kShoulderAngleAtCalibration, 
                                                             kShoulderPotInverted, kShoulderEncInverted);            
            elbowPotAndEncoderHAL    = new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, Constants.kElbowEncoderCanBus, kElbowPotentiometerNTurns, 
                                                            kElbowPotentiometerGearRatio, kElbowPotNormalizedVoltageAtCalib, kElbowAngleAtCalibration, 
                                                            kElbowPotInverted, kElbowEncInverted); 
            clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kClawSolenoidID);
        }
        else
        {
            turretMotor = null;
            shoulderMotor = null;
            elbowMotor    = null;            
            shoulderPotAndEncoderHAL = new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, Constants.kShoulderEncoderCanBus, kShoulderPotentiometerNTurns, 
                                                             kShoulderPotentiometerGearRatio, kShoulderPotNormalizedVoltageAtCalib, kShoulderAngleAtCalibration, 
                                                             kShoulderPotInverted, kShoulderEncInverted);            
            elbowPotAndEncoderHAL    = new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, Constants.kElbowEncoderCanBus, kElbowPotentiometerNTurns, 
                                                            kElbowPotentiometerGearRatio, kElbowPotNormalizedVoltageAtCalib, kElbowAngleAtCalibration, 
                                                            kElbowPotInverted, kElbowEncInverted); 
            clawSolenoid = null;
        }

        if (turretMotor != null) {
            turretMotor.configFactoryDefault();
            turretMotor.configOpenloopRamp(1);
            // set quadrature encoder to absolute encoder value
            turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kRelativePIDId, 0);
            turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kAbsolutePIDId, 0);
            syncTurretEncoders();
            // enable turret soft limits
            turretMotor.configForwardSoftLimitThreshold(+kTurretSoftLimitDeg / kTurretEncoderUnitsToDegrees);
            turretMotor.configReverseSoftLimitThreshold(-kTurretSoftLimitDeg / kTurretEncoderUnitsToDegrees);
            turretMotor.configForwardSoftLimitEnable(true);
            turretMotor.configReverseSoftLimitEnable(true);

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
            shoulderMotor.enableVoltageCompensation(false);     // don't enable VoltageCompensation!  This will changes us to 'Simple' neutral deadband mode, which causes oscillation
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
            elbowMotor.enableVoltageCompensation(false);       // don't enable VoltageCompensation!  This will changes us to 'Simple' neutral deadband mode, which causes oscillation
        }
    }

    // Turret
    public ArmHAL setTurretPower(double power) {
        if (turretMotor != null) {
            turretMotor.set(ControlMode.PercentOutput, power);
        }
        return this;
    }

    public ArmHAL setTurretNeutralMode(NeutralMode neutralMode) {
        if (turretMotor != null) {
            turretMotor.setNeutralMode(neutralMode);
        }
        return this;
    }

    public ArmHAL syncTurretEncoders() {
        if(turretMotor != null) {
            turretMotor.setSelectedSensorPosition(
                ((turretMotor.getSelectedSensorPosition(kAbsolutePIDId) - kTurretEncoderZeroingCalib + (4096/2)) % (4096)) - (4096/2), 
                kRelativePIDId, 
                0);
        }
        return this;
    }

    public double getTurretAngleDeg()   {return getTurretRelative() * kTurretGearRatio * kTurretEncoderUnitsToDegrees;}
    public double getTurretRelative()   {return turretMotor != null ? turretMotor.getSelectedSensorPosition(kRelativePIDId) : 0;}
    public double getTurretAbsolute()   {return turretMotor != null ? turretMotor.getSelectedSensorPosition(kAbsolutePIDId) : 0;}

    // Arm

    
    // Shoulder
    public PotAndEncoder getShoulderPotEncoder()    {return shoulderPotEncoder;}

    public double getShoulderFalconSensorPosition() { return shoulderMotor != null ? shoulderMotor.getSelectedSensorPosition(): 0; }
    public void setShoulderFalconSensorPosition(double _units) { if (shoulderMotor != null) {shoulderMotor.setSelectedSensorPosition(_units); }}

    public void disableShoulderSoftLimits() {
        if (shoulderMotor != null) {        
            shoulderMotor.configForwardSoftLimitEnable(false);
            shoulderMotor.configReverseSoftLimitEnable(false);
        }
    }

    public void enableShoulderSoftLimits(double revLimitSensorUnits, double fwdLimitSensorUnits) {
        if (shoulderMotor != null) {        
            shoulderMotor.configForwardSoftLimitThreshold(fwdLimitSensorUnits);
            shoulderMotor.configReverseSoftLimitThreshold(revLimitSensorUnits);
            shoulderMotor.configForwardSoftLimitEnable(true);
            shoulderMotor.configReverseSoftLimitEnable(true);
        }
    }

    public void setShoulderMotorPower(double _power) { if (shoulderMotor != null) { shoulderMotor.set(_power);  }}

    public double getShoulderCurrent()  {return shoulderMotor != null ? shoulderMotor.getSupplyCurrent() : 0;}


    // Elbow
    public PotAndEncoder getElbowPotEncoder()       {return elbowPotEncoder;}   

    public double getElbowFalconSensorPosition() { return elbowMotor != null ? elbowMotor.getSelectedSensorPosition(): 0; }
    public void setElbowFalconSensorPosition(double _units) { if (elbowMotor != null) {elbowMotor.setSelectedSensorPosition(_units); }}

    public void disableElbowSoftLimits() {
        if (elbowMotor != null) {        
            elbowMotor.configForwardSoftLimitEnable(false);
            elbowMotor.configReverseSoftLimitEnable(false);
        }
    }

    public void enableElbowSoftLimits(double revLimitSensorUnits, double fwdLimitSensorUnits) {
        if (elbowMotor != null) {        
            elbowMotor.configForwardSoftLimitThreshold(fwdLimitSensorUnits);
            elbowMotor.configReverseSoftLimitThreshold(revLimitSensorUnits);
            elbowMotor.configForwardSoftLimitEnable(true);
            elbowMotor.configReverseSoftLimitEnable(true);
        }
    }

    public void setElbowMotorPower(double _power) { if (elbowMotor != null) { elbowMotor.set(_power);  }}

    public double getElbowCurrent()  {return elbowMotor != null ? elbowMotor.getSupplyCurrent() : 0;}
        
    // Claw
    public ArmHAL setClawGrabbing(boolean clawGrabbing) {
        if(clawSolenoid != null)
            clawSolenoid.set(clawGrabbing ^ kClawSolenoidInverted); 
        return this;
    }
}

